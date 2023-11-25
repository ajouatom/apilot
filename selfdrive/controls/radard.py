#!/usr/bin/env python3
import importlib
import math
from collections import deque
from typing import Optional, Dict, Any

import capnp
from cereal import messaging, log, car
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, Priority, config_realtime_process, DT_MDL
from openpilot.system.swaglog import cloudlog

from openpilot.common.kalman.simple_kalman import KF1D

from openpilot.common.params import Params
from selfdrive.controls.lib.lane_planner import TRAJECTORY_SIZE
import numpy as np
import time

LEAD_PATH_DREL_MIN = 60 # [m] only care about far away leads
MIN_LANE_PROB = 0.6  # Minimum lanes probability to allow use.

#LEAD_PLUS_ONE_MIN_REL_DIST_V = [3.0, 6.0] # [m] min distance between lead+1 and lead at low and high distance
#LEAD_PLUS_ONE_MIN_REL_DIST_BP = [0., 100.] # [m] min distance between lead+1 and lead at low and high distance
#LEAD_PLUS_ONE_MAX_YREL_TO_LEAD = 3.0 # [m]

# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [dt * 0.01 for dt in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[interp(dt, dts, K0)], [interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = _LEAD_ACCEL_TAU
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
    self.dRel = 0
    self.vision_prob = 0.0

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float, a_rel: float, aLeadTau: float, a_ego: float):

    #apilot: changed radar target
    if abs(self.dRel - d_rel) > 3.0: # 3M이상 차이날때 초기화
      self.cnt = 0
      self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.aRel = a_rel   # REL_ACCEL: radar track만 나옴.
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    if True: #a_rel == 0:
      self.aLeadK = float(self.kf.x[ACCEL][0])
    else:
      self.aLeadK = a_rel + a_ego  ## radar track의 A_REL을 사용하도록 함. 값이 약간 더 큼.

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau = aLeadTau#_LEAD_ACCEL_TAU
    else:
      self.aLeadTau *= 0.9

    self.cnt += 1

  def get_key_for_cluster(self):
    # Weigh y higher since radar is inaccurate in this dimension
    return [self.dRel, self.yRel*2, self.vRel]

  def reset_a_lead(self, aLeadK: float, aLeadTau: float):
    self.kf = KF1D([[self.vLead], [aLeadK]], self.K_A, self.K_C, self.K_K)
    self.aLeadK = aLeadK
    self.aLeadTau = aLeadTau

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
      "aRel": float(self.aRel),
    }
  
  def get_RadarState2(self, model_prob, lead_msg, mixRadarInfo):
    useVisionMix = False
    if mixRadarInfo>0 and float(lead_msg.prob) > 0.5 and abs(float(self.aLeadK)) < abs(float(lead_msg.a[0])):
      useVisionMix = True

    #aLeadK = self.aLeadKFilter.process(float(lead_msg.a[0]) if useVisionMix else float(self.aLeadK))
    aLeadK = float(lead_msg.a[0]) if useVisionMix else float(self.aLeadK)
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel) if mixRadarInfo == 0 or self.yRel != 0 else float(-lead_msg.y[0]),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": aLeadK,
      "aLeadTau": float(self.aLeadTau),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
      "aRel": float(self.aRel),
    }


  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: Dict[int, Track]):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  tracks_len = len(tracks)

  def prob(c, c_key):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    #속도가 빠른것에 weight를 더줌. apilot, 
    #231120: 감속정지중, 전방차량정지상태인데, 주변의 노이즈성 레이더포인트가 검출되어 버림. 
    #       이 포인트는 약간 먼데, 주행하고 있는것처럼.. 인식됨. 아주 잠깐 인식됨.
    #        속도관련 weight를 없애야하나?  TG를 지나고 있는 차를 우선시하도록 만든건데...
    weight_v = interp(c.vRel + v_ego, [0, 10], [0.3, 1])
    # This is isn't exactly right, but good heuristic
    prob = prob_d * prob_y * prob_v * weight_v
    c.vision_prob = prob
    return prob

  track_key, track = max(tracks.items(), key=lambda item: prob(item[1], item[0]))
  #track = max(tracks.values(), key=prob)

  #prob_values = {track_id: prob(track) for track_id, track in tracks.items()}
  #max_prob_track_id = max(prob_values, key=prob_values.get)
  #max_prob_value = prob_values[max_prob_track_id]
  #track = tracks.get(max_prob_track_id)
  # yRel값은 왼쪽이 +값, lead.y[0]값은 왼쪽이 -값

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.35, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 15) or (v_ego + track.vRel > 3)
  ##간혹 어수선한경우, 전방에 차가 없지만, 좌우에 차가많은경우 억지로 레이더를 가져오는 경우가 있음..(레이더트랙의 경우)
  y_sane = (abs(-lead.y[0]-track.yRel) < 3.2 / 2.)  #lane_width assumed 3.2M, laplacian_pdf 의 prob값을 검증하려했지만, y값으로 처리해도 될듯함.
  if dist_sane and vel_sane and y_sane:
    return track
  else:
    return None

def get_path_adjacent_leads(v_ego, md, lane_width, clusters, mixRadarInfo):
  if len(clusters) == 0:
    return [[],[],[]]
  
  if md is not None and lane_width > 0. and len(md.laneLines) == 4 and len(md.laneLines[1].x) == TRAJECTORY_SIZE:
    # get centerline approximation using one or both lanelines
    ll_x = md.laneLines[1].x  # left and right ll x is the same
    lll_y = np.array(md.laneLines[1].y)
    rll_y = np.array(md.laneLines[2].y)
    l_prob = md.laneLineProbs[1]
    r_prob = md.laneLineProbs[2]

    # Find path from lanes as the average center lane only if min probability on both lanes is above threshold.
    if l_prob > MIN_LANE_PROB and r_prob > MIN_LANE_PROB:
      c_y = (lll_y + rll_y) / 2.
    elif l_prob > MIN_LANE_PROB:
      c_y = lll_y + (lane_width / 2)
    elif r_prob > MIN_LANE_PROB:
      c_y = rll_y - (lane_width / 2)
    else:
      c_y = None
  else:
    c_y = None
  
  if md is not None or len(md.position.x) == TRAJECTORY_SIZE or md.position.x[-1] > LEAD_PATH_DREL_MIN:
    md_y = md.position.y
    md_x = md.position.x
  else:
    md_y = None
  
  leads_left = {}
  leads_center = {}
  leads_right = {}
  half_lane_width = lane_width / 2
  for c in clusters.values():
    if md_y is not None and c.dRel <= md_x[-1] or (c_y is not None and md_x[-1] - c.dRel < ll_x[-1] - c.dRel):
      dPath = -c.yRel - interp(c.dRel, md_x, md_y)
      checkSource = 'modelPath'
    elif c_y is not None:
      dPath = -c.yRel - interp(c.dRel, ll_x, c_y.tolist())
      checkSource = 'modelLaneLines'
    else:
      dPath = -c.yRel
      checkSource = 'lowSpeedOverride'
      
    source = 'vision' if c.dRel > 145. else 'radar'
    
    #ld = c.get_RadarState(source=source, checkSource=checkSource)
    #ld = c.get_RadarState()
    lead_msg = md.leadsV3[0]
    ld = c.get_RadarState2(lead_msg.prob, lead_msg, mixRadarInfo)
    ld["dPath"] = dPath
    ld["vLat"] = math.sqrt((10*dPath)**2 + c.dRel**2)
    if abs(dPath) < half_lane_width:
      if True: #ld["vLeadK"] > -1.:
        leads_center[abs(dPath)] = ld
    elif dPath < 0.:
      leads_left[abs(dPath)] = ld
    else:
      leads_right[abs(dPath)] = ld
  
  ll,lr = [[l[k] for k in sorted(list(l.keys()))] for l in [leads_left,leads_right]]
  lc = sorted(leads_center.values(), key=lambda c:c["dRel"])
  return [ll,lc,lr]

def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]), #0.0,
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: Dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, low_speed_override: bool = True, mixRadarInfo=0) -> Dict[str, Any]:
  ## SCC레이더는 일단 보관하고 리스트에서 삭제...
  track_scc = tracks.get(0)
  if track_scc is not None:
    del tracks[0]
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  ## vision match후 발견된 track이 없으면
  ##  track_scc 가 있는 지 확인하고
  ##    비전과의 차이가 35%(5M)이상 차이나면 scc가 발견못한것이기 때문에 비전것으로 처리함.
  if track_scc is not None and track is None:
    track = track_scc
    if lead_msg.prob > .5:
      offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
      if offset_vision_dist < track.dRel - 5.0: #끼어드는 차량이 있는 경우 처리..
        track = None

    mixRadarInfo = 0 # 비젼검출이 안된것이므로, mix는 사용안하게함.

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState2(lead_msg.prob, lead_msg, mixRadarInfo)
  elif (track is None) and ready and (lead_msg.prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState2(lead_msg.prob, lead_msg, mixRadarInfo)

  return lead_dict

def get_lead_side(v_ego, tracks, md, lane_width):
  ## SCC레이더는 일단 보관하고 리스트에서 삭제...
  track_scc = tracks.get(0)
  if track_scc is not None:
    del tracks[0]

  if len(tracks) == 0:
    return [[],[],[]]
  if md is not None and len(md.lateralPlannerSolution.x) == TRAJECTORY_SIZE:
    md_y = md.lateralPlannerSolution.y
    md_x = md.lateralPlannerSolution.x
  else:
    return [[],[],[]]

  leads_center = {}
  leads_left = {}
  leads_right = {}
  for c in tracks.values():
    # d_y :  path_y - traks_y 의 diff값
    d_y = -c.yRel - interp(c.dRel, md_x, md_y)
    ld = c.get_RadarState()
    if abs(d_y) < lane_width/2:
      leads_center[c.dRel] = ld
    elif d_y < 0:
      leads_left[c.dRel] = ld
    else:
      leads_right[c.dRel] = ld

  ll,lr = [[l[k] for k in sorted(list(l.keys()))] for l in [leads_left,leads_right]]
  lc = sorted(leads_center.values(), key=lambda c:c["dRel"])
  return [ll,lc,lr]
  #return [leads_left, leads_center, leads_right]

def match_vision_track_apilot(v_ego, lead_msg, tracks, md, lane_width):
  offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA

  ## SCC레이더는 일단 보관하고 리스트에서 삭제...
  track_scc = tracks.get(0)
  if track_scc is not None:
    del tracks[0]

  ## SCC레이더 개조는 track이 1개밖에 없으니, 항상 SCC레이더는 사용한다.
  if len(tracks) > 0:
    if md is not None and len(md.lateralPlannerSolution.x) == TRAJECTORY_SIZE:
      md_y = md.lateralPlannerSolution.y
      md_x = md.lateralPlannerSolution.x
    else:
      return track_scc

    tracks_center = {}
    tracks_left = {}
    tracks_right = {}
    for track_id, c in tracks.items():
      lp_y = interp(c.dRel, md_x, md_y)
      d_y = -c.yRel - lp_y
      lw = interp(c.dRel, [0, 10, 100], [1.5, lane_width, lane_width*0.6] )
      if abs(d_y) < lw / 2:
        tracks_center[track_id] = c
      elif d_y < 0:
        tracks_left[track_id] = c
      else:
        tracks_right[track_id] = c

    if lead_msg.prob > .5: # 비젼감지된경우
      # 정지된 차량의 감지... : 감지된 비젼은 움직이는 것으로 나온다. 레이더는 정지된것으로... 10m/s이상의 차이로 match fail된다.
      # 선행차가 지나가고 있는 차량 위의 신호등이나, TG지붕등 감지를 안하게 하려면?
      #     속도가 빠른것만 먼저고름:
      #     없으면: 정지된 차량들을 골라야함. 비젼검출위주로 봐야할듯..

      ## 1. 비젼거리오차&속도오차에 검출된것을 고름
      tracks_match = {track_id: track for track_id, track in tracks_center.items() 
                        if (abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.35, 5.0])) 
                        and (abs(track.vRel + v_ego - lead_msg.v[0]) < 10)}
      if tracks_match:
        min_track_id = min(tracks_match, key=lambda id:tracks_match[id].dRel)
        return tracks_match[min_track_id]
      ## 2. 비젼거리오차에 해당되는것이 있다면... 속도가 가장 빠른것으로 선택함..
      else:
        tracks_match = {track_id: track for track_id, track in tracks_center.items() 
                          if (abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.35, 5.0])) 
                          and (track.vRel + v_ego) > -0.5}
        if tracks_match:
          min_track_id = max(tracks_match, key=lambda id:tracks_match[id].vRel)
          return tracks_match[min_track_id]
      if track_scc is not None:
        if offset_vision_dist > track_scc.dRel - 5.0: #끼어드는 차량이 있는 경우 비젼으로 처리하도록 삭제..
          track_scc = None
      return track_scc
 
    tracks_fast = {track_id: track for track_id, track in tracks_center.items() if track.vRel+v_ego > 3.0}
    if tracks_fast:
      min_track_id = min(tracks_fast, key=lambda id:tracks_fast[id].dRel)
      return tracks_fast[min_track_id]

  ## 검출된 레이더가 없는데.. 
  return track_scc

def get_lead_apilot(v_ego, ready, tracks, lead_msg, model_v_ego, md, lane_width):
  if len(tracks) > 0 and ready:
    track = match_vision_track_apilot(v_ego, lead_msg, tracks, md, lane_width)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState2(lead_msg.prob, lead_msg, mixRadarInfo=False)
  elif lead_msg.prob > .5:
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)
  
  return lead_dict


class RadarD:
  def __init__(self, radar_ts: float, delay: int = 0):
    self.current_time = 0.0

    self.tracks: Dict[int, Track] = {}
    self.kalman_params = KalmanParams(radar_ts)

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=delay+1)

    self.radar_state: Optional[capnp._DynamicStructBuilder] = None
    self.radar_state_valid = False

    self.ready = False
    self.showRadarInfo = False
    self.mixRadarInfo = 0
    self.aLeadTau = 1.5
    self.a_ego = 0.0

  def update(self, sm: messaging.SubMaster, rr: Optional[car.RadarData]):
    self.showRadarInfo = int(Params().get("ShowRadarInfo"))
    self.mixRadarInfo = int(Params().get("MixRadarInfo"))
    self.aLeadTau = float(Params().get_int("ALeadTau")) / 100. 

    self.current_time = 1e-9*max(sm.logMonoTime.values())

    radar_points = []
    radar_errors = []
    if rr is not None:
      radar_points = rr.points
      radar_errors = rr.errors
    if sm.updated['carState']:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.a_ego = sm['carState'].aEgo
    if sm.updated['modelV2']:
      self.ready = True

    ar_pts = {}
    for pt in radar_points:
      ar_pts[pt.trackId] = [pt.dRel, pt.yRel, pt.vRel, pt.measured, pt.aRel]

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], rpt[4], self.aLeadTau, self.a_ego)

    print(self.tracks)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks() and len(radar_errors) == 0
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = list(radar_errors)
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].temporalPose.trans):
      model_v_ego = sm['modelV2'].temporalPose.trans[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      if self.mixRadarInfo == 2:
        self.radar_state.leadOne = get_lead_apilot(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, sm['modelV2'], sm['lateralPlan'].laneWidth)
        self.radar_state.leadTwo = get_lead_apilot(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, sm['modelV2'], sm['lateralPlan'].laneWidth)
      else:
        self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, low_speed_override=False, mixRadarInfo=self.mixRadarInfo)
        self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, low_speed_override=False, mixRadarInfo=self.mixRadarInfo)

      if self.ready and self.showRadarInfo: #self.extended_radar_enabled and self.ready:
        ll,lc,lr = get_lead_side(self.v_ego, self.tracks, sm['modelV2'], sm['lateralPlan'].laneWidth)
        self.radar_state.leadsLeft = list(ll)
        self.radar_state.leadsCenter = list(lc)
        self.radar_state.leadsRight = list(lr)

  def publish(self, pm: messaging.PubMaster, lag_ms: float):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    radar_msg.radarState.cumLagMs = lag_ms
    pm.send("radarState", radar_msg)

    # publish tracks for UI debugging (keep last)
    tracks_msg = messaging.new_message('liveTracks', len(self.tracks))
    for index, tid in enumerate(sorted(self.tracks.keys())):
      tracks_msg.liveTracks[index] = {
        "trackId": tid,
        "dRel": float(self.tracks[tid].dRel),
        "yRel": float(self.tracks[tid].yRel),
        "vRel": float(self.tracks[tid].vRel),
        "aRel": float(self.tracks[tid].aRel),
      }
    pm.send('liveTracks', tracks_msg)


# fuses camera and radar data for best lead detection
def radard_thread(sm: Optional[messaging.SubMaster] = None, pm: Optional[messaging.PubMaster] = None, can_sock: Optional[messaging.SubSocket] = None):
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  with car.CarParams.from_bytes(Params().get("CarParams", block=True)) as msg:
    CP = msg
  cloudlog.info("radard got CarParams")

  # import the radar from the fingerprint
  cloudlog.info("radard is importing %s", CP.carName)
  RadarInterface = importlib.import_module(f'selfdrive.car.{CP.carName}.radar_interface').RadarInterface

  # *** setup messaging
  if can_sock is None:
    can_sock = messaging.sub_sock('can')
  if sm is None:
    sm = messaging.SubMaster(['modelV2', 'carState', 'lateralPlan'], ignore_avg_freq=['modelV2', 'carState', 'lateralPlan'])
  if pm is None:
    pm = messaging.PubMaster(['radarState', 'liveTracks'])

  RI = RadarInterface(CP)

  rk = Ratekeeper(1.0 / CP.radarTimeStep, print_delay_threshold=None)
  RD = RadarD(CP.radarTimeStep, RI.delay)

  #now = time.monotonic()
  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    rr = RI.update(can_strings)

    if rr is None:
      continue

    sm.update(0)

    RD.update(sm, rr)
    RD.publish(pm, -rk.remaining*1000.0)

    #print("{:.3f}{:.3f}".format(CP.radarTimeStep, time.monotonic() - now))
    #now = time.monotonic()
    #rk.keep_time()
    rk.monitor_time()


def main(sm: Optional[messaging.SubMaster] = None, pm: Optional[messaging.PubMaster] = None, can_sock: messaging.SubSocket = None):
  radard_thread(sm, pm, can_sock)


if __name__ == "__main__":
  main()
