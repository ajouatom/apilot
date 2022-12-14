import copy
import random
import numpy as np
from common.numpy_fast import clip, interp
from cereal import car
from common.realtime import DT_CTRL
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import Buttons
from common.params import Params
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_MIN, V_CRUISE_DELTA_KM, V_CRUISE_DELTA_MI
from selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
#from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import AUTO_TR_CRUISE_GAP
from selfdrive.car.hyundai.values import CAR
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery
from common.filter_simple import StreamingMovingAverage
from cereal import log

from selfdrive.road_speed_limiter import road_speed_limiter_get_max_speed, road_speed_limiter_get_active, \
  get_road_speed_limiter
#import common.loger as trace1

CREEP_SPEED = 2.3
SYNC_MARGIN = 3.

# do not modify
MIN_SET_SPEED_KPH = V_CRUISE_MIN
MAX_SET_SPEED_KPH = V_CRUISE_MAX

ALIVE_COUNT = [6, 8]
WAIT_COUNT = [12, 13, 14, 15, 16]
AliveIndex = 0
WaitIndex = 0

MIN_CURVE_SPEED = 20. * CV.KPH_TO_MS

EventName = car.CarEvent.EventName

ButtonType = car.CarState.ButtonEvent.Type
ButtonPrev = ButtonType.unknown
ButtonCnt = 0
LongPressed = False

XState = log.LongitudinalPlan.XState

class CruiseHelper:

  def __init__(self):
    self.mapValid = 0
    self.v_cruise_kph_apply = 20
    self.v_cruise_kph_backup = 20
    self.curve_speed_last = 255
    self.longActiveUser = 0
    self.preBrakePressed = False
    self.preGasPressedMax = 0.0
    self.gasPressedCount = 0
    self.curvature = 0
    self.position_x = 1000.0
    self.position_y = 300.0
    self.cruiseButtons = 0
    self.userCruisePaused = True
    self.radarAlarmCount = 0
    self.naviSpeedLimitTarget = 30
    self.dRel = 0
    self.vRel = 0
    self.dRelValidCount = 0
    self.trafficState = 0
    self.xState = XState.cruise

    self.active_cam = False
    self.over_speed_limit = False

    self.roadLimitSpeed = 0.0
    self.ndaActive = 0
    self.trafficSignedFrame = 0

    self.update_params_count = 0

    self.longCruiseGap = 4 #int(Params().get("PrevCruiseGap"))

    self.autoCurveSpeedCtrl = int(Params().get("AutoCurveSpeedCtrl"))
    self.autoCurveSpeedFactor = float(int(Params().get("AutoCurveSpeedFactor", encoding="utf8")))*0.01
    self.autoNaviSpeedCtrl = int(Params().get("AutoNaviSpeedCtrl"))
    self.autoRoadLimitCtrl = int(Params().get("AutoRoadLimitCtrl", encoding="utf8"))
    self.naviSpeedLimitDecelRate = float(Params().get("NaviSpeedLimitDecelRate", encoding="utf8"))*0.01
    #self.naviDecelMarginDist = float(int(Params().get("NaviDecelMarginDist", encoding="utf8")))
    #self.naviDecelRate = float(int(Params().get("NaviDecelRate", encoding="utf8")))
    self.autoResumeFromGasSpeed = float(int(Params().get("AutoResumeFromGasSpeed", encoding="utf8")))
    self.autoResumeFromGas = Params().get_bool("AutoResumeFromGas")
    self.autoResumeFromBrakeRelease = Params().get_bool("AutoResumeFromBrakeRelease")
    self.autoSyncCruiseSpeed = Params().get_bool("AutoSyncCruiseSpeed")
    self.autoResumeFromBrakeReleaseDist = float(int(Params().get("AutoResumeFromBrakeReleaseDist", encoding="utf8")))
    self.autoResumeFromBrakeReleaseLeadCar = Params().get_bool("AutoResumeFromBrakeReleaseLeadCar")
    self.autoResumeFromBrakeCarSpeed = float(int(Params().get("AutoResumeFromBrakeCarSpeed", encoding="utf8")))
    self.autoResumeFromBrakeReleaseTrafficSign  = Params().get_bool("AutoResumeFromBrakeReleaseTrafficSign")
    self.longControlActiveSound = int(Params().get("LongControlActiveSound"))
    #self.accelLimitEcoSpeed = float(int(Params().get("AccelLimitEcoSpeed", encoding="utf8")))
    self.autoSpeedUptoRoadSpeedLimit = float(int(Params().get("AutoSpeedUptoRoadSpeedLimit", encoding="utf8"))) / 100.
    #self.accelLimitConfusedModel = int(Params().get("AccelLimitConfusedModel"))
    self.autoSpeedAdjustWithLeadCar = float(int(Params().get("AutoSpeedAdjustWithLeadCar", encoding="utf8"))) / 1.
    self.cruiseButtonMode = int(Params().get("CruiseButtonMode"))
    self.gapButtonMode = int(Params().get("GapButtonMode"))
    self.autoResumeFromGasSpeedMode = int(Params().get("AutoResumeFromGasSpeedMode"))
    self.myDrivingMode = int(Params().get("InitMyDrivingMode"))
    self.mySafeModeFactor = float(int(Params().get("MySafeModeFactor", encoding="utf8"))) / 100. if self.myDrivingMode == 2 else 1.0
    self.liveSteerRatioApply  = float(int(Params().get("LiveSteerRatioApply", encoding="utf8"))) / 100.
    self.autoCancelFromGas = int(Params().get("AutoCancelFromGas"))

  def update_params(self, frame):
    if frame % 20 == 0:
      self.update_params_count += 1
      self.update_params_count = self.update_params_count % 20

      if self.update_params_count == 0:
        self.autoCurveSpeedCtrl = int(Params().get("AutoCurveSpeedCtrl"))
        self.autoCurveSpeedFactor = float(int(Params().get("AutoCurveSpeedFactor", encoding="utf8")))*0.01
      elif self.update_params_count == 1:
        self.autoNaviSpeedCtrl = int(Params().get("AutoNaviSpeedCtrl"))
        self.autoRoadLimitCtrl = int(Params().get("AutoRoadLimitCtrl", encoding="utf8"))
      elif self.update_params_count == 2:
        self.naviSpeedLimitDecelRate = float(Params().get("NaviSpeedLimitDecelRate", encoding="utf8"))*0.01
        #self.naviDecelMarginDist = float(int(Params().get("NaviDecelMarginDist", encoding="utf8")))
      elif self.update_params_count == 3:
        #self.naviDecelRate = float(int(Params().get("NaviDecelRate", encoding="utf8")))
        self.autoResumeFromGasSpeed = float(int(Params().get("AutoResumeFromGasSpeed", encoding="utf8")))
      elif self.update_params_count == 4:
        self.autoResumeFromGas = Params().get_bool("AutoResumeFromGas")
        self.autoResumeFromBrakeRelease = Params().get_bool("AutoResumeFromBrakeRelease")
      elif self.update_params_count == 5:
        self.autoSyncCruiseSpeed = Params().get_bool("AutoSyncCruiseSpeed")
        self.autoResumeFromBrakeReleaseDist = float(int(Params().get("AutoResumeFromBrakeReleaseDist", encoding="utf8")))
      elif self.update_params_count == 6:
        self.autoResumeFromBrakeReleaseLeadCar = Params().get_bool("AutoResumeFromBrakeReleaseLeadCar")
        self.autoResumeFromBrakeCarSpeed = float(int(Params().get("AutoResumeFromBrakeCarSpeed", encoding="utf8")))
      elif self.update_params_count == 7:
        self.autoResumeFromBrakeReleaseTrafficSign  = Params().get_bool("AutoResumeFromBrakeReleaseTrafficSign")
        self.longControlActiveSound = int(Params().get("LongControlActiveSound"))
      elif self.update_params_count == 8:
        #self.accelLimitEcoSpeed = float(int(Params().get("AccelLimitEcoSpeed", encoding="utf8")))
        self.autoSpeedUptoRoadSpeedLimit = float(int(Params().get("AutoSpeedUptoRoadSpeedLimit", encoding="utf8"))) / 100.
      elif self.update_params_count == 9:
        #self.accelLimitConfusedModel = int(Params().get("AccelLimitConfusedModel"))
        self.autoSpeedAdjustWithLeadCar = float(int(Params().get("AutoSpeedAdjustWithLeadCar", encoding="utf8"))) / 1.
      elif self.update_params_count == 10:
        self.cruiseButtonMode = int(Params().get("CruiseButtonMode"))
        self.autoResumeFromGasSpeedMode = int(Params().get("AutoResumeFromGasSpeedMode"))
      elif self.update_params_count == 11:
        #self.myDrivingMode = int(Params().get("InitMyDrivingMode")) #????????? ????????? ?????????...
        self.mySafeModeFactor = float(int(Params().get("MySafeModeFactor", encoding="utf8"))) / 100. if self.myDrivingMode == 2 else 1.0
        self.liveSteerRatioApply  = float(int(Params().get("LiveSteerRatioApply", encoding="utf8"))) / 100.
      elif self.update_params_count == 12:
        self.autoCancelFromGas = int(Params().get("AutoCancelFromGas"))
        self.gapButtonMode = int(Params().get("GapButtonMode"))

  @staticmethod
  def get_lead(sm):

    radar = sm['radarState']
    if radar.leadOne.status and radar.leadOne.radar:
      return radar.leadOne

    return None

# ajouatom
# navi ????????????
# - ??????????????????(mapValid)??? ???????????? ?????????????????????(v_cruise_kph) ??????(v_cruise_kph_backup)
# - ????????????????????? ????????? ????????? ??????????????? ??????
# - ????????? ???????????? ??????????????? ???????
#   - ????????? ????????????: resume????????? ??????????????? ??????, ???????????????????????? ??????????????? ???.
#   - ????????? ????????????: ???????????????????????????, ???????????????????????? ??????????????? ???.
#   - ??????????????? ????????????: ????????????????????? ??????, resume??? ??????????????? ??????
# - ????????????????????? ????????????
# - ??????????????????? ??????..
# - ????????? ????????????(HW)
#   - ??????????????? ?????????: Cruise disable... 
#   - ??????????????? ????????? ??????: Cruise disable... ??????
#   - ????????? ?????????: 30Km/h???????????? auto Resume
#   - ????????? ????????? ??????: zz
  def cal_curve_speed(self, controls, v_ego, frame, curve_speed_last):
    curve_speed_ms = curve_speed_last
    bCurve = False
    md = controls.sm['modelV2']
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.position.y) == TRAJECTORY_SIZE:
      x = md.position.x
      y = md.position.y
      self.position_x = x[-1]
      self.position_y = y[-1]
      if x[-1] > 100.0 and abs(y[-1]) > 20.0:
        bCurve = True
      else:
        curve_speed_ms = 255.
    else:
      self.position_x = 1000.0
      self.position_y = 300.0

    if bCurve and frame % 20 == 0:
      curvature = self.curvature
      dy = np.gradient(y, x)
      d2y = np.gradient(dy, x)
      curv = d2y / (1 + dy ** 2) ** 1.5

      start = int(interp(v_ego, [10., 27.], [10, TRAJECTORY_SIZE-10]))
      if abs(curvature) > 0.0008: # opkr
        curv = curv[5:TRAJECTORY_SIZE-10]
      else:
        curv = curv[start:min(start+10, TRAJECTORY_SIZE)]
      a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
      v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
      model_speed = np.mean(v_curvature) * 0.85 * self.autoCurveSpeedFactor
      if np.isnan(model_speed):
        model_speed = 0.0

      curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))

      #if model_speed < v_ego:
      #  curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))
      #else:
      #  curve_speed_ms = 255.

      if np.isnan(curve_speed_ms):
        curve_speed_ms = 255.

      #controls.debugText1 = 'CURVE={:5.1f},MODEL={:5.1f},POS={:5.1f},{:5.1f}'.format( curve_speed_ms*CV.MS_TO_KPH, model_speed*CV.MS_TO_KPH, x[TRAJECTORY_SIZE-1], y[TRAJECTORY_SIZE-1])
  
    return curve_speed_ms

  def cruise_control(self, controls, CS, active_mode=0):  #active_mode => -3(OFF auto), -2(OFF brake), -1(OFF user), 0(OFF), 1(ON user), 2(ON gas), 3(ON auto)
    if controls.enabled:
      if active_mode > 0 and controls.CC.longEnabled:
        if self.longActiveUser <= 0:
          controls.LoC.reset(v_pid=CS.vEgo)
        if self.longControlActiveSound >= 2 and self.longActiveUser != active_mode:
          controls.events.add(EventName.cruiseResume)
        self.longActiveUser = active_mode
        self.userCruisePaused = False
      elif active_mode <= 0:
        if self.longActiveUser != active_mode and self.longControlActiveSound >= 2:
          #controls.events.add(EventName.cruisePaused)
          pass
        self.longActiveUser = active_mode

  @staticmethod
  def pcmCruiseControl(v_cruise_kph_last, v_cruise_kph):
    pass

  def get_lead_rel(self, controls):    
    lead = self.get_lead(controls.sm)
    dRel = lead.dRel if lead is not None else 0
    vRel = lead.vRel if lead is not None else 0
    return dRel, vRel

  def update_cruise_buttons(self, enabled, buttonEvents, v_cruise_kph, metric):
    global ButtonCnt, LongPressed, ButtonPrev

    button_speed_up_diff = 1
    button_speed_dn_diff = 10 if self.cruiseButtonMode in [3, 4] else 1

    button_type = 0
    if enabled:
      if ButtonCnt:
        ButtonCnt += 1
      for b in buttonEvents:
        if b.pressed and not ButtonCnt and (b.type == ButtonType.accelCruise or b.type == ButtonType.decelCruise or b.type == ButtonType.gapAdjustCruise):
          ButtonCnt = 1
          ButtonPrev = b.type
        elif not b.pressed and ButtonCnt:
          if not LongPressed and b.type == ButtonType.accelCruise:
            v_cruise_kph += button_speed_up_diff if metric else button_speed_up_diff * CV.MPH_TO_KPH
            button_type = ButtonType.accelCruise
          elif not LongPressed and b.type == ButtonType.decelCruise:
            v_cruise_kph -= button_speed_dn_diff if metric else button_speed_dn_diff * CV.MPH_TO_KPH
            button_type = ButtonType.decelCruise
          elif not LongPressed and b.type == ButtonType.gapAdjustCruise:
            if self.gapButtonMode == 0:
              self.longCruiseGap = 1 if self.longCruiseGap == 4 else self.longCruiseGap + 1
            elif self.gapButtonMode == 1:
              self.longCruiseGap = 1 if self.longCruiseGap == 5 else self.longCruiseGap + 1
            elif self.gapButtonMode == 2:
              self.longCruiseGap = 4 if self.longCruiseGap == 5 else self.longCruiseGap + 1

            #Params().put("PrevCruiseGap", str(self.longCruiseGap))

          LongPressed = False
          ButtonCnt = 0
      if ButtonCnt > 30:
        LongPressed = True
        V_CRUISE_DELTA = V_CRUISE_DELTA_KM if metric else V_CRUISE_DELTA_MI
        if ButtonPrev == ButtonType.accelCruise:
          v_cruise_kph += V_CRUISE_DELTA - v_cruise_kph % V_CRUISE_DELTA
          button_type = ButtonType.accelCruise
          ButtonCnt %= 30
        elif ButtonPrev == ButtonType.decelCruise:
          v_cruise_kph -= V_CRUISE_DELTA - -v_cruise_kph % V_CRUISE_DELTA
          button_type = ButtonType.decelCruise
          ButtonCnt %= 30
        elif ButtonPrev == ButtonType.gapAdjustCruise:
          button_type = ButtonType.gapAdjustCruise
          ButtonCnt = 0
    v_cruise_kph = clip(v_cruise_kph, MIN_SET_SPEED_KPH, MAX_SET_SPEED_KPH)
    return button_type, LongPressed, v_cruise_kph

  def update_speed_navi(self, CS, controls, v_cruise_kph):
    navi = CS.naviSafetyInfo
    #controls.debugText1 = 'S{}/{},D{}/{},N{}'.format(navi.sign, navi.speed2, navi.dist1, navi.dist2, navi.speedLimit)
    v_ego_kph = int(CS.vEgo * CV.MS_TO_KPH + 0.5) + 3.0
    v_kph_apply = v_cruise_kph
    if self.naviSpeedLimitDecelRate > 0.0 and navi.speedLimit > 0 and navi.speedLimit < 255 and v_kph_apply > navi.speedLimit:
      v_kph_apply = min(v_ego_kph, self.naviSpeedLimitTarget)
      self.naviSpeedLimitTarget = max(navi.speedLimit, v_kph_apply - self.naviSpeedLimitDecelRate * 2.5 * DT_CTRL)
    else:
      self.naviSpeedLimitTarget = v_cruise_kph
      return 0
    return self.naviSpeedLimitTarget

  def update_speed_nda(self, CS, controls):
    clu11_speed = CS.vEgoCluster * CV.MS_TO_KPH
    road_speed_limiter = get_road_speed_limiter()
    self.ndaActive = 1 if road_speed_limiter_get_active() > 0 else 0
    apply_limit_speed, road_limit_speed, left_dist, first_started, max_speed_log = \
      road_speed_limiter.get_max_speed(clu11_speed, True) #self.is_metric)

    self.active_cam = road_limit_speed > 0 and left_dist > 0

    if road_speed_limiter.roadLimitSpeed is not None:
      camSpeedFactor = clip(road_speed_limiter.roadLimitSpeed.camSpeedFactor, 1.0, 1.1)
      self.over_speed_limit = road_speed_limiter.roadLimitSpeed.camLimitSpeedLeftDist > 0 and \
                              0 < road_limit_speed * camSpeedFactor < clu11_speed + 2
    else:
      self.over_speed_limit = False

    #str1 = 'applyLimit={},speedLimit={},leftDist={}'.format(apply_limit_speed, road_limit_speed, left_dist)
    #controls.debugText1 = str1
    self.roadLimitSpeed = controls.sm['roadLimitSpeed'].roadLimitSpeed

    return clip(apply_limit_speed, 0, MAX_SET_SPEED_KPH), clip(self.roadLimitSpeed, 30, MAX_SET_SPEED_KPH)

  def update_speed_curve(self, CS, controls):
    curve_speed = self.cal_curve_speed(controls, CS.vEgo, controls.sm.frame, self.curve_speed_last)
    self.curve_speed_last = curve_speed
    return clip(curve_speed * CV.MS_TO_KPH, MIN_SET_SPEED_KPH, MAX_SET_SPEED_KPH)

  def v_cruise_speed_up(self, v_cruise_kph, roadSpeed):
    if v_cruise_kph < roadSpeed:
      v_cruise_kph = roadSpeed
    else:
      for speed in range (40, 150, 10):
        if v_cruise_kph < speed:
          v_cruise_kph = speed
          break
    return clip(v_cruise_kph, MIN_SET_SPEED_KPH, MAX_SET_SPEED_KPH)

  def update_v_cruise_apilot(self, v_cruise_kph, buttonEvents, enabled, metric, controls, CS):
    frame = controls.sm.frame
    self.update_params(frame)
    button,buttonLong,buttonSpeed = self.update_cruise_buttons(enabled, buttonEvents, v_cruise_kph, metric)
    naviSpeed, roadSpeed = self.update_speed_nda(CS, controls)
    carNaviSpeed = self.update_speed_navi(CS, controls, v_cruise_kph)
    
    curveSpeed = self.update_speed_curve(CS, controls) ## longitudinal_control??? ?????????.. ??????????????? ??????..

    v_ego_kph = int(CS.vEgo * CV.MS_TO_KPH + 0.5) + 3.0 #??????????????? v_cruise_kph?????? ?????? ?????? 3??? ??????.
    v_ego_kph_set = clip(v_ego_kph, MIN_SET_SPEED_KPH, MAX_SET_SPEED_KPH)
    xState = controls.sm['longitudinalPlan'].xState
    dRel, vRel = self.get_lead_rel(controls)
    resume_cond = abs(CS.steeringAngleDeg) < 20 # and not CS.steeringPressed
    leadCarSpeed = v_ego_kph + vRel*CV.MS_TO_KPH
    
    #if 2 < dRel < 20:
    #  self.dRelValidCount += 1
    #elif dRel != 0 or self.dRelValidCount < 50 or abs(CS.steeringAngleDeg)<30:
    #  self.dRelValidCount = 0
    #else:
    #  self.dRelValidCount = 0
    #  if self.radarAlarmCount == 0:
    #    self.radarAlarmCount = 2000
    #    #v_cruise_kph = min(v_ego_kph_set, v_cruise_kph) # ???????????? ????????? ???????????? ?????? ??????????????? ?????????.
    
    trafficState = (controls.sm['longitudinalPlan'].trafficState % 100)
    if self.longActiveUser>0:
      if xState != self.xState and xState == XState.softHold:
        controls.events.add(EventName.autoHold)
      if xState == XState.softHold and self.trafficState != 2 and trafficState == 2:
        if (frame - self.trafficSignedFrame)*DT_CTRL > 20.0: # ???????????? ????????? ????????????, ????????? 20?????? ????????? ?????????.
          controls.events.add(EventName.trafficSignChanged)
          self.trafficSignedFrame = frame
        #self.radarAlarmCount = 2000 if self.radarAlarmCount == 0 else self.radarAlarmCount
      elif xState == XState.e2eCruise and self.trafficState != 2 and trafficState == 2 and CS.vEgo < 0.1:
        controls.events.add(EventName.trafficSignGreen)
    self.trafficState = trafficState
    self.dRel = dRel
    self.vRel = vRel
    self.radarAlarmCount = self.radarAlarmCount - 1 if self.radarAlarmCount > 0 else 0

    if controls.enabled:
      ##### Cruise Button ??????...
      if buttonLong:
        if button in [ButtonType.accelCruise, ButtonType.decelCruise]:
          v_cruise_kph = buttonSpeed
          self.v_cruise_kph_backup = v_cruise_kph #?????????????????? ??????
        elif button == ButtonType.gapAdjustCruise:  ##?????????.... ????????? ??????~
          #myDrivingMode = int(Params().get("MyDrivingMode"))
          self.myDrivingMode = self.myDrivingMode + 1 if self.myDrivingMode < 4 else 1
          #Params().put("MyDrivingMode", str(myDrivingMode))
      else:
        self.cruiseButtons = button
        if button == ButtonType.accelCruise:   
          controls.cruiseButtonCounter += 1
          if self.longActiveUser <= 0:
            self.cruise_control(controls, CS, 1)
            v_cruise_kph = max(v_cruise_kph, self.v_cruise_kph_backup, v_ego_kph_set) #??????????????? ????????? ????????? ??????..
          else:
            if xState == XState.softHold:
              self.cruise_control(controls, CS, 1) # SOFT_HOLD????????? ???????????? ???????????? ?????????.
            #elif xState == XState.e2eStop:
            #  pass
            elif self.cruiseButtonMode in [1,2]:
              v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph, roadSpeed)
            else:
              v_cruise_kph = buttonSpeed
          self.v_cruise_kph_backup = v_cruise_kph #?????????????????? ??????
        elif button == ButtonType.decelCruise:
          controls.cruiseButtonCounter -= 1
          if self.longActiveUser <= 0:
            v_cruise_kph = v_ego_kph_set  ## ??????????????? ???????????????
            self.cruise_control(controls, CS, 1)
          else:
            if xState == XState.softHold:
              self.cruise_control(controls, CS, 1) # SOFT_HOLD????????? ???????????? ???????????? ?????????.
            if CS.gasPressed and v_cruise_kph < v_ego_kph_set:
              v_cruise_kph = v_ego_kph_set
              self.v_cruise_kph_backup = v_cruise_kph #?????????????????? ??????
            elif xState == XState.softHold:
              pass
            elif xState == XState.e2eStop and v_ego_kph < 5: #5km/h ??????, ???????????????.. (-)??? ????????? ???????????????... ????????? ???????????????? ??????????????? ??????..
              v_cruise_kph = 3
              self.cruise_control(controls, CS, -1)
              pass
            elif v_cruise_kph > v_ego_kph_set+1 and self.cruiseButtonMode in [1,2]:
              v_cruise_kph = v_ego_kph_set
              self.v_cruise_kph_backup = v_cruise_kph #?????????????????? ??????
            else:
              if self.cruiseButtonMode==2:
                self.userCruisePaused = True
                self.cruise_control(controls, CS, -1)
              else:
                v_cruise_kph = buttonSpeed
                self.v_cruise_kph_backup = v_cruise_kph
      ###### gas, brake?????? ??????...
      if CS.brakePressed:
        self.cruise_control(controls, CS, -2)
        if not self.preBrakePressed:
          self.v_cruise_kph_backup = v_cruise_kph
      elif CS.gasPressed:
        if self.autoCancelFromGas > 0 and v_ego_kph < self.autoCancelFromGas: # ???????????? ???????????? ??????????????? ????????? ??????????????????. ????????? ???????????? ???????????????, ?????????, ????????? ???????????? ??????..
          self.cruise_control(controls, CS, -2)
        elif v_ego_kph > v_cruise_kph and self.autoSyncCruiseSpeed:
          v_cruise_kph = v_ego_kph_set
          self.v_cruise_kph_backup = v_cruise_kph #????????? ?????? ??????
        if xState == XState.softHold: #????????? ?????????????????? ??????????????? ????????? ???????????? ??????~
          self.cruise_control(controls, CS, -2)
        elif xState in [XState.e2eStop, XState.e2eCruise] and v_ego_kph_set < v_cruise_kph and controls.v_future*CV.MS_TO_KPH < v_ego_kph * 0.6: # ????????? ????????? ???????????? ????????? ????????? ????????? ????????? ?????? ??????.
          v_cruise_kph = v_ego_kph_set
          pass
      elif not CS.gasPressed and self.gasPressedCount > 2 and self.longCruiseGap != 5: #????????? ???????????? ??????..
        if False: #CS.myDrivingMode == 2: ???????????? ??????..
          self.cruise_control(controls, CS, -3)
          self.userCruisePaused = True
        else:
          if self.longActiveUser <= 0:
            if ((resume_cond and v_ego_kph >= self.autoResumeFromGasSpeed) or CS.gas >= 0.6) and self.autoResumeFromGas:
              if self.autoResumeFromGasSpeedMode == 0: #??????????????? ??????
                if self.preGasPressedMax > 0.25:
                  v_cruise_kph = self.v_cruise_kph_backup # 25%?????? GAS??? ?????????..
                else:
                  v_cruise_kph = v_ego_kph_set  # ??????????????? ??????~
              elif self.autoResumeFromGasSpeedMode == 1:   #????????????
                 v_cruise_kph = self.v_cruise_kph_backup 
              elif self.autoResumeFromGasSpeedMode == 2:   #???????????? ??????????????? ????????????..
                if dRel > 0:
                  v_cruise_kph = self.v_cruise_kph_backup 
                else:
                  v_cruise_kph = v_ego_kph_set  # ??????????????? ??????~
              self.cruise_control(controls, CS, 3)
          else:
            if self.gasPressedCount * DT_CTRL < 0.6 and v_ego_kph_set > 30.0:  #1???????????? Gas????????? ???????????? ?????????...
              v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph, roadSpeed)

      elif not CS.brakePressed and self.preBrakePressed:
        if v_ego_kph < 5.0 and xState == XState.softHold:
          self.cruise_control(controls, CS, 3)
        elif self.autoResumeFromBrakeRelease and self.longCruiseGap != 5: # ???????????? ????????? ?????? ????????? ON
          if resume_cond and v_ego_kph > 1.0 and self.autoResumeFromBrakeReleaseDist < dRel and self.autoResumeFromBrakeReleaseDist > 0:
            v_cruise_kph = v_ego_kph_set  # ??????????????? ??????~
            self.cruise_control(controls, CS, 3)
          elif v_ego_kph < 60.0 and xState == XState.e2eStop and self.position_y > 5.0 and abs(self.position_y) < 3.0 and self.autoResumeFromBrakeReleaseTrafficSign:
            v_cruise_kph = v_ego_kph_set  
            self.cruise_control(controls, CS, 3)
          elif dRel==0 and v_ego_kph >= self.autoResumeFromBrakeCarSpeed and self.autoResumeFromBrakeCarSpeed > 0:
            v_cruise_kph = v_ego_kph_set
            self.cruise_control(controls, CS, 3)
          elif v_ego_kph < 1.0 and 2 < dRel < 10 and self.autoResumeFromBrakeReleaseLeadCar:
            self.cruise_control(controls, CS, 3)
      elif self.userCruisePaused:
        if v_ego_kph > 3.0 and dRel > 0 and vRel < 0:          
          v_cruise_kph = v_ego_kph_set
          self.cruise_control(controls, CS, 3)
        elif v_ego_kph > 20.0 and xState == XState.e2eStop and abs(self.position_y) < 3.0:
          v_cruise_kph = v_ego_kph_set
          self.cruise_control(controls, CS, 3)
        pass

      ###### ????????? ????????????~~~
      self.v_cruise_kph_apply = v_cruise_kph

      ###### leadCar ?????? ????????????
      roadSpeed1 = roadSpeed * self.autoSpeedUptoRoadSpeedLimit
      if v_cruise_kph < roadSpeed1 and dRel > 0 and vRel > 0 and self.autoSpeedUptoRoadSpeedLimit > 0:
        if leadCarSpeed > v_cruise_kph:
          v_cruise_kph = max(v_cruise_kph, min(leadCarSpeed, roadSpeed1))
          self.v_cruise_kph_apply = v_cruise_kph
      elif self.autoSpeedAdjustWithLeadCar > 0.0 and dRel > 0:
        leadCarSpeed1 = max(leadCarSpeed + self.autoSpeedAdjustWithLeadCar, 30)
        if leadCarSpeed1 < v_cruise_kph:
          self.v_cruise_kph_apply = leadCarSpeed1
      #controls.debugText1 = 'LC={:3.1f},{:3.1f},RS={:3.1f},SS={:3.1f}'.format( leadCarSpeed, vRel*CV.MS_TO_KPH, roadSpeed, self.v_cruise_kph_apply)      

      ###### naviSpeed, roadSpeed, curveSpeed??????
      if self.autoNaviSpeedCtrl > 0 and naviSpeed > 0:
        self.v_cruise_kph_apply = min(self.v_cruise_kph_apply, naviSpeed)
        self.ndaActive = 2 if self.ndaActive == 1 else 0
      elif self.autoNaviSpeedCtrl > 1 and carNaviSpeed > 0:
        self.v_cruise_kph_apply = min(self.v_cruise_kph_apply, carNaviSpeed)
        self.ndaActive = 2
      if roadSpeed > 30 and False: # ????????????????????? ????????????..
        if self.autoRoadLimitCtrl == 1:
          self.v_cruise_kph_apply = min(self.v_cruise_kph_apply, roadSpeed)
        elif self.autoRoadLimitCtrl == 2:
          self.v_cruise_kph_apply = min(self.v_cruise_kph_apply, roadSpeed)
      if self.autoCurveSpeedCtrl==2:
        self.v_cruise_kph_apply = min(self.v_cruise_kph_apply, curveSpeed)
    else: #not enabled
      self.v_cruise_kph_backup = v_cruise_kph #not enabled

    self.preBrakePressed = CS.brakePressed
    self.xState = xState
    if CS.gasPressed:
      self.gasPressedCount += 1
      if CS.gas > self.preGasPressedMax:
        self.preGasPressedMax = CS.gas
      #controls.debugText1 = 'GAS: {:3.1f}/{:3.1f}={:3.1f}'.format(CS.gas*100., self.preGasPressedMax*100., self.gasPressedCount * DT_CTRL)
    else:
      self.preGasPressedMax = 0.0
      self.gasPressedCount = 0
    return v_cruise_kph

def enable_radar_tracks(CP, logcan, sendcan):
  # START: Try to enable radar tracks
  print("Try to enable radar tracks")  
  # if self.CP.openpilotLongitudinalControl and self.CP.carFingerprint in [HYUNDAI_CAR.SANTA_FE_2022]:
  if CP.openpilotLongitudinalControl: # and CP.carFingerprint in [CAR.SANTA_FE, CAR.SANTA_FE_HEV_2022, CAR.NEXO]:
    rdr_fw = None
    rdr_fw_address = 0x7d0 #??????????????? ??????..
    if True:
      for i in range(10):
        print("O yes")
      try:
        for i in range(40):
          try:
            query = IsoTpParallelQuery(sendcan, logcan, CP.sccBus, [rdr_fw_address], [b'\x10\x07'], [b'\x50\x07'], debug=True)
            for addr, dat in query.get_data(0.1).items(): # pylint: disable=unused-variable
              print("ecu write data by id ...")
              new_config = b"\x00\x00\x00\x01\x00\x01"
              #new_config = b"\x00\x00\x00\x00\x00\x01"
              dataId = b'\x01\x42'
              WRITE_DAT_REQUEST = b'\x2e'
              WRITE_DAT_RESPONSE = b'\x68'
              query = IsoTpParallelQuery(sendcan, logcan, CP.sccBus, [rdr_fw_address], [WRITE_DAT_REQUEST+dataId+new_config], [WRITE_DAT_RESPONSE], debug=True)
              query.get_data(0)
              print(f"Try {i+1}")
              break
            break
          except Exception as e:
            print(f"Failed {i}: {e}") 
      except Exception as e:
        print("Failed to enable tracks" + str(e))
  print("END Try to enable radar tracks")
  # END try to enable radar tracks

