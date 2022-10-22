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

class CruiseHelper:

  def __init__(self):
    self.mapValid = 0
    self.v_cruise_kph_apply = 20
    self.curve_speed_last = 255
    self.longActiveUser = 0
    self.preBrakePressed = False
    self.preGasPressed = False
    self.curvature = 0
    self.position_x = 1000.0
    self.position_y = 300.0
    self.activate_E2E = False
    self.cruiseButtons = 0
    self.userCruisePaused = True
    self.accelLimitEco = 0.6
    self.accelLimitTurn = 0.2
    self.accelBoost = 1.0
    self.autoSpeedUptoRoadSpeed = False
    self.autoSpeedAdjustWithLeadCar = 0

    self.active_cam = False
    self.over_speed_limit = False

    self.roadLimitSpeed = 0.0

    self.update_params(0, True)

  def update_params(self, frame, all):
    if all or frame % 100 == 0:
      self.autoCurveSpeedCtrl = Params().get_bool("AutoCurveSpeedCtrl")
      self.autoCurveSpeedFactor = float(int(Params().get("AutoCurveSpeedFactor", encoding="utf8")))*0.01
      self.autoNaviSpeedCtrl = Params().get_bool("AutoNaviSpeedCtrl")
      self.autoRoadLimitCtrl = int(Params().get("AutoRoadLimitCtrl", encoding="utf8"))
      #self.naviDecelMarginDist = float(int(Params().get("NaviDecelMarginDist", encoding="utf8")))
      #self.naviDecelRate = float(int(Params().get("NaviDecelRate", encoding="utf8")))
    if all or (frame + 30) % 100 == 0:
      self.autoResumeFromGasSpeed = float(int(Params().get("AutoResumeFromGasSpeed", encoding="utf8")))
      self.autoResumeFromGas = Params().get_bool("AutoResumeFromGas")
      self.autoResumeFromBrakeRelease = Params().get_bool("AutoResumeFromBrakeRelease")
      self.autoSyncCruiseSpeed = Params().get_bool("AutoSyncCruiseSpeed")
    if all or (frame + 60) % 100 == 0:
      self.autoResumeFromBrakeReleaseDist = float(int(Params().get("AutoResumeFromBrakeReleaseDist", encoding="utf8")))
      self.autoResumeFromBrakeReleaseLeadCar = Params().get_bool("AutoResumeFromBrakeReleaseLeadCar")
      self.longControlActiveSound = int(Params().get("LongControlActiveSound"))
      self.accelLimitEco = float(int(Params().get("AccelLimitEconomy", encoding="utf8"))) / 100.
      self.accelLimitTurn = float(int(Params().get("AccelLimitTurn", encoding="utf8"))) / 100.
      self.accelBoost = float(int(Params().get("AccelBoost", encoding="utf8"))) / 100.
      self.autoSpeedUptoRoadSpeed = Params().get_bool("AutoSpeedUptoRoadSpeed")
      self.accelLimitConfusedModel = int(Params().get("AccelLimitConfusedModel"))
      self.autoSpeedAdjustWithLeadCar = float(int(Params().get("AutoSpeedAdjustWithLeadCar", encoding="utf8"))) / 100.
      self.cruiseButtonMode = int(Params().get("CruiseButtonMode"))
      
  @staticmethod
  def get_lead(sm):

    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne

    return None

# ajouatom
# navi 운영방법
# - 속도제한신호(mapValid)가 들어오면 현재크루즈속도(v_cruise_kph) 백업(v_cruise_kph_backup)
# - 속도제한신호가 나가면 백업된 크르즈속도 복원
# - 운행중 사용자가 속도변경을 하면?
#   - 버튼을 눌렀을때: resume버튼시 백업속도로 변경, 현재크루즈속도를 백업하도록 함.
#   - 엑셀을 밟았을때: 현재크루즈속도변경, 현재크르주속도를 백업하도록 함.
#   - 브레이크를 밟았을때: 현재크루즈속도 변경, resume시 백업속도로 주행
# - 속도제한신호가 들어오면
# - 구간단속은???? 몰라..
# - 크루즈 운용방법(HW)
#   - 브레이크를 밟으면: Cruise disable... 
#   - 브레이클를 밟았다 떼면: Cruise disable... 유지
#   - 엑셀을 밟으면: 30Km/h이상되면 auto Resume
#   - 엑셀을 밟았다 떼면: zz
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

      controls.debugText1 = 'CURVE={:5.1f},MODEL={:5.1f},POS={:5.1f},{:5.1f}'.format( curve_speed_ms*CV.MS_TO_KPH, model_speed*CV.MS_TO_KPH, x[TRAJECTORY_SIZE-1], y[TRAJECTORY_SIZE-1])
  
    return curve_speed_ms

  def cruise_control(self, controls, CS, active_mode=0):  #active_mode => -3(OFF auto), -2(OFF brake), -1(OFF user), 0(OFF), 1(ON user), 2(ON gas), 3(ON auto)
    if controls.enabled:
      if active_mode > 0:
        if self.longActiveUser <= 0:
          controls.LoC.reset(v_pid=CS.vEgo)
        if self.longControlActiveSound >= 2 and self.longActiveUser != active_mode:
          controls.events.add(EventName.cruiseResume)
        self.longActiveUser = active_mode
        self.userCruisePaused = False
      else:
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
            v_cruise_kph += 1 if metric else 1 * CV.MPH_TO_KPH
            button_type = ButtonType.accelCruise
          elif not LongPressed and b.type == ButtonType.decelCruise:
            v_cruise_kph -= 1 if metric else 1 * CV.MPH_TO_KPH
            button_type = ButtonType.decelCruise
          elif not LongPressed and b.type == ButtonType.gapAdjustCruise:
            pass
          LongPressed = False
          ButtonCnt = 0
      if ButtonCnt > 70:
        LongPressed = True
        V_CRUISE_DELTA = V_CRUISE_DELTA_KM if metric else V_CRUISE_DELTA_MI
        if ButtonPrev == ButtonType.accelCruise:
          v_cruise_kph += V_CRUISE_DELTA - v_cruise_kph % V_CRUISE_DELTA
          button_type = ButtonType.accelCruise
        elif ButtonPrev == ButtonType.decelCruise:
          v_cruise_kph -= V_CRUISE_DELTA - -v_cruise_kph % V_CRUISE_DELTA
          button_type = ButtonType.decelCruise
        ButtonCnt %= 70
    v_cruise_kph = clip(v_cruise_kph, MIN_SET_SPEED_KPH, MAX_SET_SPEED_KPH)
    return button_type, LongPressed, v_cruise_kph

  def update_speed_nda(self, CS, controls):
    clu11_speed = CS.cluSpeedMs * CV.MS_TO_KPH
    road_speed_limiter = get_road_speed_limiter()
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

    return apply_limit_speed, self.roadLimitSpeed

  def update_speed_curve(self, CS, controls):
    curve_speed = self.cal_curve_speed(controls, CS.vEgo, controls.sm.frame, self.curve_speed_last)
    self.curve_speed_last = curve_speed
    return curve_speed * CV.MS_TO_KPH

  def update_v_cruise_apilot(self, v_cruise_kph, buttonEvents, enabled, metric, controls, CS):
    self.activate_E2E = True # E2E모드 항상켬~
    self.update_params(controls.sm.frame, False)
    button,buttonLong,buttonSpeed = self.update_cruise_buttons(enabled, buttonEvents, v_cruise_kph, metric)
    naviSpeed, roadSpeed = self.update_speed_nda(CS, controls)
    curveSpeed = self.update_speed_curve(CS, controls)

    v_ego_kph = int(CS.vEgo * CV.MS_TO_KPH + 0.5) + 3.0 #실제속도가 v_cruise_kph보다 조금 빨라 3을 더함.
    v_ego_kph_set = clip(v_ego_kph, MIN_SET_SPEED_KPH, MAX_SET_SPEED_KPH)
    xState = controls.sm['longitudinalPlan'].xState
    dRel, vRel = self.get_lead_rel(controls)
    resume_cond = abs(CS.steeringAngleDeg) < 20 # and not CS.steeringPressed
    leadCarSpeed = v_ego_kph + vRel*CV.MS_TO_KPH

    if controls.enabled:
      ##### Cruise Button 처리...
      if buttonLong:
        if button in [ButtonType.accelCruise, ButtonType.decelCruise]:
          v_cruise_kph = buttonSpeed
      else:
        self.cruiseButtons = button
        if self.longActiveUser <= 0 and button in [ButtonType.accelCruise, ButtonType.decelCruise]:
          if button == ButtonType.decelCruise:
            v_cruise_kph = v_ego_kph_set  ## 현재속도도 크루즈세트
          self.cruise_control(controls, CS, 1)
        elif button == ButtonType.accelCruise:
          if self.longActiveUser <= 0:
            self.cruise_control(controls, CS, 1)
          else:
            v_cruise_kph = buttonSpeed
        elif button == ButtonType.decelCruise:
          if self.cruiseButtonMode==1:
            self.userCruisePaused = True
            self.cruise_control(controls, CS, -1)
          elif CS.gasPressed and v_cruise_kph < v_ego_kph_set:
            v_cruise_kph = v_ego_kph_set
          else:
            v_cruise_kph = buttonSpeed
      ###### gas, brake관련 처리...
      if CS.brakePressed:
        self.cruise_control(controls, CS, -2)
      elif CS.gasPressed:
        if v_ego_kph > v_cruise_kph and self.autoSyncCruiseSpeed:
          v_cruise_kph = v_ego_kph_set
        if self.longActiveUser <= 0:
          if resume_cond and v_ego_kph >= self.autoResumeFromGasSpeed:
            self.cruise_control(controls, CS, 3)
      elif not CS.brakePressed and self.preBrakePressed and self.autoResumeFromBrakeRelease:
        if resume_cond and v_ego_kph > 3.0 and self.autoResumeFromBrakeReleaseDist < dRel:
          self.cruise_control(controls, CS, 3)
        elif v_ego_kph < 10.0 and xState == "E2E_STOP2":
          self.cruise_control(controls, CS, 3)
        elif v_ego_kph < 60.0 and xState == "E2E_STOP" and abs(self.position_y) < 3.0:
          self.cruise_control(controls, CS, 3)
        elif v_ego_kph >= 40.0 and dRel==0:
          v_cruise_kph = v_ego_kph_set
          self.cruise_control(controls, CS, 3)
        elif v_ego_kph < 1.0 and 2 < dRel < 10 and self.autoResumeFromBrakeReleaseLeadCar:
          self.cruise_control(controls, CS, 3)
      elif self.userCruisePaused:
        if v_ego_kph > 3.0 and dRel > 0 and vRel < 0:
          self.cruise_control(controls, CS, 3)
        elif v_ego_kph > 20.0 and xState == "E2E_STOP" and self.activate_E2E:
          self.cruise_control(controls, CS, 3)
        pass
      elif CS.cruiseGap == 2 and self.preGasPressed == True and not CS.gasPressed:
        self.cruise_control(controls, CS, -3)
        self.userCruisePaused = True

      ###### 크루즈 속도제어~~~
      self.v_cruise_kph_apply = v_cruise_kph

      ###### leadCar 관련 속도처리
      roadSpeed1 = 30 if roadSpeed == 0 else roadSpeed
      if v_cruise_kph < roadSpeed1 and dRel > 0 and vRel > 0 and self.autoSpeedUptoRoadSpeed:
        if leadCarSpeed > v_cruise_kph:
          v_cruise_kph = max(v_cruise_kph, min(leadCarSpeed, roadSpeed1))
          self.v_cruise_kph_apply = v_cruise_kph
      elif self.autoSpeedAdjustWithLeadCar > 0.0 and dRel > 0:
        leadCarSpeed1 = max(leadCarSpeed + self.autoSpeedAdjustWithLeadCar, 30)
        if leadCarSpeed1 < v_cruise_kph:
          self.v_cruise_kph_apply = leadCarSpeed1

      ###### naviSpeed, roadSpeed, curveSpeed처리
      if self.autoNaviSpeedCtrl and naviSpeed > 0:
        self.v_cruise_kph_apply = min(self.v_cruise_kph_apply, naviSpeed)
      if roadSpeed > 0:
        if self.autoRoadLimitCtrl == 1:
          self.v_cruise_kph_apply = min(self.v_cruise_kph_apply, roadSpeed)
        elif self.autoRoadLimitCtrl == 2:
          self.v_cruise_kph_apply = min(self.v_cruise_kph_apply, roadSpeed)
      if self.autoCurveSpeedCtrl:
        self.v_cruise_kph_apply = min(self.v_cruise_kph_apply, curveSpeed)

    self.preBrakePressed = CS.brakePressed
    self.preGasPressed = CS.gasPressed
    return v_cruise_kph

def enable_radar_tracks(CP, logcan, sendcan):
  # START: Try to enable radar tracks
  print("Try to enable radar tracks")
  # if self.CP.openpilotLongitudinalControl and self.CP.carFingerprint in [HYUNDAI_CAR.SANTA_FE_2022]:
  if CP.openpilotLongitudinalControl and CP.carFingerprint in [CAR.SANTA_FE_HEV_2022]:
    rdr_fw = None
    for fw in CP.carFw:
      if fw.ecu == "fwdRadar":
        rdr_fw = fw
        break
    print(f"Found fwdRadar: {rdr_fw.fwVersion}")
    if rdr_fw.fwVersion in [b'\xf1\x8799110S1500\xf1\x00TM__ SCC FHCUP      1.00 1.00 99110-S1500         ', b'TM__ SCC FHCUP      1.00 1.00 99110-S1500 \x04!\x15\x07    ', b'TMhe SCC FHCUP      1.00 1.00 99110-CL500 \x04$\x164    ', b'\xf1\x00TMhe SCC FHCUP      1.00 1.00 99110-CL500         ']:
      for i in range(10):
        print("O yes")
      try:
        for i in range(40):
          try:
            query = IsoTpParallelQuery(sendcan, logcan, 0, [rdr_fw.address], [b'\x10\x07'], [b'\x50\x07'], debug=True)
            for addr, dat in query.get_data(0.1).items(): # pylint: disable=unused-variable
              print("ecu write data by id ...")
              new_config = b"\x00\x00\x00\x01\x00\x01"
              #new_config = b"\x00\x00\x00\x00\x00\x01"
              dataId = b'\x01\x42'
              WRITE_DAT_REQUEST = b'\x2e'
              WRITE_DAT_RESPONSE = b'\x68'
              query = IsoTpParallelQuery(sendcan, logcan, 0, [rdr_fw.address], [WRITE_DAT_REQUEST+dataId+new_config], [WRITE_DAT_RESPONSE], debug=True)
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
