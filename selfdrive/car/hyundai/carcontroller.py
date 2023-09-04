from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, common_fault_avoidance
from openpilot.selfdrive.car.hyundai import hyundaicanfd, hyundaican
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR

import random
from random import randint
from openpilot.common.params import Params

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0
    self.frame = 0

    self.accel_last = 0
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    #self.send_lfa_mfa_lkas = True if self.car_fingerprint in FEATURES["send_lfa_mfa"] and self.car_fingerprint not in [CAR.HYUNDAI_GENESIS] else False
    self.send_lfa_mfa_lkas = CP.flags & HyundaiFlags.SEND_LFA.value
    self.last_button_frame = 0
    self.pcmCruiseButtonDelay = 0
    self.jerkUpperLowerLimit = 12.0       
    self.speedCameraHapticEndFrame = 0
    self.hapticFeedbackWhenSpeedCamera = 0
    self.maxAngleFrames = MAX_ANGLE_FRAMES
    self.softHoldMode = 1
    self.blinking_signal = False #아이콘 깜박이용 1Hz
    self.blinking_frame = int(1.0 / DT_CTRL)
    self.steerDeltaUp = 3
    self.steerDeltaDown = 7
    self.button_wait = 12

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    self.params.STEER_DELTA_UP = self.steerDeltaUp
    self.params.STEER_DELTA_DOWN = self.steerDeltaDown
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, self.maxAngleFrames,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_steer = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_steer_last = apply_steer

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    if CC.activeHda == 2 and self.speedCameraHapticEndFrame < 0: # 과속카메라 감속시작
      self.speedCameraHapticEndFrame = self.frame + (8.0 / DT_CTRL)  #6초간 켜줌..
    elif CC.activeHda != 2:
      self.speedCameraHapticEndFrame = -1

    if self.frame < self.speedCameraHapticEndFrame and self.hapticFeedbackWhenSpeedCamera>0:
      haptic_stop = (self.speedCameraHapticEndFrame - (5.0/DT_CTRL)) < self.frame < (self.speedCameraHapticEndFrame - (3.0/DT_CTRL))
      if not haptic_stop:
         left_lane_warning = right_lane_warning = self.hapticFeedbackWhenSpeedCamera 
     
    if self.frame % self.blinking_frame == 0:
      self.blinking_signal = True
    elif self.frame % self.blinking_frame == self.blinking_frame / 2:
      self.blinking_signal = False

    can_sends = []

    # *** common hyundai stuff ***
    if self.frame % 100 == 0:
      self.jerkUpperLowerLimit = float(int(Params().get("JerkUpperLowerLimit", encoding="utf8")))
      self.hapticFeedbackWhenSpeedCamera = int(Params().get("HapticFeedbackWhenSpeedCamera", encoding="utf8"))
      self.maxAngleFrames = int(Params().get("MaxAngleFrames", encoding="utf8"))
      self.softHoldMode = int(Params().get("SoftHoldMode", encoding="utf8"))
      self.steerDeltaUp = int(Params().get("SteerDeltaUp", encoding="utf8"))
      self.steerDeltaDown = int(Params().get("SteerDeltaDown", encoding="utf8"))

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append([0x7b1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", self.CAN.ECAN])

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_steer))

      # prevent LFA from activating on HDA2 by sending "no lane lines detected" to ADAS ECU
      if self.frame % 5 == 0 and hda2:
        can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.hda2_lfa_block_msg,
                                                          self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

      # blinkers
      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units))
          self.accel_last = accel
      else:
        # button presses
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, self.send_lfa_mfa_lkas, apply_steer, apply_steer_req,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning))

      if not self.CP.openpilotLongitudinalControl:
        if CC.cruiseControl.cancel:
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP.carFingerprint))
        elif CC.cruiseControl.resume:
          # send resume at a max freq of 10Hz
          if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
            # send 25 messages at a time to increases the likelihood of resume being accepted
            #can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint)] * 25)
            can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
            self.last_button_frame = self.frame
        else:
          target = int(set_speed_in_units+0.5)
          current = int(CS.out.cruiseState.speed*CV.MS_TO_KPH + 0.5)

          #CC.debugTextCC = "BTN:00,T:{:.1f},C:{:.1f},{},{}".format(target, current, self.wait_timer, self.alive_timer)
          if CC.enabled and (self.frame - self.last_button_frame) > self.button_wait and CS.cruise_buttons[-1] == Buttons.NONE:
            self.button_wait = randint(7,15)
            self.last_button_frame = self.frame
            if not CS.out.cruiseState.enabled:
              if CC.longActive: # and hud_control.leadVisible:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
                #CC.debugTextCC = "BTN:++,T:{:.1f},C:{:.1f}".format(target, current)
              elif CC.longActive:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))
                #CC.debugTextCC = "BTN:--,T:{:.1f},C:{:.1f}".format(target, current)
              elif CS.out.cruiseGap != hud_control.cruiseGap:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP.carFingerprint))
                #print("currentGap = {}, target = {}".format(CS.out.cruiseGap, hud_control.cruiseGap))
            elif CS.out.cruiseGap != hud_control.cruiseGap:
              can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP.carFingerprint))
              #print("currentGap = {}, target = {}".format(CS.out.cruiseGap, hud_control.cruiseGap))
            elif target < current and current>= 31:
              can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))
              #CC.debugTextCC = "BTN:--,T:{:.1f},C:{:.1f}".format(target, current)
            elif target > current and current < 160:
              can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
              #CC.debugTextCC = "BTN:++,T:{:.1f},C:{:.1f}".format(target, current)

      CC.debugTextCC = "230206"

      if self.CP.carFingerprint in (CAR.GENESIS_G90_2019, CAR.GENESIS_G90, CAR.K7):
        can_sends.append(hyundaican.create_mdps12(self.packer, self.frame, CS.mdps12))

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        if False: #sunnyhaibin, smoother stop test
          stopping = stopping and CS.out.vEgoRaw < 0.05
        # TODO: unclear if this is needed
        startingJerk = 1
        if self.CP.carFingerprint in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022):
          startingJerk = 5        
        if actuators.jerk <= 0:
          jerk_l = max(1, - actuators.jerk * 2)
          jerk_u = 0
        else:
          jerk = self.jerkUpperLowerLimit if actuators.longControlState in [LongCtrlState.pid,LongCtrlState.stopping] else startingJerk  #comma: jerk=1
          jerk_u = jerk #actuators.jerk *3
          jerk_l = 0.5 # jerk #0 if actuators.jerk > 0.5 else 1.0
        can_sends.extend(hyundaican.create_acc_commands_mix_scc(self.CP, self.packer, CC.enabled, accel, jerk_u, jerk_l, int(self.frame / 2),
                                                      hud_control, set_speed_in_units, stopping, CC, CS, self.softHoldMode))
        self.accel_last = accel

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC, self.blinking_signal))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl: 
        if self.CP.sccBus == 0:
          can_sends.extend(hyundaican.create_acc_opt(self.CP, CS, self.packer))
        elif CS.scc13 is not None:
          can_sends.append(hyundaican.create_acc_opt_copy(self.CP, CS, self.packer))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl  and self.CP.sccBus == 0:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def create_button_messages(self, CC: car.CarControl, CS: car.CarState, use_clu11: bool):
    can_sends = []
    if use_clu11:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP.carFingerprint))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends
