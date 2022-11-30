from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import clip
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai import hyundaicanfd, hyundaican
from selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR
import random
from common.params import Params

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
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0
    self.frame = 0

    self.accel_last = 0
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.pcmCruiseButtonDelay = 0
    self.jerkUpperLowerLimit = 12.0       

  def update(self, CC, CS):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    steer = actuators.steer
    if self.CP.carFingerprint in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022):
      # these cars have significantly more torque than most HKG; limit to 70% of max
      steer = clip(steer, -0.7, 0.7)
    new_steer = int(round(steer * self.params.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    if not CC.latActive:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends = []

    # *** common hyundai stuff ***
    if self.frame % 100 == 0:
      self.jerkUpperLowerLimit = float(int(Params().get("JerkUpperLowerLimit", encoding="utf8")))

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl:
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, 5
      can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

    # >90 degree steering fault prevention
    # Count up to MAX_ANGLE_FRAMES, at which point we need to cut torque to avoid a steering fault
    if CC.latActive and abs(CS.out.steeringAngleDeg) >= MAX_ANGLE:
      self.angle_limit_counter += 1
    else:
      self.angle_limit_counter = 0

    # Cut steer actuation bit for two frames and hold torque with induced temporary fault
    torque_fault = CC.latActive and self.angle_limit_counter > MAX_ANGLE_FRAMES
    lat_active = CC.latActive and not torque_fault

    if self.angle_limit_counter >= MAX_ANGLE_FRAMES + MAX_ANGLE_CONSECUTIVE_FRAMES:
      self.angle_limit_counter = 0

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, CC.enabled, lat_active, apply_steer))

      # disable LFA on HDA2
      if self.frame % 5 == 0 and hda2:
        can_sends.append(hyundaicanfd.create_cam_0x2a4(self.packer, CS.cam_0x2a4))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CP, CC.enabled))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CP, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units))
          self.accel_last = accel
      else:
        # button presses
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
          # cruise cancel
          if CC.cruiseControl.cancel:
            if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
              can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, CS.cruise_info))
              self.last_button_frame = self.frame
            else:
              for _ in range(20):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, CS.buttons_counter+1, Buttons.CANCEL))
              self.last_button_frame = self.frame

          # cruise standstill resume
          elif CC.cruiseControl.resume:
            if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
              # TODO: resume for alt button cars
              pass
            else:
              for _ in range(20):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, CS.buttons_counter+1, Buttons.RES_ACCEL))
              self.last_button_frame = self.frame
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer, lat_active,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning))

      ##HW: 임시로 코드넣음.. 나중에  MDPS 저속(60kmh)개조 된것들은 넣어줘야함.
      if self.frame % 2 == 0 and False: # and self.mdpsMode:
        can_sends.append(hyundaican.create_clu11_mdps(self.packer, self.frame, CS.clu11, Buttons.NONE, self.CP.carFingerprint, 60))

      if not self.CP.openpilotLongitudinalControl:
        if CC.cruiseControl.cancel:
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP.carFingerprint))
        elif CC.cruiseControl.resume:
          # send resume at a max freq of 10Hz
          if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
            # send 25 messages at a time to increases the likelihood of resume being accepted
            can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint)] * 25)
            self.last_button_frame = self.frame
        else:
          target = set_speed_in_units
          current = int(CS.out.cruiseState.speed*CV.MS_TO_KPH + 0.5)

          #CC.debugTextCC = "BTN:00,T:{:.1f},C:{:.1f},{},{}".format(target, current, self.wait_timer, self.alive_timer)
          if CC.enabled and (self.frame - self.last_button_frame)*DT_CTRL > 0.12:
            self.last_button_frame = self.frame
            if not CS.out.cruiseState.enabled:
              if CC.longActive: # and hud_control.leadVisible:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
                #CC.debugTextCC = "BTN:++,T:{:.1f},C:{:.1f}".format(target, current)
              elif CC.longActive:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))
                #CC.debugTextCC = "BTN:--,T:{:.1f},C:{:.1f}".format(target, current)
            elif target < current:
              if current >= 31:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))
                #CC.debugTextCC = "BTN:--,T:{:.1f},C:{:.1f}".format(target, current)
            elif target > current:
              if current < 160:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
                #CC.debugTextCC = "BTN:++,T:{:.1f},C:{:.1f}".format(target, current)

      CC.debugTextCC = "221114a"

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        # TODO: unclear if this is needed
        startingJerk = 1
        if self.CP.carFingerprint in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022):
          startingJerk = 5
        jerk = self.jerkUpperLowerLimit if actuators.longControlState in [LongCtrlState.pid,LongCtrlState.starting] else startingJerk  #comma: jerk=3
        can_sends.extend(hyundaican.create_acc_commands_mix_scc(self.CP, self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      hud_control, set_speed_in_units, stopping, CC, CS))

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.car_fingerprint in (CAR.SONATA, CAR.PALISADE, CAR.IONIQ, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021,
                                                          CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.KIA_CEED, CAR.KIA_SELTOS, CAR.KONA_EV, CAR.KONA_EV_2022,
                                                          CAR.ELANTRA_2021, CAR.ELANTRA_HEV_2021, CAR.SONATA_HYBRID, CAR.KONA_HEV, CAR.SANTA_FE_2022, CAR.SANTA_FE,
                                                          CAR.KIA_K5_2021, CAR.IONIQ_HEV_2022, CAR.SANTA_FE_HEV_2022, CAR.GENESIS_G70_2020, CAR.SANTA_FE_PHEV_2022):
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl: #  and self.CP.sccBus == 0:
        can_sends.extend(hyundaican.create_acc_opt(self.CP, CS, self.packer))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl  and self.CP.sccBus == 0:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
