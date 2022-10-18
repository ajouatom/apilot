from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import clip
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai import hyundaicanfd, hyundaican
from selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR

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

    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0

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

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and self.CP.openpilotLongitudinalControl and self.CP.sccBus != 2:
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, 5
      can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, CC.enabled, CC.latActive, apply_steer))

      # disable LFA on HDA2
      if self.frame % 5 == 0 and hda2:
        can_sends.append(hyundaicanfd.create_cam_0x2a4(self.packer, CS.cam_0x2a4))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CP, CC.enabled))

      if self.CP.openpilotLongitudinalControl:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CP, CC.enabled, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units))
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
                can_sends.append(hyundaicanfd.create_buttons(self.packer, CS.buttons_counter+1, Buttons.CANCEL))
              self.last_button_frame = self.frame

          # cruise standstill resume
          elif CC.cruiseControl.resume:
            if not (self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
              for _ in range(20):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, CS.buttons_counter+1, Buttons.RES_ACCEL))
              self.last_button_frame = self.frame
    else:
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

      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer, lat_active,
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
            can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint)] * 25)
            self.last_button_frame = self.frame

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        # TODO: unclear if this is needed
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        if self.CP.sccBus != 2:
          can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled and CC.longActive, accel, jerk, int(self.frame / 2),
                                                        hud_control.leadVisible, set_speed_in_units, stopping, CC.cruiseControl.override))
        else:
          can_sends.extend(hyundaican.create_acc_commands_mix_scc(self.CP, self.packer, CC.enabled and CC.longActive, accel, jerk, int(self.frame / 2),
                                                        hud_control.leadVisible, set_speed_in_units, stopping, CC.cruiseControl.override, CS))

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.car_fingerprint in (CAR.SONATA, CAR.PALISADE, CAR.IONIQ, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021,
                                                          CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.KIA_CEED, CAR.KIA_SELTOS, CAR.KONA_EV, CAR.KONA_EV_2022,
                                                          CAR.ELANTRA_2021, CAR.ELANTRA_HEV_2021, CAR.SONATA_HYBRID, CAR.KONA_HEV, CAR.SANTA_FE_2022,
                                                          CAR.KIA_K5_2021, CAR.IONIQ_HEV_2022, CAR.SANTA_FE_HEV_2022, CAR.GENESIS_G70_2020, CAR.SANTA_FE_PHEV_2022):
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled, CC.enabled and CC.longActive))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl  and self.CP.sccBus != 2:
        can_sends.extend(hyundaican.create_acc_opt(self.packer))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl  and self.CP.sccBus != 2:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
