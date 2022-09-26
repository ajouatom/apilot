from cereal import car
from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car.nissan import nissancan
from selfdrive.car.nissan.values import CAR, CarControllerParams

VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint
    self.frame = 0

    self.lkas_max_torque = 0
    self.last_angle = 0

    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    ### STEER ###
    lkas_hud_msg = CS.lkas_hud_msg
    lkas_hud_info_msg = CS.lkas_hud_info_msg
    apply_angle = actuators.steeringAngleDeg

    steer_hud_alert = 1 if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw) else 0

    if CC.latActive:
      # windup slower
      if self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle):
        angle_rate_lim = interp(CS.out.vEgo, CarControllerParams.ANGLE_DELTA_BP, CarControllerParams.ANGLE_DELTA_V)
      else:
        angle_rate_lim = interp(CS.out.vEgo, CarControllerParams.ANGLE_DELTA_BP, CarControllerParams.ANGLE_DELTA_VU)

      apply_angle = clip(apply_angle, self.last_angle - angle_rate_lim, self.last_angle + angle_rate_lim)

      # Max torque from driver before EPS will give up and not apply torque
      if not bool(CS.out.steeringPressed):
        self.lkas_max_torque = CarControllerParams.LKAS_MAX_TORQUE
      else:
        # Scale max torque based on how much torque the driver is applying to the wheel
        self.lkas_max_torque = max(
          # Scale max torque down to half LKAX_MAX_TORQUE as a minimum
          CarControllerParams.LKAS_MAX_TORQUE * 0.5,
          # Start scaling torque at STEER_THRESHOLD
          CarControllerParams.LKAS_MAX_TORQUE - 0.6 * max(0, abs(CS.out.steeringTorque) - CarControllerParams.STEER_THRESHOLD)
        )

    else:
      apply_angle = CS.out.steeringAngleDeg
      self.lkas_max_torque = 0

    self.last_angle = apply_angle

    if self.CP.carFingerprint in (CAR.ROGUE, CAR.XTRAIL, CAR.ALTIMA) and pcm_cancel_cmd:
      can_sends.append(nissancan.create_acc_cancel_cmd(self.packer, self.car_fingerprint, CS.cruise_throttle_msg))

    # TODO: Find better way to cancel!
    # For some reason spamming the cancel button is unreliable on the Leaf
    # We now cancel by making propilot think the seatbelt is unlatched,
    # this generates a beep and a warning message every time you disengage
    if self.CP.carFingerprint in (CAR.LEAF, CAR.LEAF_IC) and self.frame % 2 == 0:
      can_sends.append(nissancan.create_cancel_msg(self.packer, CS.cancel_msg, pcm_cancel_cmd))

    can_sends.append(nissancan.create_steering_control(
      self.packer, apply_angle, self.frame, CC.enabled, self.lkas_max_torque))

    if lkas_hud_msg and lkas_hud_info_msg:
      if self.frame % 2 == 0:
        can_sends.append(nissancan.create_lkas_hud_msg(
          self.packer, lkas_hud_msg, CC.enabled, hud_control.leftLaneVisible, hud_control.rightLaneVisible, hud_control.leftLaneDepart, hud_control.rightLaneDepart))

      if self.frame % 50 == 0:
        can_sends.append(nissancan.create_lkas_hud_info_msg(
          self.packer, lkas_hud_info_msg, steer_hud_alert
        ))

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
