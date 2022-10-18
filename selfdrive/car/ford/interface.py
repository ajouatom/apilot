#!/usr/bin/env python3
from cereal import car
from common.conversions import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.ford.values import CAR, Ecu, TransmissionType, GearShifter
from selfdrive.car.interfaces import CarInterfaceBase

CarParams = car.CarParams


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, experimental_long=False):
    if car_fw is None:
      car_fw = []

    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "ford"
    ret.dashcamOnly = True
    ret.safetyConfigs = [get_safety_config(CarParams.SafetyModel.ford)]

    # Angle-based steering
    ret.steerControlType = CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.4
    ret.steerLimitTimer = 1.0
    tire_stiffness_factor = 1.0

    if candidate == CAR.ESCAPE_MK4:
      ret.wheelbase = 2.71
      ret.steerRatio = 14.3  # Copied from Focus
      ret.mass = 1750 + STD_CARGO_KG

    elif candidate == CAR.EXPLORER_MK6:
      ret.wheelbase = 3.025
      ret.steerRatio = 16.8  # learned
      ret.mass = 2050 + STD_CARGO_KG

    elif candidate == CAR.FOCUS_MK4:
      ret.wheelbase = 2.7
      ret.steerRatio = 13.8  # learned
      ret.mass = 1350 + STD_CARGO_KG

    else:
      raise ValueError(f"Unsupported car: {candidate}")

    # Auto Transmission: 0x732 ECU or Gear_Shift_by_Wire_FD1
    found_ecus = [fw.ecu for fw in car_fw]
    if Ecu.shiftByWire in found_ecus or 0x5A in fingerprint[0]:
      ret.transmissionType = TransmissionType.automatic
    else:
      ret.transmissionType = TransmissionType.manual
      ret.minEnableSpeed = 20.0 * CV.MPH_TO_MS

    # BSM: Side_Detect_L_Stat, Side_Detect_R_Stat
    # TODO: detect bsm in car_fw?
    ret.enableBsm = 0x3A6 in fingerprint[0] and 0x3A7 in fingerprint[0]

    # LCA can steer down to zero
    ret.minSteerSpeed = 0.

    ret.autoResumeSng = ret.minEnableSpeed == -1.
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    events = self.create_common_events(ret, extra_gears=[GearShifter.manumatic])
    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    return self.CC.update(c, self.CS)
