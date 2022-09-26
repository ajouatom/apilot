import copy
from collections import deque
from cereal import car
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from selfdrive.car.nissan.values import CAR, DBC, CarControllerParams

TORQUE_SAMPLES = 12

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.lkas_hud_msg = None
    self.lkas_hud_info_msg = None

    self.steeringTorqueSamples = deque(TORQUE_SAMPLES*[0], TORQUE_SAMPLES)
    self.shifter_values = can_define.dv["GEARBOX"]["GEAR_SHIFTER"]

  def update(self, cp, cp_adas, cp_cam):
    ret = car.CarState.new_message()

    if self.CP.carFingerprint in (CAR.ROGUE, CAR.XTRAIL, CAR.ALTIMA):
      ret.gas = cp.vl["GAS_PEDAL"]["GAS_PEDAL"]
    elif self.CP.carFingerprint in (CAR.LEAF, CAR.LEAF_IC):
      ret.gas = cp.vl["CRUISE_THROTTLE"]["GAS_PEDAL"]

    ret.gasPressed = bool(ret.gas > 3)

    if self.CP.carFingerprint in (CAR.ROGUE, CAR.XTRAIL, CAR.ALTIMA):
      ret.brakePressed = bool(cp.vl["DOORS_LIGHTS"]["USER_BRAKE_PRESSED"])
    elif self.CP.carFingerprint in (CAR.LEAF, CAR.LEAF_IC):
      ret.brakePressed = bool(cp.vl["CRUISE_THROTTLE"]["USER_BRAKE_PRESSED"])

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS_FRONT"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS_FRONT"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.

    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    if self.CP.carFingerprint == CAR.ALTIMA:
      ret.cruiseState.enabled = bool(cp.vl["CRUISE_STATE"]["CRUISE_ENABLED"])
    else:
      ret.cruiseState.enabled = bool(cp_adas.vl["CRUISE_STATE"]["CRUISE_ENABLED"])

    if self.CP.carFingerprint in (CAR.ROGUE, CAR.XTRAIL):
      ret.seatbeltUnlatched = cp.vl["HUD"]["SEATBELT_DRIVER_LATCHED"] == 0
      ret.cruiseState.available = bool(cp_cam.vl["PRO_PILOT"]["CRUISE_ON"])
    elif self.CP.carFingerprint in (CAR.LEAF, CAR.LEAF_IC):
      if self.CP.carFingerprint == CAR.LEAF:
        ret.seatbeltUnlatched = cp.vl["SEATBELT"]["SEATBELT_DRIVER_LATCHED"] == 0
      elif self.CP.carFingerprint == CAR.LEAF_IC:
        ret.seatbeltUnlatched = cp.vl["CANCEL_MSG"]["CANCEL_SEATBELT"] == 1
      ret.cruiseState.available = bool(cp.vl["CRUISE_THROTTLE"]["CRUISE_AVAILABLE"])
    elif self.CP.carFingerprint == CAR.ALTIMA:
      ret.seatbeltUnlatched = cp.vl["HUD"]["SEATBELT_DRIVER_LATCHED"] == 0
      ret.cruiseState.available = bool(cp_adas.vl["PRO_PILOT"]["CRUISE_ON"])

    if self.CP.carFingerprint == CAR.ALTIMA:
      speed = cp.vl["PROPILOT_HUD"]["SET_SPEED"]
    else:
      speed = cp_adas.vl["PROPILOT_HUD"]["SET_SPEED"]

    if speed != 255:
      if self.CP.carFingerprint in (CAR.LEAF, CAR.LEAF_IC):
        conversion = CV.MPH_TO_MS if cp.vl["HUD_SETTINGS"]["SPEED_MPH"] else CV.KPH_TO_MS
      else:
        conversion = CV.MPH_TO_MS if cp.vl["HUD"]["SPEED_MPH"] else CV.KPH_TO_MS
      ret.cruiseState.speed = speed * conversion
      ret.cruiseState.speedCluster = (speed - 1) * conversion  # Speed on HUD is always 1 lower than actually sent on can bus

    if self.CP.carFingerprint == CAR.ALTIMA:
      ret.steeringTorque = cp_cam.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    else:
      ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]

    self.steeringTorqueSamples.append(ret.steeringTorque)
    # Filtering driver torque to prevent steeringPressed false positives
    ret.steeringPressed = bool(abs(sum(self.steeringTorqueSamples) / TORQUE_SAMPLES) > CarControllerParams.STEER_THRESHOLD)

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]

    ret.leftBlinker = bool(cp.vl["LIGHTS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["LIGHTS"]["RIGHT_BLINKER"])

    ret.doorOpen = any([cp.vl["DOORS_LIGHTS"]["DOOR_OPEN_RR"],
                        cp.vl["DOORS_LIGHTS"]["DOOR_OPEN_RL"],
                        cp.vl["DOORS_LIGHTS"]["DOOR_OPEN_FR"],
                        cp.vl["DOORS_LIGHTS"]["DOOR_OPEN_FL"]])

    ret.espDisabled = bool(cp.vl["ESP"]["ESP_DISABLED"])

    can_gear = int(cp.vl["GEARBOX"]["GEAR_SHIFTER"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    if self.CP.carFingerprint == CAR.ALTIMA:
      self.lkas_enabled = bool(cp.vl["LKAS_SETTINGS"]["LKAS_ENABLED"])
    else:
      self.lkas_enabled = bool(cp_adas.vl["LKAS_SETTINGS"]["LKAS_ENABLED"])

    self.cruise_throttle_msg = copy.copy(cp.vl["CRUISE_THROTTLE"])

    if self.CP.carFingerprint in (CAR.LEAF, CAR.LEAF_IC):
      self.cancel_msg = copy.copy(cp.vl["CANCEL_MSG"])

    if self.CP.carFingerprint != CAR.ALTIMA:
      self.lkas_hud_msg = copy.copy(cp_adas.vl["PROPILOT_HUD"])
      self.lkas_hud_info_msg = copy.copy(cp_adas.vl["PROPILOT_HUD_INFO_MSG"])

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS_FRONT"),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS_FRONT"),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS_REAR"),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS_REAR"),

      ("STEER_ANGLE", "STEER_ANGLE_SENSOR"),

      ("DOOR_OPEN_FR", "DOORS_LIGHTS"),
      ("DOOR_OPEN_FL", "DOORS_LIGHTS"),
      ("DOOR_OPEN_RR", "DOORS_LIGHTS"),
      ("DOOR_OPEN_RL", "DOORS_LIGHTS"),

      ("RIGHT_BLINKER", "LIGHTS"),
      ("LEFT_BLINKER", "LIGHTS"),

      ("ESP_DISABLED", "ESP"),

      ("GEAR_SHIFTER", "GEARBOX"),
    ]

    checks = [
      # sig_address, frequency
      ("STEER_ANGLE_SENSOR", 100),
      ("WHEEL_SPEEDS_REAR", 50),
      ("WHEEL_SPEEDS_FRONT", 50),
      ("ESP", 25),
      ("GEARBOX", 25),
      ("DOORS_LIGHTS", 10),
      ("LIGHTS", 10),
    ]

    if CP.carFingerprint in (CAR.ROGUE, CAR.XTRAIL, CAR.ALTIMA):
      signals += [
        ("USER_BRAKE_PRESSED", "DOORS_LIGHTS"),

        ("GAS_PEDAL", "GAS_PEDAL"),
        ("SEATBELT_DRIVER_LATCHED", "HUD"),
        ("SPEED_MPH", "HUD"),

        ("PROPILOT_BUTTON", "CRUISE_THROTTLE"),
        ("CANCEL_BUTTON", "CRUISE_THROTTLE"),
        ("GAS_PEDAL_INVERTED", "CRUISE_THROTTLE"),
        ("SET_BUTTON", "CRUISE_THROTTLE"),
        ("RES_BUTTON", "CRUISE_THROTTLE"),
        ("FOLLOW_DISTANCE_BUTTON", "CRUISE_THROTTLE"),
        ("NO_BUTTON_PRESSED", "CRUISE_THROTTLE"),
        ("GAS_PEDAL", "CRUISE_THROTTLE"),
        ("USER_BRAKE_PRESSED", "CRUISE_THROTTLE"),
        ("NEW_SIGNAL_2", "CRUISE_THROTTLE"),
        ("GAS_PRESSED_INVERTED", "CRUISE_THROTTLE"),
        ("unsure1", "CRUISE_THROTTLE"),
        ("unsure2", "CRUISE_THROTTLE"),
        ("unsure3", "CRUISE_THROTTLE"),
      ]

      checks += [
        ("GAS_PEDAL", 100),
        ("CRUISE_THROTTLE", 50),
        ("HUD", 25),
      ]

    elif CP.carFingerprint in (CAR.LEAF, CAR.LEAF_IC):
      signals += [
        ("USER_BRAKE_PRESSED", "CRUISE_THROTTLE"),
        ("GAS_PEDAL", "CRUISE_THROTTLE"),
        ("CRUISE_AVAILABLE", "CRUISE_THROTTLE"),
        ("SPEED_MPH", "HUD_SETTINGS"),
        ("SEATBELT_DRIVER_LATCHED", "SEATBELT"),

        # Copy other values, we use this to cancel
        ("CANCEL_SEATBELT", "CANCEL_MSG"),
        ("NEW_SIGNAL_1", "CANCEL_MSG"),
        ("NEW_SIGNAL_2", "CANCEL_MSG"),
        ("NEW_SIGNAL_3", "CANCEL_MSG"),
      ]
      checks += [
        ("BRAKE_PEDAL", 100),
        ("CRUISE_THROTTLE", 50),
        ("CANCEL_MSG", 50),
        ("HUD_SETTINGS", 25),
        ("SEATBELT", 10),
      ]

    if CP.carFingerprint == CAR.ALTIMA:
      signals += [
        ("LKAS_ENABLED", "LKAS_SETTINGS"),
        ("CRUISE_ENABLED", "CRUISE_STATE"),
        ("SET_SPEED", "PROPILOT_HUD"),
      ]
      checks += [
        ("CRUISE_STATE", 10),
        ("LKAS_SETTINGS", 10),
        ("PROPILOT_HUD", 50),
      ]
      return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 1)

    signals.append(("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR"))
    checks.append(("STEER_TORQUE_SENSOR", 100))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_adas_can_parser(CP):
    # this function generates lists for signal, messages and initial values

    if CP.carFingerprint == CAR.ALTIMA:
      signals = [
        ("DESIRED_ANGLE", "LKAS"),
        ("SET_0x80_2", "LKAS"),
        ("MAX_TORQUE", "LKAS"),
        ("SET_0x80", "LKAS"),
        ("COUNTER", "LKAS"),
        ("LKA_ACTIVE", "LKAS"),

        ("CRUISE_ON", "PRO_PILOT"),
      ]
      checks = [
        ("LKAS", 100),
        ("PRO_PILOT", 100),
      ]
    else:
      signals = [
        # sig_name, sig_address
        ("LKAS_ENABLED", "LKAS_SETTINGS"),

        ("CRUISE_ENABLED", "CRUISE_STATE"),

        ("DESIRED_ANGLE", "LKAS"),
        ("SET_0x80_2", "LKAS"),
        ("MAX_TORQUE", "LKAS"),
        ("SET_0x80", "LKAS"),
        ("COUNTER", "LKAS"),
        ("LKA_ACTIVE", "LKAS"),

        # Below are the HUD messages. We copy the stock message and modify
        ("LARGE_WARNING_FLASHING", "PROPILOT_HUD"),
        ("SIDE_RADAR_ERROR_FLASHING1", "PROPILOT_HUD"),
        ("SIDE_RADAR_ERROR_FLASHING2", "PROPILOT_HUD"),
        ("LEAD_CAR", "PROPILOT_HUD"),
        ("LEAD_CAR_ERROR", "PROPILOT_HUD"),
        ("FRONT_RADAR_ERROR", "PROPILOT_HUD"),
        ("FRONT_RADAR_ERROR_FLASHING", "PROPILOT_HUD"),
        ("SIDE_RADAR_ERROR_FLASHING3", "PROPILOT_HUD"),
        ("LKAS_ERROR_FLASHING", "PROPILOT_HUD"),
        ("SAFETY_SHIELD_ACTIVE", "PROPILOT_HUD"),
        ("RIGHT_LANE_GREEN_FLASH", "PROPILOT_HUD"),
        ("LEFT_LANE_GREEN_FLASH", "PROPILOT_HUD"),
        ("FOLLOW_DISTANCE", "PROPILOT_HUD"),
        ("AUDIBLE_TONE", "PROPILOT_HUD"),
        ("SPEED_SET_ICON", "PROPILOT_HUD"),
        ("SMALL_STEERING_WHEEL_ICON", "PROPILOT_HUD"),
        ("unknown59", "PROPILOT_HUD"),
        ("unknown55", "PROPILOT_HUD"),
        ("unknown26", "PROPILOT_HUD"),
        ("unknown28", "PROPILOT_HUD"),
        ("unknown31", "PROPILOT_HUD"),
        ("SET_SPEED", "PROPILOT_HUD"),
        ("unknown43", "PROPILOT_HUD"),
        ("unknown08", "PROPILOT_HUD"),
        ("unknown05", "PROPILOT_HUD"),
        ("unknown02", "PROPILOT_HUD"),

        ("NA_HIGH_ACCEL_TEMP", "PROPILOT_HUD_INFO_MSG"),
        ("SIDE_RADAR_NA_HIGH_CABIN_TEMP", "PROPILOT_HUD_INFO_MSG"),
        ("SIDE_RADAR_MALFUNCTION", "PROPILOT_HUD_INFO_MSG"),
        ("LKAS_MALFUNCTION", "PROPILOT_HUD_INFO_MSG"),
        ("FRONT_RADAR_MALFUNCTION", "PROPILOT_HUD_INFO_MSG"),
        ("SIDE_RADAR_NA_CLEAN_REAR_CAMERA", "PROPILOT_HUD_INFO_MSG"),
        ("NA_POOR_ROAD_CONDITIONS", "PROPILOT_HUD_INFO_MSG"),
        ("CURRENTLY_UNAVAILABLE", "PROPILOT_HUD_INFO_MSG"),
        ("SAFETY_SHIELD_OFF", "PROPILOT_HUD_INFO_MSG"),
        ("FRONT_COLLISION_NA_FRONT_RADAR_OBSTRUCTION", "PROPILOT_HUD_INFO_MSG"),
        ("PEDAL_MISSAPPLICATION_SYSTEM_ACTIVATED", "PROPILOT_HUD_INFO_MSG"),
        ("SIDE_IMPACT_NA_RADAR_OBSTRUCTION", "PROPILOT_HUD_INFO_MSG"),
        ("WARNING_DO_NOT_ENTER", "PROPILOT_HUD_INFO_MSG"),
        ("SIDE_IMPACT_SYSTEM_OFF", "PROPILOT_HUD_INFO_MSG"),
        ("SIDE_IMPACT_MALFUNCTION", "PROPILOT_HUD_INFO_MSG"),
        ("FRONT_COLLISION_MALFUNCTION", "PROPILOT_HUD_INFO_MSG"),
        ("SIDE_RADAR_MALFUNCTION2", "PROPILOT_HUD_INFO_MSG"),
        ("LKAS_MALFUNCTION2", "PROPILOT_HUD_INFO_MSG"),
        ("FRONT_RADAR_MALFUNCTION2", "PROPILOT_HUD_INFO_MSG"),
        ("PROPILOT_NA_MSGS", "PROPILOT_HUD_INFO_MSG"),
        ("BOTTOM_MSG", "PROPILOT_HUD_INFO_MSG"),
        ("HANDS_ON_WHEEL_WARNING", "PROPILOT_HUD_INFO_MSG"),
        ("WARNING_STEP_ON_BRAKE_NOW", "PROPILOT_HUD_INFO_MSG"),
        ("PROPILOT_NA_FRONT_CAMERA_OBSTRUCTED", "PROPILOT_HUD_INFO_MSG"),
        ("PROPILOT_NA_HIGH_CABIN_TEMP", "PROPILOT_HUD_INFO_MSG"),
        ("WARNING_PROPILOT_MALFUNCTION", "PROPILOT_HUD_INFO_MSG"),
        ("ACC_UNAVAILABLE_HIGH_CABIN_TEMP", "PROPILOT_HUD_INFO_MSG"),
        ("ACC_NA_FRONT_CAMERA_IMPARED", "PROPILOT_HUD_INFO_MSG"),
        ("unknown07", "PROPILOT_HUD_INFO_MSG"),
        ("unknown10", "PROPILOT_HUD_INFO_MSG"),
        ("unknown15", "PROPILOT_HUD_INFO_MSG"),
        ("unknown23", "PROPILOT_HUD_INFO_MSG"),
        ("unknown19", "PROPILOT_HUD_INFO_MSG"),
        ("unknown31", "PROPILOT_HUD_INFO_MSG"),
        ("unknown32", "PROPILOT_HUD_INFO_MSG"),
        ("unknown46", "PROPILOT_HUD_INFO_MSG"),
        ("unknown61", "PROPILOT_HUD_INFO_MSG"),
        ("unknown55", "PROPILOT_HUD_INFO_MSG"),
        ("unknown50", "PROPILOT_HUD_INFO_MSG"),
      ]

      checks = [
        ("PROPILOT_HUD_INFO_MSG", 2),
        ("LKAS_SETTINGS", 10),
        ("CRUISE_STATE", 50),
        ("PROPILOT_HUD", 50),
        ("LKAS", 100),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = []
    checks = []

    if CP.carFingerprint in (CAR.ROGUE, CAR.XTRAIL):
      signals.append(("CRUISE_ON", "PRO_PILOT"))
      checks.append(("PRO_PILOT", 100))
    elif CP.carFingerprint == CAR.ALTIMA:
      signals.append(("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR"))
      checks.append(("STEER_TORQUE_SENSOR", 100))
      return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 1)
