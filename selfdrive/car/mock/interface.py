#!/usr/bin/env python3
import math
from cereal import car
from common.conversions import Conversions as CV
from system.swaglog import cloudlog
import cereal.messaging as messaging
from selfdrive.car import gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

# mocked car interface to work with chffrplus
TS = 0.01  # 100Hz
YAW_FR = 0.2  # ~0.8s time constant on yaw rate filter
# low pass gain
LPG = 2 * math.pi * YAW_FR * TS / (1 + 2 * math.pi * YAW_FR * TS)


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    cloudlog.debug("Using Mock Car Interface")

    self.sm = messaging.SubMaster(['gyroscope', 'gpsLocation', 'gpsLocationExternal'])

    self.speed = 0.
    self.prev_speed = 0.
    self.yaw_rate = 0.
    self.yaw_rate_meas = 0.

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, experimental_long=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "mock"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.noOutput)]
    ret.mass = 1700.
    ret.rotationalInertia = 2500.
    ret.wheelbase = 2.70
    ret.centerToFront = ret.wheelbase * 0.5
    ret.steerRatio = 13.  # reasonable
    ret.tireStiffnessFront = 1e6    # very stiff to neglect slip
    ret.tireStiffnessRear = 1e6     # very stiff to neglect slip

    return ret

  # returns a car.CarState
  def _update(self, c):
    self.sm.update(0)

    # get basic data from phone and gps since CAN isn't connected
    if self.sm.updated['gyroscope']:
      self.yaw_rate_meas = -self.sm['gyroscope'].gyroUncalibrated.v[0]

    gps_sock = 'gpsLocationExternal' if self.sm.rcv_frame['gpsLocationExternal'] > 1 else 'gpsLocation'
    if self.sm.updated[gps_sock]:
      self.prev_speed = self.speed
      self.speed = self.sm[gps_sock].speed

    # create message
    ret = car.CarState.new_message()

    # speeds
    ret.vEgo = self.speed
    ret.vEgoRaw = self.speed
    a = self.speed - self.prev_speed

    ret.aEgo = a
    ret.brakePressed = a < -0.5

    ret.standstill = self.speed < 0.01
    ret.wheelSpeeds.fl = self.speed
    ret.wheelSpeeds.fr = self.speed
    ret.wheelSpeeds.rl = self.speed
    ret.wheelSpeeds.rr = self.speed

    self.yawRate = LPG * self.yaw_rate_meas + (1. - LPG) * self.yaw_rate
    curvature = self.yaw_rate / max(self.speed, 1.)
    ret.steeringAngleDeg = curvature * self.CP.steerRatio * self.CP.wheelbase * CV.RAD_TO_DEG

    return ret

  def apply(self, c):
    # in mock no carcontrols
    actuators = car.CarControl.Actuators.new_message()
    return actuators, []
