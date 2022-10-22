#!/usr/bin/env python3
# pylint: disable=E1101
import os
import importlib
import unittest
from collections import defaultdict, Counter
from typing import List, Optional, Tuple
from parameterized import parameterized_class

from cereal import log, car
from common.realtime import DT_CTRL
from selfdrive.boardd.boardd import can_capnp_to_can_list, can_list_to_can_capnp
from selfdrive.car.fingerprints import all_known_cars
from selfdrive.car.car_helpers import interfaces
from selfdrive.car.gm.values import CAR as GM
from selfdrive.car.honda.values import CAR as HONDA, HONDA_BOSCH
from selfdrive.car.hyundai.values import CAR as HYUNDAI
from selfdrive.car.tests.routes import non_tested_cars, routes, CarTestRoute
from selfdrive.test.openpilotci import get_url
from tools.lib.logreader import LogReader
from tools.lib.route import Route

from panda.tests.safety import libpandasafety_py
from panda.tests.safety.common import package_can_msg

PandaType = log.PandaState.PandaType

NUM_JOBS = int(os.environ.get("NUM_JOBS", "1"))
JOB_ID = int(os.environ.get("JOB_ID", "0"))

ignore_addr_checks_valid = [
  GM.BUICK_REGAL,
  HYUNDAI.GENESIS_G70_2020,
]

# build list of test cases
routes_by_car = defaultdict(set)
for r in routes:
  routes_by_car[r.car_model].add(r)

test_cases: List[Tuple[str, Optional[CarTestRoute]]] = []
for i, c in enumerate(sorted(all_known_cars())):
  if i % NUM_JOBS == JOB_ID:
    test_cases.extend((c, r) for r in routes_by_car.get(c, (None, )))

SKIP_ENV_VAR = "SKIP_LONG_TESTS"


class TestCarModelBase(unittest.TestCase):
  car_model = None
  test_route = None
  ci = True

  @unittest.skipIf(SKIP_ENV_VAR in os.environ, f"Long running test skipped. Unset {SKIP_ENV_VAR} to run")
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == 'TestCarModel' or cls.__name__.endswith('Base'):
      raise unittest.SkipTest

    if 'FILTER' in os.environ:
      if not cls.car_model.startswith(tuple(os.environ.get('FILTER').split(','))):
        raise unittest.SkipTest

    if cls.test_route is None:
      if cls.car_model in non_tested_cars:
        print(f"Skipping tests for {cls.car_model}: missing route")
        raise unittest.SkipTest
      raise Exception(f"missing test route for {cls.car_model}")

    experimental_long = False
    test_segs = (2, 1, 0)
    if cls.test_route.segment is not None:
      test_segs = (cls.test_route.segment,)

    for seg in test_segs:
      try:
        if cls.ci:
          lr = LogReader(get_url(cls.test_route.route, seg))
        else:
          lr = LogReader(Route(cls.test_route.route).log_paths()[seg])
      except Exception:
        continue

      car_fw = []
      can_msgs = []
      fingerprint = defaultdict(dict)
      for msg in lr:
        if msg.which() == "can":
          for m in msg.can:
            if m.src < 64:
              fingerprint[m.src][m.address] = len(m.dat)
          can_msgs.append(msg)
        elif msg.which() == "carParams":
          car_fw = msg.carParams.carFw
          if msg.carParams.openpilotLongitudinalControl:
            experimental_long = True
          if cls.car_model is None and not cls.ci:
            cls.car_model = msg.carParams.carFingerprint

      if len(can_msgs) > int(50 / DT_CTRL):
        break
    else:
      raise Exception(f"Route: {repr(cls.test_route.route)} with segments: {test_segs} not found or no CAN msgs found. Is it uploaded?")

    cls.can_msgs = sorted(can_msgs, key=lambda msg: msg.logMonoTime)

    cls.CarInterface, cls.CarController, cls.CarState = interfaces[cls.car_model]
    cls.CP = cls.CarInterface.get_params(cls.car_model, fingerprint, car_fw, experimental_long)
    assert cls.CP
    assert cls.CP.carFingerprint == cls.car_model

  def setUp(self):
    self.CI = self.CarInterface(self.CP, self.CarController, self.CarState)
    assert self.CI

    # TODO: check safetyModel is in release panda build
    self.safety = libpandasafety_py.libpandasafety

    cfg = self.CP.safetyConfigs[-1]
    set_status = self.safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam)
    self.assertEqual(0, set_status, f"failed to set safetyModel {cfg}")
    self.safety.init_tests()

  def test_car_params(self):
    if self.CP.dashcamOnly:
      self.skipTest("no need to check carParams for dashcamOnly")

    # make sure car params are within a valid range
    self.assertGreater(self.CP.mass, 1)

    if self.CP.steerControlType != car.CarParams.SteerControlType.angle:
      tuning = self.CP.lateralTuning.which()
      if tuning == 'pid':
        self.assertTrue(len(self.CP.lateralTuning.pid.kpV))
      elif tuning == 'torque':
        self.assertTrue(self.CP.lateralTuning.torque.kf > 0)
      elif tuning == 'indi':
        self.assertTrue(len(self.CP.lateralTuning.indi.outerLoopGainV))
      else:
        raise Exception("unknown tuning")

  def test_car_interface(self):
    # TODO: also check for checksum violations from can parser
    can_invalid_cnt = 0
    can_valid = False
    CC = car.CarControl.new_message()

    for i, msg in enumerate(self.can_msgs):
      CS = self.CI.update(CC, (msg.as_builder().to_bytes(),))
      self.CI.apply(CC)

      if CS.canValid:
        can_valid = True

      # wait max of 2s for low frequency msgs to be seen
      if i > 200 or can_valid:
        can_invalid_cnt += not CS.canValid

    self.assertEqual(can_invalid_cnt, 0)

  def test_radar_interface(self):
    os.environ['NO_RADAR_SLEEP'] = "1"
    RadarInterface = importlib.import_module(f'selfdrive.car.{self.CP.carName}.radar_interface').RadarInterface
    RI = RadarInterface(self.CP)
    assert RI

    error_cnt = 0
    for i, msg in enumerate(self.can_msgs):
      rr = RI.update((msg.as_builder().to_bytes(),))
      if rr is not None and i > 50:
        error_cnt += car.RadarData.Error.canError in rr.errors
    self.assertEqual(error_cnt, 0)

  def test_panda_safety_rx_valid(self):
    if self.CP.dashcamOnly:
      self.skipTest("no need to check panda safety for dashcamOnly")

    start_ts = self.can_msgs[0].logMonoTime

    failed_addrs = Counter()
    for can in self.can_msgs:
      # update panda timer
      t = (can.logMonoTime - start_ts) / 1e3
      self.safety.set_timer(int(t))

      # run all msgs through the safety RX hook
      for msg in can.can:
        if msg.src >= 64:
          continue

        to_send = package_can_msg([msg.address, 0, msg.dat, msg.src % 4])
        if self.safety.safety_rx_hook(to_send) != 1:
          failed_addrs[hex(msg.address)] += 1

      # ensure all msgs defined in the addr checks are valid
      if self.car_model not in ignore_addr_checks_valid:
        self.safety.safety_tick_current_rx_checks()
        if t > 1e6:
          self.assertTrue(self.safety.addr_checks_valid())
    self.assertFalse(len(failed_addrs), f"panda safety RX check failed: {failed_addrs}")

  def test_panda_safety_carstate(self):
    """
      Assert that panda safety matches openpilot's carState
    """
    if self.CP.dashcamOnly:
      self.skipTest("no need to check panda safety for dashcamOnly")

    CC = car.CarControl.new_message()

    # warm up pass, as initial states may be different
    for can in self.can_msgs[:300]:
      for msg in can_capnp_to_can_list(can.can, src_filter=range(64)):
        to_send = package_can_msg(msg)
        self.safety.safety_rx_hook(to_send)
        self.CI.update(CC, (can_list_to_can_capnp([msg, ]), ))

    if not self.CP.pcmCruise:
      self.safety.set_controls_allowed(0)

    controls_allowed_prev = False
    CS_prev = car.CarState.new_message()
    checks = defaultdict(lambda: 0)
    for can in self.can_msgs:
      CS = self.CI.update(CC, (can.as_builder().to_bytes(), ))
      for msg in can_capnp_to_can_list(can.can, src_filter=range(64)):
        msg = list(msg)
        msg[3] %= 4
        to_send = package_can_msg(msg)
        ret = self.safety.safety_rx_hook(to_send)
        self.assertEqual(1, ret, f"safety rx failed ({ret=}): {to_send}")

      # TODO: check rest of panda's carstate (steering, ACC main on, etc.)

      checks['gasPressed'] += CS.gasPressed != self.safety.get_gas_pressed_prev()
      checks['cruiseState'] += CS.cruiseState.enabled and not CS.cruiseState.available
      if self.CP.carName not in ("hyundai", "volkswagen", "body"):
        # TODO: fix standstill mismatches for other makes
        checks['standstill'] += CS.standstill == self.safety.get_vehicle_moving()

      # TODO: remove this exception once this mismatch is resolved
      brake_pressed = CS.brakePressed
      if CS.brakePressed and not self.safety.get_brake_pressed_prev():
        if self.CP.carFingerprint in (HONDA.PILOT, HONDA.PASSPORT, HONDA.RIDGELINE) and CS.brake > 0.05:
          brake_pressed = False
      safety_brake_pressed = self.safety.get_brake_pressed_prev() or self.safety.get_regen_braking_prev()
      checks['brakePressed'] += brake_pressed != safety_brake_pressed

      if self.CP.pcmCruise:
        # On most pcmCruise cars, openpilot's state is always tied to the PCM's cruise state.
        # On Honda Nidec, we always engage on the rising edge of the PCM cruise state, but
        # openpilot brakes to zero even if the min ACC speed is non-zero (i.e. the PCM disengages).
        if self.CP.carName == "honda" and self.CP.carFingerprint not in HONDA_BOSCH:
          # only the rising edges are expected to match
          if CS.cruiseState.enabled and not CS_prev.cruiseState.enabled:
            checks['controlsAllowed'] += not self.safety.get_controls_allowed()
        else:
          checks['controlsAllowed'] += not CS.cruiseState.enabled and self.safety.get_controls_allowed()
      else:
        # Check for enable events on rising edge of controls allowed
        button_enable = any(evt.enable for evt in CS.events)
        mismatch = button_enable != (self.safety.get_controls_allowed() and not controls_allowed_prev)
        checks['controlsAllowed'] += mismatch
        controls_allowed_prev = self.safety.get_controls_allowed()
        if button_enable and not mismatch:
          self.safety.set_controls_allowed(False)

      if self.CP.carName == "honda":
        checks['mainOn'] += CS.cruiseState.available != self.safety.get_acc_main_on()

      CS_prev = CS

    failed_checks = {k: v for k, v in checks.items() if v > 0}
    self.assertFalse(len(failed_checks), f"panda safety doesn't agree with openpilot: {failed_checks}")


@parameterized_class(('car_model', 'test_route'), test_cases)
class TestCarModel(TestCarModelBase):
  pass


if __name__ == "__main__":
  unittest.main()
