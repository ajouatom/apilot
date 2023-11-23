#!/usr/bin/env python3

import unittest

from parameterized import parameterized

from openpilot.selfdrive.test.process_replay.regen import regen_segment, DummyFrameReader
from openpilot.selfdrive.test.process_replay.process_replay import check_openpilot_enabled
from openpilot.selfdrive.test.openpilotci import get_url
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.framereader import FrameReader

TESTED_SEGMENTS = [
  ("PRIUS_C2", "0982d79ebb0de295|2021-01-04--17-13-21--13"), # TOYOTA PRIUS 2017:     NEO, pandaStateDEPRECATED, no peripheralState, sensorEventsDEPRECATED
  # Enable these once regen on CI becomes faster or use them for different tests running controlsd in isolation
  # ("MAZDA_C3", "bd6a637565e91581|2021-10-30--15-14-53--4"),  # MAZDA.CX9_2021:        TICI, incomplete managerState
  # ("FORD_C3", "54827bf84c38b14f|2023-01-26--21-59-07--4"),   # FORD.BRONCO_SPORT_MK1: TICI
]


def ci_setup_data_readers(route, sidx):
  lr = LogReader(get_url(route, sidx, "rlog"))
  frs = {
    'roadCameraState': FrameReader(get_url(route, sidx, "fcamera")),
    'driverCameraState': DummyFrameReader.zero_dcamera()
  }
  if next((True for m in lr if m.which() == "wideRoadCameraState"), False):
    frs["wideRoadCameraState"] = FrameReader(get_url(route, sidx, "ecamera"))

  return lr, frs


class TestRegen(unittest.TestCase):
  @parameterized.expand(TESTED_SEGMENTS)
  def test_engaged(self, case_name, segment):
    route, sidx = segment.rsplit("--", 1)
    lr, frs = ci_setup_data_readers(route, sidx)
    output_logs = regen_segment(lr, frs, disable_tqdm=True)

    engaged = check_openpilot_enabled(output_logs)
    self.assertTrue(engaged, f"openpilot not engaged in {case_name}")


if __name__=='__main__':
  unittest.main()
