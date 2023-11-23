#!/usr/bin/env python3
import bz2
import math
import json
import os
import pathlib
import psutil
import pytest
import shutil
import subprocess
import time
import numpy as np
import unittest
from collections import Counter, defaultdict
from functools import cached_property
from pathlib import Path

from cereal import car
import cereal.messaging as messaging
from cereal.services import SERVICE_LIST
from openpilot.common.basedir import BASEDIR
from openpilot.common.timeout import Timeout
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.events import EVENTS, ET
from openpilot.system.hardware import HARDWARE
from openpilot.selfdrive.test.helpers import set_params_enabled, release_only
from openpilot.system.hardware.hw import Paths
from openpilot.tools.lib.logreader import LogReader

# Baseline CPU usage by process
PROCS = {
  "selfdrive.controls.controlsd": 39.0,
  "./loggerd": 14.0,
  "./encoderd": 17.0,
  "./camerad": 14.5,
  "./locationd": 11.0,
  "./mapsd": 1.5,
  "selfdrive.controls.plannerd": 16.5,
  "./_ui": 18.0,
  "selfdrive.locationd.paramsd": 9.0,
  "./sensord": 7.0,
  "selfdrive.controls.radard": 4.5,
  "selfdrive.modeld.modeld": 13.0,
  "selfdrive.modeld.dmonitoringmodeld": 8.0,
  "selfdrive.modeld.navmodeld": 1.0,
  "selfdrive.thermald.thermald": 3.87,
  "selfdrive.locationd.calibrationd": 2.0,
  "selfdrive.locationd.torqued": 5.0,
  "./_soundd": (1.0, 65.0),
  "selfdrive.monitoring.dmonitoringd": 4.0,
  "./proclogd": 1.54,
  "system.logmessaged": 0.2,
  "selfdrive.tombstoned": 0,
  "./logcatd": 0,
  "system.micd": 10.0,
  "system.timezoned": 0,
  "selfdrive.boardd.pandad": 0,
  "selfdrive.statsd": 0.4,
  "selfdrive.navd.navd": 0.4,
  "system.loggerd.uploader": 3.0,
  "system.loggerd.deleter": 0.1,
}

PROCS.update({
  "tici": {
    "./boardd": 4.0,
    "./ubloxd": 0.02,
    "system.sensord.pigeond": 6.0,
  },
  "tizi": {
     "./boardd": 19.0,
    "system.qcomgpsd.qcomgpsd": 1.0,
  }
}.get(HARDWARE.get_device_type(), {}))

TIMINGS = {
  # rtols: max/min, rsd
  "can": [2.5, 0.35],
  "pandaStates": [2.5, 0.35],
  "peripheralState": [2.5, 0.35],
  "sendcan": [2.5, 0.35],
  "carState": [2.5, 0.35],
  "carControl": [2.5, 0.35],
  "controlsState": [2.5, 0.35],
  "lateralPlan": [2.5, 0.5],
  "longitudinalPlan": [2.5, 0.5],
  "roadCameraState": [2.5, 0.35],
  "driverCameraState": [2.5, 0.35],
  "modelV2": [2.5, 0.35],
  "driverStateV2": [2.5, 0.40],
  "navModel": [2.5, 0.35],
  "mapRenderState": [2.5, 0.35],
  "liveLocationKalman": [2.5, 0.35],
  "wideRoadCameraState": [1.5, 0.35],
}


def cputime_total(ct):
  return ct.cpuUser + ct.cpuSystem + ct.cpuChildrenUser + ct.cpuChildrenSystem


@pytest.mark.tici
class TestOnroad(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    if "DEBUG" in os.environ:
      segs = filter(lambda x: os.path.exists(os.path.join(x, "rlog")), Path(Paths.log_root()).iterdir())
      segs = sorted(segs, key=lambda x: x.stat().st_mtime)
      print(segs[-3])
      cls.lr = list(LogReader(os.path.join(segs[-3], "rlog")))
      return

    # setup env
    params = Params()
    if "CI" in os.environ:
      params.clear_all()
    params.remove("CurrentRoute")
    set_params_enabled()
    os.environ['TESTING_CLOSET'] = '1'
    if os.path.exists(Paths.log_root()):
      shutil.rmtree(Paths.log_root())
    os.system("rm /dev/shm/*")

    # Make sure athena isn't running
    os.system("pkill -9 -f athena")

    # start manager and run openpilot for a minute
    proc = None
    try:
      manager_path = os.path.join(BASEDIR, "selfdrive/manager/manager.py")
      proc = subprocess.Popen(["python", manager_path])

      sm = messaging.SubMaster(['carState'])
      with Timeout(150, "controls didn't start"):
        while sm.rcv_frame['carState'] < 0:
          sm.update(1000)

      # make sure we get at least two full segments
      route = None
      cls.segments = []
      with Timeout(300, "timed out waiting for logs"):
        while route is None:
          route = params.get("CurrentRoute", encoding="utf-8")
          time.sleep(0.1)

        while len(cls.segments) < 3:
          segs = set()
          if Path(Paths.log_root()).exists():
            segs = set(Path(Paths.log_root()).glob(f"{route}--*"))
          cls.segments = sorted(segs, key=lambda s: int(str(s).rsplit('--')[-1]))
          time.sleep(2)

      # chop off last, incomplete segment
      cls.segments = cls.segments[:-1]

    finally:
      cls.gpu_procs = {psutil.Process(int(f.name)).name() for f in pathlib.Path('/sys/devices/virtual/kgsl/kgsl/proc/').iterdir() if f.is_dir()}

      if proc is not None:
        proc.terminate()
        if proc.wait(60) is None:
          proc.kill()

    cls.lrs = [list(LogReader(os.path.join(str(s), "rlog"))) for s in cls.segments]

    # use the second segment by default as it's the first full segment
    cls.lr = list(LogReader(os.path.join(str(cls.segments[1]), "rlog")))
    cls.log_path = cls.segments[1]

  @cached_property
  def service_msgs(self):
    msgs = defaultdict(list)
    for m in self.lr:
      msgs[m.which()].append(m)
    return msgs

  def test_service_frequencies(self):
    for s, msgs in self.service_msgs.items():
      if s in ('initData', 'sentinel'):
        continue

      # skip gps services for now
      if s in ('ubloxGnss', 'ubloxRaw', 'gnssMeasurements', 'gpsLocation', 'gpsLocationExternal', 'qcomGnss'):
        continue

      with self.subTest(service=s):
        assert len(msgs) >= math.floor(SERVICE_LIST[s].frequency*55)

  def test_cloudlog_size(self):
    msgs = [m for m in self.lr if m.which() == 'logMessage']

    total_size = sum(len(m.as_builder().to_bytes()) for m in msgs)
    self.assertLess(total_size, 3.5e5)

    cnt = Counter(json.loads(m.logMessage)['filename'] for m in msgs)
    big_logs = [f for f, n in cnt.most_common(3) if n / sum(cnt.values()) > 30.]
    self.assertEqual(len(big_logs), 0, f"Log spam: {big_logs}")

  def test_log_sizes(self):
    for f in self.log_path.iterdir():
      assert f.is_file()

      sz = f.stat().st_size / 1e6
      if f.name in ("qlog", "rlog"):
        with open(f, 'rb') as ff:
          sz = len(bz2.compress(ff.read())) / 1e6

      if f.name == "qcamera.ts":
        assert 2.15 < sz < 2.35
      elif f.name == "qlog":
        assert 0.7 < sz < 1.0
      elif f.name == "rlog":
        assert 5 < sz < 50
      elif f.name.endswith('.hevc'):
        assert 70 < sz < 77
      else:
        raise NotImplementedError

  def test_ui_timings(self):
    result = "\n"
    result += "------------------------------------------------\n"
    result += "-------------- UI Draw Timing ------------------\n"
    result += "------------------------------------------------\n"

    ts = [m.uiDebug.drawTimeMillis for m in self.service_msgs['uiDebug']]
    result += f"min  {min(ts):.2f}ms\n"
    result += f"max  {max(ts):.2f}ms\n"
    result += f"std  {np.std(ts):.2f}ms\n"
    result += f"mean {np.mean(ts):.2f}ms\n"
    result += "------------------------------------------------\n"
    print(result)

    self.assertLess(max(ts), 250.)
    self.assertLess(np.mean(ts), 10.)
    #self.assertLess(np.std(ts), 5.)

    # some slow frames are expected since camerad/modeld can preempt ui
    veryslow = [x for x in ts if x > 40.]
    assert len(veryslow) < 5, f"Too many slow frame draw times: {veryslow}"

  def test_cpu_usage(self):
    result = "\n"
    result += "------------------------------------------------\n"
    result += "------------------ CPU Usage -------------------\n"
    result += "------------------------------------------------\n"

    plogs_by_proc = defaultdict(list)
    for pl in self.service_msgs['procLog']:
      for x in pl.procLog.procs:
        if len(x.cmdline) > 0:
          n = list(x.cmdline)[0]
          plogs_by_proc[n].append(x)
    print(plogs_by_proc.keys())

    cpu_ok = True
    dt = (self.service_msgs['procLog'][-1].logMonoTime - self.service_msgs['procLog'][0].logMonoTime) / 1e9
    for proc_name, expected_cpu in PROCS.items():

      err = ""
      cpu_usage = 0.
      x = plogs_by_proc[proc_name]
      if len(x) > 2:
        cpu_time = cputime_total(x[-1]) - cputime_total(x[0])
        cpu_usage = cpu_time / dt * 100.

        if isinstance(expected_cpu, tuple):
          exp = str(expected_cpu)
          minn, maxx = expected_cpu
        else:
          exp = f"{expected_cpu:5.2f}"
          minn = min(expected_cpu * 0.65, max(expected_cpu - 1.0, 0.0))
          maxx = max(expected_cpu * 1.15, expected_cpu + 5.0)

        if cpu_usage > maxx:
          err = "using more CPU than expected"
        elif cpu_usage < minn:
          err = "using less CPU than expected"
      else:
        err = "NO METRICS FOUND"

      result += f"{proc_name.ljust(35)}  {cpu_usage:5.2f}% ({exp}%) {err}\n"
      if len(err) > 0:
        cpu_ok = False

    # Ensure there's no missing procs
    all_procs = {p.name for p in self.service_msgs['managerState'][0].managerState.processes if p.shouldBeRunning}
    for p in all_procs:
      with self.subTest(proc=p):
        assert any(p in pp for pp in PROCS.keys()), f"Expected CPU usage missing for {p}"

    result += "------------------------------------------------\n"
    print(result)

    self.assertTrue(cpu_ok)

  def test_memory_usage(self):
    mems = [m.deviceState.memoryUsagePercent for m in self.service_msgs['deviceState']]
    print("Memory usage: ", mems)

    # check for big leaks. note that memory usage is
    # expected to go up while the MSGQ buffers fill up
    self.assertLessEqual(max(mems) - min(mems), 3.0)

  def test_gpu_usage(self):
    self.assertEqual(self.gpu_procs, {"weston", "_ui", "camerad", "selfdrive.modeld.modeld"})

  def test_camera_processing_time(self):
    result = "\n"
    result += "------------------------------------------------\n"
    result += "-------------- Debayer Timing ------------------\n"
    result += "------------------------------------------------\n"

    ts = [getattr(m, m.which()).processingTime for m in self.lr if 'CameraState' in m.which()]
    self.assertLess(min(ts), 0.025, f"high execution time: {min(ts)}")
    result += f"execution time: min  {min(ts):.5f}s\n"
    result += f"execution time: max  {max(ts):.5f}s\n"
    result += f"execution time: mean {np.mean(ts):.5f}s\n"
    result += "------------------------------------------------\n"
    print(result)

  @unittest.skip("TODO: enable once timings are fixed")
  def test_camera_frame_timings(self):
    result = "\n"
    result += "------------------------------------------------\n"
    result += "-----------------  SoF Timing ------------------\n"
    result += "------------------------------------------------\n"
    for name in ['roadCameraState', 'wideRoadCameraState', 'driverCameraState']:
      ts = [getattr(m, m.which()).timestampSof for m in self.lr if name in m.which()]
      d_ms = np.diff(ts) / 1e6
      d50 = np.abs(d_ms-50)
      self.assertLess(max(d50), 1.0, f"high sof delta vs 50ms: {max(d50)}")
      result += f"{name} sof delta vs 50ms: min  {min(d50):.5f}s\n"
      result += f"{name} sof delta vs 50ms: max  {max(d50):.5f}s\n"
      result += f"{name} sof delta vs 50ms: mean {d50.mean():.5f}s\n"
      result += "------------------------------------------------\n"
    print(result)

  def test_mpc_execution_timings(self):
    result = "\n"
    result += "------------------------------------------------\n"
    result += "-----------------  MPC Timing ------------------\n"
    result += "------------------------------------------------\n"

    cfgs = [("lateralPlan", 0.05, 0.05), ("longitudinalPlan", 0.05, 0.05)]
    for (s, instant_max, avg_max) in cfgs:
      ts = [getattr(m, s).solverExecutionTime for m in self.service_msgs[s]]
      self.assertLess(max(ts), instant_max, f"high '{s}' execution time: {max(ts)}")
      self.assertLess(np.mean(ts), avg_max, f"high avg '{s}' execution time: {np.mean(ts)}")
      result += f"'{s}' execution time: min  {min(ts):.5f}s\n"
      result += f"'{s}' execution time: max  {max(ts):.5f}s\n"
      result += f"'{s}' execution time: mean {np.mean(ts):.5f}s\n"
    result += "------------------------------------------------\n"
    print(result)

  def test_model_execution_timings(self):
    result = "\n"
    result += "------------------------------------------------\n"
    result += "----------------- Model Timing -----------------\n"
    result += "------------------------------------------------\n"
    # TODO: this went up when plannerd cpu usage increased, why?
    cfgs = [
      ("modelV2", 0.050, 0.036),
      ("driverStateV2", 0.050, 0.026),
    ]
    for (s, instant_max, avg_max) in cfgs:
      ts = [getattr(m, s).modelExecutionTime for m in self.service_msgs[s]]
      self.assertLess(max(ts), instant_max, f"high '{s}' execution time: {max(ts)}")
      self.assertLess(np.mean(ts), avg_max, f"high avg '{s}' execution time: {np.mean(ts)}")
      result += f"'{s}' execution time: min  {min(ts):.5f}s\n"
      result += f"'{s}' execution time: max {max(ts):.5f}s\n"
      result += f"'{s}' execution time: mean {np.mean(ts):.5f}s\n"
    result += "------------------------------------------------\n"
    print(result)

  def test_timings(self):
    passed = True
    result = "\n"
    result += "------------------------------------------------\n"
    result += "----------------- Service Timings --------------\n"
    result += "------------------------------------------------\n"
    for s, (maxmin, rsd) in TIMINGS.items():
      msgs = [m.logMonoTime for m in self.service_msgs[s]]
      if not len(msgs):
        raise Exception(f"missing {s}")

      ts = np.diff(msgs) / 1e9
      dt = 1 / SERVICE_LIST[s].frequency

      try:
        np.testing.assert_allclose(np.mean(ts), dt, rtol=0.03, err_msg=f"{s} - failed mean timing check")
        np.testing.assert_allclose([np.max(ts), np.min(ts)], dt, rtol=maxmin, err_msg=f"{s} - failed max/min timing check")
      except Exception as e:
        result += str(e) + "\n"
        passed = False

      if np.std(ts) / dt > rsd:
        result += f"{s} - failed RSD timing check\n"
        passed = False

      result += f"{s.ljust(40)}: {np.array([np.mean(ts), np.max(ts), np.min(ts)])*1e3}\n"
      result += f"{''.ljust(40)}  {np.max(np.absolute([np.max(ts)/dt, np.min(ts)/dt]))} {np.std(ts)/dt}\n"
    result += "="*67
    print(result)
    self.assertTrue(passed)

  @release_only
  def test_startup(self):
    startup_alert = None
    for msg in self.lrs[0]:
      # can't use carEvents because the first msg can be dropped while loggerd is starting up
      if msg.which() == "controlsState":
        startup_alert = msg.controlsState.alertText1
        break
    expected = EVENTS[car.CarEvent.EventName.startup][ET.PERMANENT].alert_text_1
    self.assertEqual(startup_alert, expected, "wrong startup alert")

  def test_engagable(self):
    no_entries = Counter()
    for m in self.service_msgs['carEvents']:
      for evt in m.carEvents:
        if evt.noEntry:
          no_entries[evt.name] += 1

    eng = [m.controlsState.engageable for m in self.service_msgs['controlsState']]
    assert all(eng), \
           f"Not engageable for whole segment:\n- controlsState.engageable: {Counter(eng)}\n- No entry events: {no_entries}"


if __name__ == "__main__":
  unittest.main()
