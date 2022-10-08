#!/usr/bin/env python3
import datetime
import os
import re
import shutil
import signal
import subprocess
import time
import glob
from typing import NoReturn

from common.file_helpers import mkdirs_exists_ok
from selfdrive.loggerd.config import ROOT
import selfdrive.sentry as sentry
from system.swaglog import cloudlog
from system.version import get_commit

MAX_SIZE = 1_000_000 * 100  # allow up to 100M
MAX_TOMBSTONE_FN_LEN = 62  # 85 - 23 ("<dongle id>/crash/")

TOMBSTONE_DIR = "/data/tombstones/"
APPORT_DIR = "/var/crash/"


def safe_fn(s):
  extra = ['_']
  return "".join(c for c in s if c.isalnum() or c in extra).rstrip()


def clear_apport_folder():
  for f in glob.glob(APPORT_DIR + '*'):
    try:
      os.remove(f)
    except Exception:
      pass


def get_apport_stacktrace(fn):
  try:
    cmd = f'apport-retrace -s <(cat <(echo "Package: openpilot") "{fn}")'
    return subprocess.check_output(cmd, shell=True, encoding='utf8', timeout=30, executable='/bin/bash')  # pylint: disable=unexpected-keyword-arg
  except subprocess.CalledProcessError:
    return "Error getting stacktrace"
  except subprocess.TimeoutExpired:
    return "Timeout getting stacktrace"


def get_tombstones():
  """Returns list of (filename, ctime) for all tombstones in /data/tombstones
  and apport crashlogs in /var/crash"""
  files = []
  for folder in [TOMBSTONE_DIR, APPORT_DIR]:
    if os.path.exists(folder):
      with os.scandir(folder) as d:

        # Loop over first 1000 directory entries
        for _, f in zip(range(1000), d):
          if f.name.startswith("tombstone"):
            files.append((f.path, int(f.stat().st_ctime)))
          elif f.name.endswith(".crash") and f.stat().st_mode == 0o100640:
            files.append((f.path, int(f.stat().st_ctime)))
  return files


def report_tombstone_apport(fn):
  f_size = os.path.getsize(fn)
  if f_size > MAX_SIZE:
    cloudlog.error(f"Tombstone {fn} too big, {f_size}. Skipping...")
    return

  message = ""  # One line description of the crash
  contents = ""  # Full file contents without coredump
  path = ""  # File path relative to openpilot directory

  proc_maps = False

  with open(fn) as f:
    for line in f:
      if "CoreDump" in line:
        break
      elif "ProcMaps" in line:
        proc_maps = True
      elif "ProcStatus" in line:
        proc_maps = False

      if not proc_maps:
        contents += line

      if "ExecutablePath" in line:
        path = line.strip().split(': ')[-1]
        path = path.replace('/data/openpilot/', '')
        message += path
      elif "Signal" in line:
        message += " - " + line.strip()

        try:
          sig_num = int(line.strip().split(': ')[-1])
          message += " (" + signal.Signals(sig_num).name + ")"  # pylint: disable=no-member
        except ValueError:
          pass

  stacktrace = get_apport_stacktrace(fn)
  stacktrace_s = stacktrace.split('\n')
  crash_function = "No stacktrace"

  if len(stacktrace_s) > 2:
    found = False

    # Try to find first entry in openpilot, fall back to first line
    for line in stacktrace_s:
      if "at selfdrive/" in line:
        crash_function = line
        found = True
        break

    if not found:
      crash_function = stacktrace_s[1]

    # Remove arguments that can contain pointers to make sentry one-liner unique
    crash_function = " ".join(x for x in crash_function.split(' ')[1:] if not x.startswith('0x'))
    crash_function = re.sub(r'\(.*?\)', '', crash_function)

  contents = stacktrace + "\n\n" + contents
  message = message + " - " + crash_function
  sentry.report_tombstone(fn, message, contents)

  # Copy crashlog to upload folder
  clean_path = path.replace('/', '_')
  date = datetime.datetime.now().strftime("%Y-%m-%d--%H-%M-%S")

  new_fn = f"{date}_{get_commit(default='nocommit')[:8]}_{safe_fn(clean_path)}"[:MAX_TOMBSTONE_FN_LEN]

  crashlog_dir = os.path.join(ROOT, "crash")
  mkdirs_exists_ok(crashlog_dir)

  # Files could be on different filesystems, copy, then delete
  shutil.copy(fn, os.path.join(crashlog_dir, new_fn))

  try:
    os.remove(fn)
  except PermissionError:
    pass


def main() -> NoReturn:
  sentry.init(sentry.SentryProject.SELFDRIVE_NATIVE)

  # Clear apport folder on start, otherwise duplicate crashes won't register
  clear_apport_folder()
  initial_tombstones = set(get_tombstones())

  while True:
    now_tombstones = set(get_tombstones())

    for fn, _ in (now_tombstones - initial_tombstones):
      try:
        cloudlog.info(f"reporting new tombstone {fn}")
        if fn.endswith(".crash"):
          report_tombstone_apport(fn)
        else:
          cloudlog.error(f"unknown crash type: {fn}")
      except Exception:
        cloudlog.exception(f"Error reporting tombstone {fn}")

    initial_tombstones = now_tombstones
    time.sleep(5)


if __name__ == "__main__":
  main()
