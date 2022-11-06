#!/usr/bin/env python3
import os
import re
import datetime
import subprocess
import psutil
import shutil
import signal
import fcntl
import time
import threading
from collections import defaultdict
from pathlib import Path
from typing import List, Union, Optional
from markdown_it import MarkdownIt

from common.basedir import BASEDIR
from common.params import Params
from system.hardware import AGNOS, HARDWARE
from system.swaglog import cloudlog
from selfdrive.controls.lib.alertmanager import set_offroad_alert
from system.version import is_tested_branch

LOCK_FILE = os.getenv("UPDATER_LOCK_FILE", "/tmp/safe_staging_overlay.lock")
STAGING_ROOT = os.getenv("UPDATER_STAGING_ROOT", "/data/safe_staging")

OVERLAY_UPPER = os.path.join(STAGING_ROOT, "upper")
OVERLAY_METADATA = os.path.join(STAGING_ROOT, "metadata")
OVERLAY_MERGED = os.path.join(STAGING_ROOT, "merged")
FINALIZED = os.path.join(STAGING_ROOT, "finalized")

OVERLAY_INIT = Path(os.path.join(BASEDIR, ".overlay_init"))

DAYS_NO_CONNECTIVITY_MAX = 14     # do not allow to engage after this many days
DAYS_NO_CONNECTIVITY_PROMPT = 10  # send an offroad prompt after this many days

class WaitTimeHelper:
  def __init__(self):
    self.ready_event = threading.Event()
    self.only_check_for_update = False
    signal.signal(signal.SIGHUP, self.update_now)
    signal.signal(signal.SIGUSR1, self.check_now)

  def update_now(self, signum: int, frame) -> None:
    cloudlog.info("caught SIGHUP, attempting to downloading update")
    self.only_check_for_update = False
    self.ready_event.set()

  def check_now(self, signum: int, frame) -> None:
    cloudlog.info("caught SIGUSR1, checking for updates")
    self.only_check_for_update = True
    self.ready_event.set()

  def sleep(self, t: float) -> None:
    self.ready_event.wait(timeout=t)


def run(cmd: List[str], cwd: Optional[str] = None) -> str:
  return subprocess.check_output(cmd, cwd=cwd, stderr=subprocess.STDOUT, encoding='utf8')


def set_consistent_flag(consistent: bool) -> None:
  os.sync()
  consistent_file = Path(os.path.join(FINALIZED, ".overlay_consistent"))
  if consistent:
    consistent_file.touch()
  elif not consistent:
    consistent_file.unlink(missing_ok=True)
  os.sync()

def parse_release_notes(basedir: str) -> bytes:
  try:
    with open(os.path.join(basedir, "RELEASES.md"), "rb") as f:
      r = f.read().split(b'\n\n', 1)[0]  # Slice latest release notes
    try:
      return bytes(MarkdownIt().render(r.decode("utf-8")), encoding="utf-8")
    except Exception:
      return r + b"\n"
  except FileNotFoundError:
    pass
  except Exception:
    cloudlog.exception("failed to parse release notes")
  return b""

def setup_git_options(cwd: str) -> None:
  # We sync FS object atimes (which NEOS doesn't use) and mtimes, but ctimes
  # are outside user control. Make sure Git is set up to ignore system ctimes,
  # because they change when we make hard links during finalize. Otherwise,
  # there is a lot of unnecessary churn. This appears to be a common need on
  # OSX as well: https://www.git-tower.com/blog/make-git-rebase-safe-on-osx/

  # We are using copytree to copy the directory, which also changes
  # inode numbers. Ignore those changes too.

  # Set protocol to the new version (default after git 2.26) to reduce data
  # usage on git fetch --dry-run from about 400KB to 18KB.
  git_cfg = [
    ("core.trustctime", "false"),
    ("core.checkStat", "minimal"),
    ("protocol.version", "2"),
    ("gc.auto", "0"),
    ("gc.autoDetach", "false"),
  ]
  for option, value in git_cfg:
    run(["git", "config", option, value], cwd)


def dismount_overlay() -> None:
  if os.path.ismount(OVERLAY_MERGED):
    cloudlog.info("unmounting existing overlay")
    run(["sudo", "umount", "-l", OVERLAY_MERGED])


def init_overlay() -> None:

  # Re-create the overlay if BASEDIR/.git has changed since we created the overlay
  if OVERLAY_INIT.is_file() and os.path.ismount(OVERLAY_MERGED):
    git_dir_path = os.path.join(BASEDIR, ".git")
    new_files = run(["find", git_dir_path, "-newer", str(OVERLAY_INIT)])
    if not len(new_files.splitlines()):
      # A valid overlay already exists
      return
    else:
      cloudlog.info(".git directory changed, recreating overlay")

  cloudlog.info("preparing new safe staging area")

  params = Params()
  params.put_bool("UpdateAvailable", False)
  set_consistent_flag(False)
  dismount_overlay()
  run(["sudo", "rm", "-rf", STAGING_ROOT])
  if os.path.isdir(STAGING_ROOT):
    shutil.rmtree(STAGING_ROOT)

  for dirname in [STAGING_ROOT, OVERLAY_UPPER, OVERLAY_METADATA, OVERLAY_MERGED]:
    os.mkdir(dirname, 0o755)

  if os.lstat(BASEDIR).st_dev != os.lstat(OVERLAY_MERGED).st_dev:
    raise RuntimeError("base and overlay merge directories are on different filesystems; not valid for overlay FS!")

  # Leave a timestamped canary in BASEDIR to check at startup. The device clock
  # should be correct by the time we get here. If the init file disappears, or
  # critical mtimes in BASEDIR are newer than .overlay_init, continue.sh can
  # assume that BASEDIR has used for local development or otherwise modified,
  # and skips the update activation attempt.
  consistent_file = Path(os.path.join(BASEDIR, ".overlay_consistent"))
  if consistent_file.is_file():
    consistent_file.unlink()
  OVERLAY_INIT.touch()

  os.sync()
  overlay_opts = f"lowerdir={BASEDIR},upperdir={OVERLAY_UPPER},workdir={OVERLAY_METADATA}"

  mount_cmd = ["mount", "-t", "overlay", "-o", overlay_opts, "none", OVERLAY_MERGED]
  run(["sudo"] + mount_cmd)
  run(["sudo", "chmod", "755", os.path.join(OVERLAY_METADATA, "work")])

  git_diff = run(["git", "diff"], OVERLAY_MERGED)
  params.put("GitDiff", git_diff)
  cloudlog.info(f"git diff output:\n{git_diff}")


def finalize_update() -> None:
  """Take the current OverlayFS merged view and finalize a copy outside of
  OverlayFS, ready to be swapped-in at BASEDIR. Copy using shutil.copytree"""

  # Remove the update ready flag and any old updates
  cloudlog.info("creating finalized version of the overlay")
  set_consistent_flag(False)

  # Copy the merged overlay view and set the update ready flag
  if os.path.exists(FINALIZED):
    shutil.rmtree(FINALIZED)
  shutil.copytree(OVERLAY_MERGED, FINALIZED, symlinks=True)

  run(["git", "reset", "--hard"], FINALIZED)
  run(["git", "submodule", "foreach", "--recursive", "git", "reset", "--hard"], FINALIZED)

  cloudlog.info("Starting git cleanup in finalized update")
  t = time.monotonic()
  try:
    run(["git", "gc"], FINALIZED)
    run(["git", "lfs", "prune"], FINALIZED)
    cloudlog.event("Done git cleanup", duration=time.monotonic() - t)
  except subprocess.CalledProcessError:
    cloudlog.exception(f"Failed git cleanup, took {time.monotonic() - t:.3f} s")

  set_consistent_flag(True)
  cloudlog.info("done finalizing overlay")


def handle_agnos_update() -> None:
  from system.hardware.tici.agnos import flash_agnos_update, get_target_slot_number

  cur_version = HARDWARE.get_os_version()
  updated_version = run(["bash", "-c", r"unset AGNOS_VERSION && source launch_env.sh && \
                          echo -n $AGNOS_VERSION"], OVERLAY_MERGED).strip()

  cloudlog.info(f"AGNOS version check: {cur_version} vs {updated_version}")
  if cur_version == updated_version:
    return

  # prevent an openpilot getting swapped in with a mismatched or partially downloaded agnos
  set_consistent_flag(False)

  cloudlog.info(f"Beginning background installation for AGNOS {updated_version}")
  set_offroad_alert("Offroad_NeosUpdate", True)

  manifest_path = os.path.join(OVERLAY_MERGED, "system/hardware/tici/agnos.json")
  target_slot_number = get_target_slot_number()
  flash_agnos_update(manifest_path, target_slot_number, cloudlog)
  set_offroad_alert("Offroad_NeosUpdate", False)



class Updater:
  def __init__(self):
    self.params = Params()
    self.branches = defaultdict(lambda: '')
    self._has_internet: bool = False

  @property
  def has_internet(self) -> bool:
    return self._has_internet

  @property
  def target_branch(self) -> str:
    b: Union[str, None] = self.params.get("UpdaterTargetBranch", encoding='utf-8')
    if b is None:
      b = self.get_branch(BASEDIR)
      self.params.put("UpdaterTargetBranch", b)
    return b

  @property
  def update_ready(self) -> bool:
    consistent_file = Path(os.path.join(FINALIZED, ".overlay_consistent"))
    if consistent_file.is_file():
      hash_mismatch = self.get_commit_hash(BASEDIR) != self.branches[self.target_branch]
      branch_mismatch = self.get_branch(BASEDIR) != self.target_branch
      on_target_branch = self.get_branch(FINALIZED) == self.target_branch
      return ((hash_mismatch or branch_mismatch) and on_target_branch)
    return False

  @property
  def update_available(self) -> bool:
    if os.path.isdir(OVERLAY_MERGED):
      hash_mismatch = self.get_commit_hash(OVERLAY_MERGED) != self.branches[self.target_branch]
      branch_mismatch = self.get_branch(OVERLAY_MERGED) != self.target_branch
      return hash_mismatch or branch_mismatch
    return False

  def get_branch(self, path: str) -> str:
    return run(["git", "rev-parse", "--abbrev-ref", "HEAD"], path).rstrip()

  def get_commit_hash(self, path: str = OVERLAY_MERGED) -> str:
    return run(["git", "rev-parse", "HEAD"], path).rstrip()

  def set_params(self, failed_count: int, exception: Optional[str]) -> None:
    self.params.put("UpdateFailedCount", str(failed_count))

    self.params.put_bool("UpdaterFetchAvailable", self.update_available)
    self.params.put("UpdaterAvailableBranches", ','.join(self.branches.keys()))

    last_update = datetime.datetime.utcnow()
    if failed_count == 0:
      t = last_update.isoformat()
      self.params.put("LastUpdateTime", t.encode('utf8'))
    else:
      try:
        t = self.params.get("LastUpdateTime", encoding='utf8')
        last_update = datetime.datetime.fromisoformat(t)
      except (TypeError, ValueError):
        pass

    if exception is None:
      self.params.remove("LastUpdateException")
    else:
      self.params.put("LastUpdateException", exception)

    # Write out current and new version info
    def get_description(basedir: str) -> str:
      if not os.path.exists(basedir):
        return ""

      version = ""
      branch = ""
      commit = ""
      commit_date = ""
      try:
        branch = self.get_branch(basedir)
        commit = self.get_commit_hash(basedir)[:7]
        with open(os.path.join(basedir, "common", "version.h")) as f:
          version = f.read().split('"')[1]

        commit_unix_ts = run(["git", "show", "-s", "--format=%ct", "HEAD"], basedir).rstrip()
        dt = datetime.datetime.fromtimestamp(int(commit_unix_ts))
        commit_date = dt.strftime("%b %d")
      except Exception:
        cloudlog.exception("updater.get_description")
      return f"{version} / {branch} / {commit} / {commit_date}"
    self.params.put("UpdaterCurrentDescription", get_description(BASEDIR))
    self.params.put("UpdaterCurrentReleaseNotes", parse_release_notes(BASEDIR))
    self.params.put("UpdaterNewDescription", get_description(FINALIZED))
    self.params.put("UpdaterNewReleaseNotes", parse_release_notes(FINALIZED))
    self.params.put_bool("UpdateAvailable", self.update_ready)

    # Handle user prompt
    for alert in ("Offroad_UpdateFailed", "Offroad_ConnectivityNeeded", "Offroad_ConnectivityNeededPrompt"):
      set_offroad_alert(alert, False)

    now = datetime.datetime.utcnow()
    dt = now - last_update
    if failed_count > 15 and exception is not None and self.has_internet:
      if is_tested_branch():
        extra_text = "Ensure the software is correctly installed. Uninstall and re-install if this error persists."
      else:
        extra_text = exception
      set_offroad_alert("Offroad_UpdateFailed", True, extra_text=extra_text)
    elif dt.days > DAYS_NO_CONNECTIVITY_MAX and failed_count > 1:
      set_offroad_alert("Offroad_ConnectivityNeeded", True)
    elif dt.days > DAYS_NO_CONNECTIVITY_PROMPT:
      remaining = max(DAYS_NO_CONNECTIVITY_MAX - dt.days, 1)
      set_offroad_alert("Offroad_ConnectivityNeededPrompt", True, extra_text=f"{remaining} day{'' if remaining == 1 else 's'}.")

  def check_for_update(self) -> None:
    cloudlog.info("checking for updates")

    excluded_branches = ('release2', 'release2-staging', 'dashcam', 'dashcam-staging')

    try:
      run(["git", "ls-remote", "origin", "HEAD"], OVERLAY_MERGED)
      self._has_internet = True
    except subprocess.CalledProcessError:
      self._has_internet = False

    setup_git_options(OVERLAY_MERGED)
    output = run(["git", "ls-remote", "--heads"], OVERLAY_MERGED)

    self.branches = defaultdict(lambda: None)
    for line in output.split('\n'):
      ls_remotes_re = r'(?P<commit_sha>\b[0-9a-f]{5,40}\b)(\s+)(refs\/heads\/)(?P<branch_name>.*$)'
      x = re.fullmatch(ls_remotes_re, line.strip())
      if x is not None and x.group('branch_name') not in excluded_branches:
        self.branches[x.group('branch_name')] = x.group('commit_sha')

    cur_branch = self.get_branch(OVERLAY_MERGED)
    cur_commit = self.get_commit_hash(OVERLAY_MERGED)
    new_branch = self.target_branch
    new_commit = self.branches[new_branch]
    if (cur_branch, cur_commit) != (new_branch, new_commit):
      cloudlog.info(f"update available, {cur_branch} ({str(cur_commit)[:7]}) -> {new_branch} ({str(new_commit)[:7]})")
    else:
      cloudlog.info(f"up to date on {cur_branch} ({str(cur_commit)[:7]})")

  def fetch_update(self) -> None:
    cloudlog.info("attempting git fetch inside staging overlay")

    self.params.put("UpdaterState", "downloading...")

    # TODO: cleanly interrupt this and invalidate old update
    set_consistent_flag(False)
    self.params.put_bool("UpdateAvailable", False)

    setup_git_options(OVERLAY_MERGED)

    branch = self.target_branch
    git_fetch_output = run(["git", "fetch", "origin", branch], OVERLAY_MERGED)
    cloudlog.info("git fetch success: %s", git_fetch_output)

    cloudlog.info("git reset in progress")
    cmds = [
      ["git", "checkout", "--force", "--no-recurse-submodules", "-B", branch, "FETCH_HEAD"],
      ["git", "reset", "--hard"],
      ["git", "clean", "-xdff"],
      ["git", "submodule", "sync"],
      ["git", "submodule", "update", "--init", "--recursive"],
      ["git", "submodule", "foreach", "--recursive", "git", "reset", "--hard"],
    ]
    r = [run(cmd, OVERLAY_MERGED) for cmd in cmds]
    cloudlog.info("git reset success: %s", '\n'.join(r))

    # TODO: show agnos download progress
    if AGNOS:
      handle_agnos_update()

    # Create the finalized, ready-to-swap update
    self.params.put("UpdaterState", "finalizing update...")
    finalize_update()
    cloudlog.info("finalize success!")


def main() -> None:
  params = Params()

  if params.get_bool("DisableUpdates"):
    cloudlog.warning("updates are disabled by the DisableUpdates param")
    exit(0)

  ov_lock_fd = open(LOCK_FILE, 'w')
  try:
    fcntl.flock(ov_lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except OSError as e:
    raise RuntimeError("couldn't get overlay lock; is another instance running?") from e

  # Set low io priority
  proc = psutil.Process()
  if psutil.LINUX:
    proc.ionice(psutil.IOPRIO_CLASS_BE, value=7)

  # Check if we just performed an update
  if Path(os.path.join(STAGING_ROOT, "old_openpilot")).is_dir():
    cloudlog.event("update installed")

  if not params.get("InstallDate"):
    t = datetime.datetime.utcnow().isoformat()
    params.put("InstallDate", t.encode('utf8'))

  updater = Updater()
  update_failed_count = 0  # TODO: Load from param?

  # no fetch on the first time
  wait_helper = WaitTimeHelper()
  wait_helper.only_check_for_update = True

  # Run the update loop
  while True:
    wait_helper.ready_event.clear()

    # Attempt an update
    exception = None
    try:
      # TODO: reuse overlay from previous updated instance if it looks clean
      init_overlay()

      # ensure we have some params written soon after startup
      updater.set_params(update_failed_count, exception)
      update_failed_count += 1

      # check for update
      params.put("UpdaterState", "checking...")
      updater.check_for_update()

      # download update
      if wait_helper.only_check_for_update:
        cloudlog.info("skipping fetch this cycle")
      else:
        updater.fetch_update()
      update_failed_count = 0
    except subprocess.CalledProcessError as e:
      cloudlog.event(
        "update process failed",
        cmd=e.cmd,
        output=e.output,
        returncode=e.returncode
      )
      exception = f"command failed: {e.cmd}\n{e.output}"
      OVERLAY_INIT.unlink(missing_ok=True)
    except Exception as e:
      cloudlog.exception("uncaught updated exception, shouldn't happen")
      exception = str(e)
      OVERLAY_INIT.unlink(missing_ok=True)

    try:
      params.put("UpdaterState", "idle")
      updater.set_params(update_failed_count, exception)
    except Exception:
      cloudlog.exception("uncaught updated exception while setting params, shouldn't happen")

    # infrequent attempts if we successfully updated recently
    wait_helper.only_check_for_update = False
    wait_helper.sleep(5*60 if update_failed_count > 0 else 1.5*60*60)


if __name__ == "__main__":
  main()
