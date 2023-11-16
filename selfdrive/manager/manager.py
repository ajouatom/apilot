#!/usr/bin/env python3
import datetime
import os
import signal
import subprocess
import sys
import traceback
from multiprocessing import Process
from typing import List, Tuple, Union

from cereal import log
import cereal.messaging as messaging
import openpilot.selfdrive.sentry as sentry
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.text_window import TextWindow
from openpilot.selfdrive.boardd.set_time import set_time
from openpilot.system.hardware import HARDWARE, PC
from openpilot.selfdrive.manager.helpers import unblock_stdout, write_onroad_params
from openpilot.selfdrive.manager.process import ensure_running, launcher
from openpilot.selfdrive.manager.process_config import managed_processes
from openpilot.selfdrive.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.system.swaglog import cloudlog, add_file_handler
from openpilot.system.version import is_dirty, get_commit, get_version, get_origin, get_short_branch, \
                           get_normalized_origin, terms_version, training_version, \
                           is_tested_branch, is_release_branch



def manager_init() -> None:
  # update system time from panda
  set_time(cloudlog)

  # save boot log
  subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "system/loggerd"))

  params = Params()
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)

  default_params: List[Tuple[str, Union[str, bytes]]] = [
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("GsmMetered", "1"),
    ("HasAcceptedTerms", "0"),
    ("LanguageSetting", "main_en"),
    ("OpenpilotEnabledToggle", "1"),
    ("LongitudinalPersonality", str(log.LongitudinalPersonality.standard)),
    ("ShowDebugUI", "0"),
    ("ShowDateTime", "1"),
    ("ShowHudMode", "4"),
    ("ShowSteerRotate", "1"),
    ("ShowPathEnd", "1"),
    ("ShowAccelRpm", "1"),
    ("ShowTpms", "1"),
    ("ShowSteerMode", "2"),
    ("ShowDeviceState", "1"),
    ("ShowConnInfo", "1"),
    ("ShowLaneInfo", "1"),
    ("ShowBlindSpot", "1"),
    ("ShowGapInfo", "1"),
    ("ShowDmInfo", "1"),
    ("ShowRadarInfo", "1"),
    ("MixRadarInfo", "0"),
    ("ShowZOffset", "122"),
    ("ShowPathMode", "9"),
    ("ShowPathColor", "12"),
    ("ShowPathModeCruiseOff", "0"),
    ("ShowPathColorCruiseOff", "19"),
    ("ShowPathModeLane", "12"),
    ("ShowPathColorLane", "13"),
    ("ShowPathWidth", "100"),
    ("ShowPlotMode", "0"),
    ("AutoResumeFromGas", "1"),
    ("AutoResumeFromGasSpeed", "30"),
    ("AutoResumeFromGasSpeedMode", "0"),    
    ("AutoCancelFromGasMode", "1"),    
    ("AutoCurveSpeedCtrlUse", "1"),
    ("AutoCurveSpeedFactor", "100"),
    ("AutoCurveSpeedFactorIn", "10"),
    ("AutoTurnControl", "0"),
    ("AutoTurnControlSpeedLaneChange", "60"),
    ("AutoTurnControlSpeedTurn", "20"),
    ("AutoTurnControlTurnEnd", "3"),
    ("AutoLaneChangeSpeed", "30"),
    ("AutoNaviSpeedCtrl", "1"),
    ("AutoNaviSpeedCtrlMode", "0"),
    ("AutoNaviSpeedCtrlStart", "22"),
    ("AutoNaviSpeedCtrlEnd", "6"),
    ("AutoNaviSpeedBumpTime", "1"),
    ("AutoNaviSpeedBumpSpeed", "35"),
    ("AutoNaviSpeedSafetyFactor", "105"),
    ("AutoNaviSpeedDecelRate", "80"),
    ("AutoRoadLimitCtrl", "0"),
    ("AutoResumeFromBrakeRelease", "1"),
    ("AutoResumeFromBrakeReleaseDist", "20"),
    ("AutoResumeFromBrakeReleaseLeadCar", "1"),
    ("AutoResumeFromBrakeCarSpeed", "40"),
    ("AutoResumeFromBrakeReleaseTrafficSign", "1"),
    ("LongControlActiveSound", "1"),
    ("StartAccelApply", "0"),
    ("StopAccelApply", "50"),
    ("TrafficStopDistanceAdjust", "180"),
    ("AutoSpeedUptoRoadSpeedLimit", "100"),
    ("ApplyLongDynamicCost", "0"), 
    ("AutoSpeedAdjustWithLeadCar", "0"),   
    ("TrafficStopAccel", "80"),     
    ("TrafficStopModelSpeed", "0"),         
    ("TrafficStopMode", "2"),         
    ("CruiseButtonMode", "0"),      
    ("CruiseSpeedUnit", "10"),      
    ("InitMyDrivingMode", "3"),      
    ("MySafeModeFactor", "80"),      
    ("LiveSteerRatioApply", "100"),      
    ("LiveTorqueCache", "0"),      
    ("MyEcoModeFactor", "80"),  
    ("CruiseMaxVals1", "160"),
    ("CruiseMaxVals2", "120"),
    ("CruiseMaxVals3", "100"),
    ("CruiseMaxVals4", "80"),
    ("CruiseMaxVals5", "70"),
    ("CruiseMaxVals6", "60"),
    ("PrevCruiseGap", "4"),      
    ("CruiseSpeedMin", "10"),
    ("AutoSyncCruiseSpeedMax", "120"),       
    ("StopDistance", "550"), 
    ("CustomMapbox", "0"),    
    ("LongitudinalTuningKpV", "100"),     
    ("LongitudinalTuningKiV", "200"),     
    ("LongitudinalTuningKf", "100"),     
    ("LongitudinalActuatorDelayUpperBound", "50"),     
    ("LongitudinalActuatorDelayLowerBound", "50"),     
    ("EnableRadarTracks", "0"),      
    ("HotspotOnBoot", "0"),      
    ("EnableAutoEngage", "0"),      
    ("SccConnectedBus2", "0"),   
    ("TFollowSpeedRatio", "110"),
    ("TFollowGap1", "110"),
    ("TFollowGap2", "120"),
    ("TFollowGap3", "140"),
    ("TFollowGap4", "160"),
    ("JerkStartLimit", "10"),    
    ("KeepEngage", "1"),
    ("HapticFeedbackWhenSpeedCamera", "0"),       
    ("MaxAngleFrames", "89"),       
    ("SoftHoldMode", "1"),       
    ("ApplyModelDistOrder", "30"),       
    ("TrafficStopAdjustRatio", "90"),       
    ("LateralTorqueCustom", "0"),       
    ("LateralTorqueAccelFactor", "2500"),       
    ("LateralTorqueFriction", "100"),       
    ("SteerActuatorDelay", "40"),       
    ("CruiseControlMode", "4"),
    ("CruiseOnDist", "0"),
    ("SteerRatioApply", "0"),
    ("SteerDeltaUp", "3"),       
    ("SteerDeltaDown", "7"),     
    ("AverageCurvature", "0"),
    ("Model", "1"),
  ]
  if not PC:
    default_params.append(("LastUpdateTime", datetime.datetime.utcnow().isoformat().encode('utf8')))

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # is this dashcam?
  if os.getenv("PASSIVE") is not None:
    params.put_bool("Passive", bool(int(os.getenv("PASSIVE", "0"))))

  if params.get("Passive") is None:
    raise Exception("Passive must be set to continue")

  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  # set version params
  params.put("Version", get_version())
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", get_commit(default=""))
  params.put("GitBranch", get_short_branch(default=""))
  params.put("GitRemote", get_origin(default=""))
  params.put_bool("IsTestedBranch", is_tested_branch())
  params.put_bool("IsReleaseBranch", is_release_branch())

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    serial = params.get("HardwareSerial")
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog

  if not is_dirty():
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=get_version(),
                       origin=get_normalized_origin(),
                       branch=get_short_branch(),
                       commit=get_commit(),
                       dirty=is_dirty(),
                       device=HARDWARE.get_device_type())
  if os.path.isfile('/data/tmux_error.log'):
    os.remove('/data/tmux_error.log')


def manager_prepare() -> None:
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:

  Process(name="road_speed_limiter", target=launcher, args=("openpilot.selfdrive.road_speed_limiter", "road_speed_limiter")).start()
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: List[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams'], poll=['deviceState'])
  pm = messaging.PubMaster(['managerState'])

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  print_timer = 0

  started_prev = False

  while True:
    sm.update()

    started = sm['deviceState'].started

    if started and not started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)

    # update onroad params, which drives boardd's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    running = ' '.join("%s%s\u001b[0m" % ("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print_timer = (print_timer + 1)%10
    if print_timer == 0:
      print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState')
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def main() -> None:
  prepare_only = os.getenv("PREPAREONLY") is not None

  manager_init()
  os.system("python /data/openpilot/selfdrive/car/hyundai/values.py > /data/params/d/SupportedCars")
  os.system("python /data/openpilot/selfdrive/car/gm/values.py > /data/params/d/SupportedCars_gm")
  subprocess.run(["python3", "/data/openpilot/selfdrive/modeld/model_switcher.py"])

  # Start UI early so prepare can happen in the background
  if not prepare_only:
    managed_processes['ui'].start()

  manager_prepare()

  if prepare_only:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
