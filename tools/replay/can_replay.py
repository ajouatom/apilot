#!/usr/bin/env python3
import os
import time
import threading
import multiprocessing
from tqdm import tqdm

os.environ['FILEREADER_CACHE'] = '1'

from common.basedir import BASEDIR
from common.realtime import config_realtime_process, Ratekeeper, DT_CTRL
from selfdrive.boardd.boardd import can_capnp_to_can_list
from tools.plotjuggler.juggle import load_segment
from panda import Panda

try:
  # this bool can be replaced when mypy understands this pattern
  panda_jungle_imported = True
  from panda_jungle import PandaJungle  # pylint: disable=import-error  # type: ignore
except ImportError:
  PandaJungle = None
  panda_jungle_imported = False


def send_thread(s, flock):
  if "Jungle" in str(type(s)):
    if "FLASH" in os.environ:
      with flock:
        s.flash()

    for i in [0, 1, 2, 3, 0xFFFF]:
      s.can_clear(i)
    s.set_ignition(False)
    time.sleep(5)
    s.set_ignition(True)
    s.set_panda_power(True)
  else:
    s.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  s.set_can_loopback(False)

  idx = 0
  ign = True
  rk = Ratekeeper(1 / DT_CTRL, print_delay_threshold=None)
  while True:
    # handle ignition cycling
    if ENABLE_IGN:
      i = (rk.frame*DT_CTRL) % (IGN_ON + IGN_OFF) < IGN_ON
      if i != ign:
        ign = i
        s.set_ignition(ign)

    snd = CAN_MSGS[idx]
    snd = list(filter(lambda x: x[-1] <= 2, snd))
    s.can_send_many(snd)
    idx = (idx + 1) % len(CAN_MSGS)

    # Drain panda message buffer
    s.can_recv()
    rk.keep_time()


def connect():
  config_realtime_process(3, 55)

  serials = {}
  flashing_lock = threading.Lock()
  while True:
    # look for new devices
    for p in [Panda, PandaJungle]:
      if p is None:
        continue

      for s in p.list():
        if s not in serials:
          print("starting send thread for", s)
          serials[s] = threading.Thread(target=send_thread, args=(p(s), flashing_lock))
          serials[s].start()

    # try to join all send threads
    cur_serials = serials.copy()
    for s, t in cur_serials.items():
      t.join(0.01)
      if not t.is_alive():
        del serials[s]

    time.sleep(1)


if __name__ == "__main__":
  if not panda_jungle_imported:
    print("\33[31m", "WARNING: cannot connect to jungles. Clone the jungle library to enable support:", "\033[0m")
    print("\033[34m", f"cd {BASEDIR} && git clone https://github.com/commaai/panda_jungle", "\033[0m")

  print("Loading log...")
  ROUTE = "77611a1fac303767/2020-03-24--09-50-38"
  REPLAY_SEGS = list(range(10, 16))  # route has 82 segments available
  CAN_MSGS = []
  logs = [f"https://commadataci.blob.core.windows.net/openpilotci/{ROUTE}/{i}/rlog.bz2" for i in REPLAY_SEGS]
  with multiprocessing.Pool(24) as pool:
    for lr in tqdm(pool.map(load_segment, logs)):
      CAN_MSGS += [can_capnp_to_can_list(m.can) for m in lr if m.which() == 'can']

  # set both to cycle ignition
  IGN_ON = int(os.getenv("ON", "0"))
  IGN_OFF = int(os.getenv("OFF", "0"))
  ENABLE_IGN = IGN_ON > 0 and IGN_OFF > 0
  if ENABLE_IGN:
    print(f"Cycling ignition: on for {IGN_ON}s, off for {IGN_OFF}s")

  connect()
