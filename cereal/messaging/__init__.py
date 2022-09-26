# must be build with scons
from .messaging_pyx import Context, Poller, SubSocket, PubSocket  # pylint: disable=no-name-in-module, import-error
from .messaging_pyx import MultiplePublishersError, MessagingError  # pylint: disable=no-name-in-module, import-error
import os
import capnp

from typing import Optional, List, Union
from collections import deque

from cereal import log
from cereal.services import service_list

assert MultiplePublishersError
assert MessagingError

NO_TRAVERSAL_LIMIT = 2**64-1
AVG_FREQ_HISTORY = 100
SIMULATION = "SIMULATION" in os.environ

# sec_since_boot is faster, but allow to run standalone too
try:
  from common.realtime import sec_since_boot
except ImportError:
  import time
  sec_since_boot = time.time
  print("Warning, using python time.time() instead of faster sec_since_boot")

context = Context()

def log_from_bytes(dat: bytes) -> capnp.lib.capnp._DynamicStructReader:
  return log.Event.from_bytes(dat, traversal_limit_in_words=NO_TRAVERSAL_LIMIT)

def new_message(service: Optional[str] = None, size: Optional[int] = None) -> capnp.lib.capnp._DynamicStructBuilder:
  dat = log.Event.new_message()
  dat.logMonoTime = int(sec_since_boot() * 1e9)
  dat.valid = True
  if service is not None:
    if size is None:
      dat.init(service)
    else:
      dat.init(service, size)
  return dat

def pub_sock(endpoint: str) -> PubSocket:
  sock = PubSocket()
  sock.connect(context, endpoint)
  return sock

def sub_sock(endpoint: str, poller: Optional[Poller] = None, addr: str = "127.0.0.1",
             conflate: bool = False, timeout: Optional[int] = None) -> SubSocket:
  sock = SubSocket()
  sock.connect(context, endpoint, addr.encode('utf8'), conflate)

  if timeout is not None:
    sock.setTimeout(timeout)

  if poller is not None:
    poller.registerSocket(sock)
  return sock


def drain_sock_raw(sock: SubSocket, wait_for_one: bool = False) -> List[bytes]:
  """Receive all message currently available on the queue"""
  ret: List[bytes] = []
  while 1:
    if wait_for_one and len(ret) == 0:
      dat = sock.receive()
    else:
      dat = sock.receive(non_blocking=True)

    if dat is None:
      break

    ret.append(dat)

  return ret

def drain_sock(sock: SubSocket, wait_for_one: bool = False) -> List[capnp.lib.capnp._DynamicStructReader]:
  """Receive all message currently available on the queue"""
  ret: List[capnp.lib.capnp._DynamicStructReader] = []
  while 1:
    if wait_for_one and len(ret) == 0:
      dat = sock.receive()
    else:
      dat = sock.receive(non_blocking=True)

    if dat is None:  # Timeout hit
      break

    dat = log_from_bytes(dat)
    ret.append(dat)

  return ret


# TODO: print when we drop packets?
def recv_sock(sock: SubSocket, wait: bool = False) -> Optional[capnp.lib.capnp._DynamicStructReader]:
  """Same as drain sock, but only returns latest message. Consider using conflate instead."""
  dat = None

  while 1:
    if wait and dat is None:
      rcv = sock.receive()
    else:
      rcv = sock.receive(non_blocking=True)

    if rcv is None:  # Timeout hit
      break

    dat = rcv

  if dat is not None:
    dat = log_from_bytes(dat)

  return dat

def recv_one(sock: SubSocket) -> Optional[capnp.lib.capnp._DynamicStructReader]:
  dat = sock.receive()
  if dat is not None:
    dat = log_from_bytes(dat)
  return dat

def recv_one_or_none(sock: SubSocket) -> Optional[capnp.lib.capnp._DynamicStructReader]:
  dat = sock.receive(non_blocking=True)
  if dat is not None:
    dat = log_from_bytes(dat)
  return dat

def recv_one_retry(sock: SubSocket) -> capnp.lib.capnp._DynamicStructReader:
  """Keep receiving until we get a message"""
  while True:
    dat = sock.receive()
    if dat is not None:
      return log_from_bytes(dat)

class SubMaster:
  def __init__(self, services: List[str], poll: Optional[List[str]] = None,
               ignore_alive: Optional[List[str]] = None, ignore_avg_freq: Optional[List[str]] = None,
               addr: str = "127.0.0.1"):
    self.frame = -1
    self.updated = {s: False for s in services}
    self.rcv_time = {s: 0. for s in services}
    self.rcv_frame = {s: 0 for s in services}
    self.alive = {s: False for s in services}
    self.freq_ok = {s: False for s in services}
    self.recv_dts = {s: deque([0.0] * AVG_FREQ_HISTORY, maxlen=AVG_FREQ_HISTORY) for s in services}
    self.sock = {}
    self.freq = {}
    self.data = {}
    self.valid = {}
    self.logMonoTime = {}

    self.poller = Poller()
    self.non_polled_services = [s for s in services if poll is not None and
                                len(poll) and s not in poll]

    self.ignore_average_freq = [] if ignore_avg_freq is None else ignore_avg_freq
    self.ignore_alive = [] if ignore_alive is None else ignore_alive

    for s in services:
      if addr is not None:
        p = self.poller if s not in self.non_polled_services else None
        self.sock[s] = sub_sock(s, poller=p, addr=addr, conflate=True)
      self.freq[s] = service_list[s].frequency

      try:
        data = new_message(s)
      except capnp.lib.capnp.KjException:  # pylint: disable=c-extension-no-member
        data = new_message(s, 0) # lists

      self.data[s] = getattr(data, s)
      self.logMonoTime[s] = 0
      self.valid[s] = data.valid

  def __getitem__(self, s: str) -> capnp.lib.capnp._DynamicStructReader:
    return self.data[s]

  def update(self, timeout: int = 1000) -> None:
    msgs = []
    for sock in self.poller.poll(timeout):
      msgs.append(recv_one_or_none(sock))

    # non-blocking receive for non-polled sockets
    for s in self.non_polled_services:
      msgs.append(recv_one_or_none(self.sock[s]))
    self.update_msgs(sec_since_boot(), msgs)

  def update_msgs(self, cur_time: float, msgs: List[capnp.lib.capnp._DynamicStructReader]) -> None:
    self.frame += 1
    self.updated = dict.fromkeys(self.updated, False)
    for msg in msgs:
      if msg is None:
        continue

      s = msg.which()
      self.updated[s] = True

      if self.rcv_time[s] > 1e-5 and self.freq[s] > 1e-5 and (s not in self.non_polled_services) \
        and (s not in self.ignore_average_freq):
        self.recv_dts[s].append(cur_time - self.rcv_time[s])

      self.rcv_time[s] = cur_time
      self.rcv_frame[s] = self.frame
      self.data[s] = getattr(msg, s)
      self.logMonoTime[s] = msg.logMonoTime
      self.valid[s] = msg.valid

      if SIMULATION:
        self.freq_ok[s] = True
        self.alive[s] = True

    if not SIMULATION:
      for s in self.data:
        # arbitrary small number to avoid float comparison. If freq is 0, we can skip the check
        if self.freq[s] > 1e-5:
          # alive if delay is within 10x the expected frequency
          self.alive[s] = (cur_time - self.rcv_time[s]) < (10. / self.freq[s])

          # TODO: check if update frequency is high enough to not drop messages
          # freq_ok if average frequency is higher than 90% of expected frequency
          avg_dt = sum(self.recv_dts[s]) / AVG_FREQ_HISTORY
          expected_dt = 1 / (self.freq[s] * 0.90)
          self.freq_ok[s] = (avg_dt < expected_dt)
        else:
          self.freq_ok[s] = True
          self.alive[s] = True

  def all_alive(self, service_list=None) -> bool:
    if service_list is None:  # check all
      service_list = self.alive.keys()
    return all(self.alive[s] for s in service_list if s not in self.ignore_alive)

  def all_freq_ok(self, service_list=None) -> bool:
    if service_list is None:  # check all
      service_list = self.alive.keys()
    return all(self.freq_ok[s] for s in service_list if s not in self.ignore_alive)

  def all_valid(self, service_list=None) -> bool:
    if service_list is None:  # check all
      service_list = self.valid.keys()
    return all(self.valid[s] for s in service_list)

  def all_checks(self, service_list=None) -> bool:
    if service_list is None:  # check all
      service_list = self.alive.keys()
    return self.all_alive(service_list=service_list) \
           and self.all_freq_ok(service_list=service_list) \
           and self.all_valid(service_list=service_list)

class PubMaster:
  def __init__(self, services: List[str]):
    self.sock = {}
    for s in services:
      self.sock[s] = pub_sock(s)

  def send(self, s: str, dat: Union[bytes, capnp.lib.capnp._DynamicStructBuilder]) -> None:
    if not isinstance(dat, bytes):
      dat = dat.to_bytes()
    self.sock[s].send(dat)

  def all_readers_updated(self, s: str) -> bool:
    return self.sock[s].all_readers_updated()  # type: ignore
