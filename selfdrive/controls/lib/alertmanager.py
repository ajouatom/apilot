import copy
import os
import json
from collections import defaultdict
from dataclasses import dataclass
from typing import List, Dict, Optional

from common.basedir import BASEDIR
from common.params import Params
from selfdrive.controls.lib.events import Alert


with open(os.path.join(BASEDIR, "selfdrive/controls/lib/alerts_offroad.json")) as f:
  OFFROAD_ALERTS = json.load(f)


def set_offroad_alert(alert: str, show_alert: bool, extra_text: Optional[str] = None) -> None:
  if show_alert:
    a = OFFROAD_ALERTS[alert]
    if extra_text is not None:
      a = copy.copy(OFFROAD_ALERTS[alert])
      a['text'] += extra_text
    Params().put(alert, json.dumps(a))
  else:
    Params().remove(alert)


@dataclass
class AlertEntry:
  alert: Optional[Alert] = None
  start_frame: int = -1
  end_frame: int = -1

  def active(self, frame: int) -> bool:
    return frame <= self.end_frame

class AlertManager:
  def __init__(self):
    self.alerts: Dict[str, AlertEntry] = defaultdict(AlertEntry)

  def add_many(self, frame: int, alerts: List[Alert]) -> None:
    for alert in alerts:
      entry = self.alerts[alert.alert_type]
      entry.alert = alert
      if not entry.active(frame):
        entry.start_frame = frame
      min_end_frame = entry.start_frame + alert.duration
      entry.end_frame = max(frame + 1, min_end_frame)

  def process_alerts(self, frame: int, clear_event_types: set) -> Optional[Alert]:
    current_alert = AlertEntry()
    for v in self.alerts.values():
      if not v.alert:
        continue

      if v.alert.event_type in clear_event_types:
        v.end_frame = -1

      # sort by priority first and then by start_frame
      greater = current_alert.alert is None or (v.alert.priority, v.start_frame) > (current_alert.alert.priority, current_alert.start_frame)
      if v.active(frame) and greater:
        current_alert = v

    return current_alert.alert
