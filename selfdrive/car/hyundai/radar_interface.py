#!/usr/bin/env python3
import math

from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import RadarInterfaceBase
from selfdrive.car.hyundai.values import DBC
from common.params import Params
from common.numpy_fast import interp

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32


def get_radar_can_parser(CP):

  if CP.openpilotLongitudinalControl and (CP.sccBus == 0 or Params().get_bool("EnableRadarTracks")):

    signals = []
    checks = []

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = f"RADAR_TRACK_{addr:x}"
      signals += [
        ("STATE", msg),
        ("AZIMUTH", msg),
        ("LONG_DIST", msg),
        ("REL_ACCEL", msg),
        ("REL_SPEED", msg),
      ]
      checks += [(msg, 50)]
    print("RadarInterface: RadarTracks..")
    #return CANParser(DBC[CP.carFingerprint]['radar'], signals, checks, 1)
    return CANParser('hyundai_kia_mando_front_radar_generated', signals, checks, 1)

  else:
    signals = [
      # sig_name, sig_address, default
      ("ObjValid", "SCC11"),
      ("ACC_ObjStatus", "SCC11"),
      ("ACC_ObjLatPos", "SCC11"),
      ("ACC_ObjDist", "SCC11"),
      ("ACC_ObjRelSpd", "SCC11"),
    ]
    checks = [
      ("SCC11", 50),
    ]
    print("RadarInterface: SCCRadar...")
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, CP.sccBus)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.new_radar = CP.openpilotLongitudinalControl and (CP.sccBus == 0 or Params().get_bool("EnableRadarTracks"))
    self.updated_messages = set()
    self.trigger_msg = 0x420 if not self.new_radar else RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

  def update(self, can_strings):
    # This one causes my radar points to not work
    # if self.radar_off_can or (self.rcp is None):
    #   return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    if self.rcp is None:
      return ret

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    if self.new_radar:
      for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
        msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

        if addr not in self.pts:
          self.pts[addr] = car.RadarData.RadarPoint.new_message()
          self.pts[addr].trackId = self.track_id
          self.track_id += 1

        valid = msg['STATE'] in [3, 4]
        if valid:
          azimuth = math.radians(msg['AZIMUTH'])
          self.pts[addr].measured = True
          self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
          self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
          self.pts[addr].vRel = msg['REL_SPEED']
          self.pts[addr].aRel = msg['REL_ACCEL']
          self.pts[addr].yvRel = float('nan')

        else:
          del self.pts[addr]

      ret.points = list(self.pts.values())
      return ret

    else:
      cpt = self.rcp.vl

      valid = cpt["SCC11"]['ACC_ObjStatus']

      for ii in range(1):
        if valid:
          if ii not in self.pts:
            self.pts[ii] = car.RadarData.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1

          vRel = cpt["SCC11"]['ACC_ObjRelSpd']
          #self.pts[ii].dRel = cpt["SCC11"]['ACC_ObjDist']  # from front of car
          #self.pts[ii].dRel = (cpt["SCC11"]['ACC_ObjDist'] - 0.6) * 1.18 # from front of car  레이더트랙과 비슷하게... 조작

          #레이더트랙, 비젼거리와 비교해보니, vRel이 (-)일때 값이 같고, vRel이 +일때 거리가 적게 측정됨. vRel값으로 참고 하여 다가오는경우 멀어지는 경우 약간 보정이 필요할것 같음..
          # 하지만.. vRel값이 0 근처인경우... 값의 변화가 많아 선행차와 등속 주행시 약간의 문제가???
          self.pts[ii].dRel = interp(vRel, [-1.0, 0.0, 0.5], [cpt["SCC11"]['ACC_ObjDist'], cpt["SCC11"]['ACC_ObjDist'], (cpt["SCC11"]['ACC_ObjDist'] - 0.6) * 1.18])
          self.pts[ii].yRel = -cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
          self.pts[ii].vRel = cpt["SCC11"]['ACC_ObjRelSpd']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = True

        else:
          if ii in self.pts:
            del self.pts[ii]

      ret.points = list(self.pts.values())
      return ret
