import math

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.selfdrive.car.hyundai.values import DBC
from openpilot.common.params import Params
from openpilot.common.filter_simple import StreamingMovingAverage

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP):

  enable_radar_tracks = Params().get_bool("EnableRadarTracks")
  scc_radar = Params().get_bool("SccConnectedBus2") and not enable_radar_tracks
  if scc_radar:
    print("RadarInterface: SCCRadar...")
    messages = [("SCC11", 50)]    
    return CANParser(DBC[CP.carFingerprint]['pt'], messages, 2)
 
  if DBC[CP.carFingerprint]['radar'] is None and not enable_radar_tracks:
    return None

  print("RadarInterface: RadarTracks...")
  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  #return CANParser(DBC[CP.carFingerprint]['radar'], messages, 1)
  return CANParser('hyundai_kia_mando_front_radar_generated', messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)
    
    self.enable_radar_tracks = Params().get_bool("EnableRadarTracks")
    self.scc_radar = Params().get_bool("SccConnectedBus2") and not self.enable_radar_tracks
    self.trigger_msg = 0x420 if self.scc_radar else self.trigger_msg

    self.dRelFilter = StreamingMovingAverage(2)
    self.vRelFilter = StreamingMovingAverage(2)
    self.valid_prev = False

  def update(self, can_strings):
    if not self.enable_radar_tracks and (self.radar_off_can or (self.rcp is None)):
      return super().update(None)

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
    
    if self.scc_radar:
      cpt = self.rcp.vl

      valid = cpt["SCC11"]['ACC_ObjStatus']

      for ii in range(1):
        if valid:
          dRel = cpt["SCC11"]['ACC_ObjDist']
          vRel = cpt["SCC11"]['ACC_ObjRelSpd']

          if ii not in self.pts:
            self.pts[ii] = car.RadarData.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
            dRel = self.dRelFilter.set(dRel)
            vRel = self.vRelFilter.set(dRel)
          else:
            dRel = self.dRelFilter.process(dRel)
            vRel = self.vRelFilter.process(dRel)
 
          self.pts[ii].dRel = dRel #cpt["SCC11"]['ACC_ObjDist']  # from front of car
          self.pts[ii].yRel = -cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
          self.pts[ii].vRel = vRel #cpt["SCC11"]['ACC_ObjRelSpd']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = True

        else:
          if ii in self.pts:
            del self.pts[ii]

      ret.points = list(self.pts.values())
      return ret

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = car.RadarData.RadarPoint.new_message()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
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
