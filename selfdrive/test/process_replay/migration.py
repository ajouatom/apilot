from collections import defaultdict

from cereal import messaging
from openpilot.selfdrive.test.process_replay.vision_meta import meta_from_encode_index
from openpilot.selfdrive.car.toyota.values import EPS_SCALE
from openpilot.selfdrive.manager.process_config import managed_processes
from panda import Panda


def migrate_all(lr, old_logtime=False, manager_states=False, panda_states=False, camera_states=False):
  msgs = migrate_sensorEvents(lr, old_logtime)
  msgs = migrate_carParams(msgs, old_logtime)
  if manager_states:
    msgs = migrate_managerState(msgs)
  if panda_states:
    msgs = migrate_pandaStates(msgs)
    msgs = migrate_peripheralState(msgs)
  if camera_states:
    msgs = migrate_cameraStates(msgs)

  return msgs


def migrate_managerState(lr):
  all_msgs = []
  for msg in lr:
    if msg.which() != "managerState":
      all_msgs.append(msg)
      continue

    new_msg = msg.as_builder()
    new_msg.managerState.processes = [{'name': name, 'running': True} for name in managed_processes]
    all_msgs.append(new_msg.as_reader())

  return all_msgs


def migrate_pandaStates(lr):
  all_msgs = []
  # TODO: safety param migration should be handled automatically
  safety_param_migration = {
    "TOYOTA PRIUS 2017": EPS_SCALE["TOYOTA PRIUS 2017"] | Panda.FLAG_TOYOTA_STOCK_LONGITUDINAL,
    "TOYOTA RAV4 2017": EPS_SCALE["TOYOTA RAV4 2017"] | Panda.FLAG_TOYOTA_ALT_BRAKE,
    "KIA EV6 2022": Panda.FLAG_HYUNDAI_EV_GAS | Panda.FLAG_HYUNDAI_CANFD_HDA2,
  }

  # Migrate safety param base on carState
  CP = next((m.carParams for m in lr if m.which() == 'carParams'), None)
  assert CP is not None, "carParams message not found"
  if CP.carFingerprint in safety_param_migration:
    safety_param = safety_param_migration[CP.carFingerprint]
  elif len(CP.safetyConfigs):
    safety_param = CP.safetyConfigs[0].safetyParam
    if CP.safetyConfigs[0].safetyParamDEPRECATED != 0:
      safety_param = CP.safetyConfigs[0].safetyParamDEPRECATED
  else:
    safety_param = CP.safetyParamDEPRECATED

  for msg in lr:
    if msg.which() == 'pandaStateDEPRECATED':
      new_msg = messaging.new_message('pandaStates', 1)
      new_msg.valid = msg.valid
      new_msg.logMonoTime = msg.logMonoTime
      new_msg.pandaStates[0] = msg.pandaStateDEPRECATED
      new_msg.pandaStates[0].safetyParam = safety_param
      all_msgs.append(new_msg.as_reader())
    elif msg.which() == 'pandaStates':
      new_msg = msg.as_builder()
      new_msg.pandaStates[-1].safetyParam = safety_param
      all_msgs.append(new_msg.as_reader())
    else:
      all_msgs.append(msg)

  return all_msgs


def migrate_peripheralState(lr):
  if any(msg.which() == "peripheralState" for msg in lr):
    return lr

  all_msg = []
  for msg in lr:
    all_msg.append(msg)
    if msg.which() not in ["pandaStates", "pandaStateDEPRECATED"]:
      continue

    new_msg = messaging.new_message("peripheralState")
    new_msg.logMonoTime = msg.logMonoTime
    all_msg.append(new_msg.as_reader())

  return all_msg


def migrate_cameraStates(lr):
  all_msgs = []
  frame_to_encode_id = defaultdict(dict)
  # just for encodeId fallback mechanism
  min_frame_id = defaultdict(lambda: float('inf'))

  for msg in lr:
    if msg.which() not in ["roadEncodeIdx", "wideRoadEncodeIdx", "driverEncodeIdx"]:
      continue

    encode_index = getattr(msg, msg.which())
    meta = meta_from_encode_index(msg.which())

    assert encode_index.segmentId < 1200, f"Encoder index segmentId greater that 1200: {msg.which()} {encode_index.segmentId}"
    frame_to_encode_id[meta.camera_state][encode_index.frameId] = encode_index.segmentId

  for msg in lr:
    if msg.which() not in ["roadCameraState", "wideRoadCameraState", "driverCameraState"]:
      all_msgs.append(msg)
      continue

    camera_state = getattr(msg, msg.which())
    min_frame_id[msg.which()] = min(min_frame_id[msg.which()], camera_state.frameId)

    encode_id = frame_to_encode_id[msg.which()].get(camera_state.frameId)
    if encode_id is None:
      print(f"Missing encoded frame for camera feed {msg.which()} with frameId: {camera_state.frameId}")
      if len(frame_to_encode_id[msg.which()]) != 0:
        continue

      # fallback mechanism for logs without encodeIdx (e.g. logs from before 2022 with dcamera recording disabled)
      # try to fake encode_id by subtracting lowest frameId
      encode_id = camera_state.frameId - min_frame_id[msg.which()]
      print(f"Faking encodeId to {encode_id} for camera feed {msg.which()} with frameId: {camera_state.frameId}")

    new_msg = messaging.new_message(msg.which())
    new_camera_state = getattr(new_msg, new_msg.which())
    new_camera_state.frameId = encode_id
    new_camera_state.encodeId = encode_id
    # timestampSof was added later so it might be missing on some old segments
    if camera_state.timestampSof == 0 and camera_state.timestampEof > 25000000:
      new_camera_state.timestampSof = camera_state.timestampEof - 18000000
    else:
      new_camera_state.timestampSof = camera_state.timestampSof
    new_camera_state.timestampEof = camera_state.timestampEof
    new_msg.logMonoTime = msg.logMonoTime
    new_msg.valid = msg.valid

    all_msgs.append(new_msg.as_reader())

  return all_msgs


def migrate_carParams(lr, old_logtime=False):
  all_msgs = []
  for msg in lr:
    if msg.which() == 'carParams':
      CP = messaging.new_message('carParams')
      CP.carParams = msg.carParams.as_builder()
      for car_fw in CP.carParams.carFw:
        car_fw.brand = CP.carParams.carName
      if old_logtime:
        CP.logMonoTime = msg.logMonoTime
      msg = CP.as_reader()
    all_msgs.append(msg)

  return all_msgs


def migrate_sensorEvents(lr, old_logtime=False):
  all_msgs = []
  for msg in lr:
    if msg.which() != 'sensorEventsDEPRECATED':
      all_msgs.append(msg)
      continue

    # migrate to split sensor events
    for evt in msg.sensorEventsDEPRECATED:
      # build new message for each sensor type
      sensor_service = ''
      if evt.which() == 'acceleration':
        sensor_service = 'accelerometer'
      elif evt.which() == 'gyro' or evt.which() == 'gyroUncalibrated':
        sensor_service = 'gyroscope'
      elif evt.which() == 'light' or evt.which() == 'proximity':
        sensor_service = 'lightSensor'
      elif evt.which() == 'magnetic' or evt.which() == 'magneticUncalibrated':
        sensor_service = 'magnetometer'
      elif evt.which() == 'temperature':
        sensor_service = 'temperatureSensor'

      m = messaging.new_message(sensor_service)
      m.valid = True
      if old_logtime:
        m.logMonoTime = msg.logMonoTime

      m_dat = getattr(m, sensor_service)
      m_dat.version = evt.version
      m_dat.sensor = evt.sensor
      m_dat.type = evt.type
      m_dat.source = evt.source
      if old_logtime:
        m_dat.timestamp = evt.timestamp
      setattr(m_dat, evt.which(), getattr(evt, evt.which()))

      all_msgs.append(m.as_reader())

  return all_msgs
