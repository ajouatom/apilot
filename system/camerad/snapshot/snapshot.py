#!/usr/bin/env python3
import subprocess
import time

import numpy as np
from PIL import Image

import cereal.messaging as messaging
from cereal.visionipc import VisionIpcClient, VisionStreamType
from common.params import Params
from common.realtime import DT_MDL
from system.hardware import PC
from selfdrive.controls.lib.alertmanager import set_offroad_alert
from selfdrive.manager.process_config import managed_processes

LM_THRESH = 120  # defined in system/camerad/imgproc/utils.h

VISION_STREAMS = {
  "roadCameraState": VisionStreamType.VISION_STREAM_ROAD,
  "driverCameraState": VisionStreamType.VISION_STREAM_DRIVER,
  "wideRoadCameraState": VisionStreamType.VISION_STREAM_WIDE_ROAD,
}


def jpeg_write(fn, dat):
  img = Image.fromarray(dat)
  img.save(fn, "JPEG")


def yuv_to_rgb(y, u, v):
  ul = np.repeat(np.repeat(u, 2).reshape(u.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)
  vl = np.repeat(np.repeat(v, 2).reshape(v.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)

  yuv = np.dstack((y, ul, vl)).astype(np.int16)
  yuv[:, :, 1:] -= 128

  m = np.array([
    [1.00000,  1.00000, 1.00000],
    [0.00000, -0.39465, 2.03211],
    [1.13983, -0.58060, 0.00000],
  ])
  rgb = np.dot(yuv, m).clip(0, 255)
  return rgb.astype(np.uint8)


def extract_image(buf, w, h, stride, uv_offset):
  y = np.array(buf[:uv_offset], dtype=np.uint8).reshape((-1, stride))[:h, :w]
  u = np.array(buf[uv_offset::2], dtype=np.uint8).reshape((-1, stride//2))[:h//2, :w//2]
  v = np.array(buf[uv_offset+1::2], dtype=np.uint8).reshape((-1, stride//2))[:h//2, :w//2]

  return yuv_to_rgb(y, u, v)


def get_snapshots(frame="roadCameraState", front_frame="driverCameraState"):
  sockets = [s for s in (frame, front_frame) if s is not None]
  sm = messaging.SubMaster(sockets)
  vipc_clients = {s: VisionIpcClient("camerad", VISION_STREAMS[s], True) for s in sockets}

  # wait 4 sec from camerad startup for focus and exposure
  while sm[sockets[0]].frameId < int(4. / DT_MDL):
    sm.update()

  for client in vipc_clients.values():
    client.connect(True)

  # grab images
  rear, front = None, None
  if frame is not None:
    c = vipc_clients[frame]
    rear = extract_image(c.recv(), c.width, c.height, c.stride, c.uv_offset)
  if front_frame is not None:
    c = vipc_clients[front_frame]
    front = extract_image(c.recv(), c.width, c.height, c.stride, c.uv_offset)
  return rear, front


def snapshot():
  params = Params()

  if (not params.get_bool("IsOffroad")) or params.get_bool("IsTakingSnapshot"):
    print("Already taking snapshot")
    return None, None

  front_camera_allowed = params.get_bool("RecordFront")
  params.put_bool("IsTakingSnapshot", True)
  set_offroad_alert("Offroad_IsTakingSnapshot", True)
  time.sleep(2.0)  # Give thermald time to read the param, or if just started give camerad time to start

  # Check if camerad is already started
  try:
    subprocess.check_call(["pgrep", "camerad"])
    print("Camerad already running")
    params.put_bool("IsTakingSnapshot", False)
    params.remove("Offroad_IsTakingSnapshot")
    return None, None
  except subprocess.CalledProcessError:
    pass

  try:
    # Allow testing on replay on PC
    if not PC:
      managed_processes['camerad'].start()

    frame = "wideRoadCameraState"
    front_frame = "driverCameraState" if front_camera_allowed else None
    rear, front = get_snapshots(frame, front_frame)
  finally:
    managed_processes['camerad'].stop()
    params.put_bool("IsTakingSnapshot", False)
    set_offroad_alert("Offroad_IsTakingSnapshot", False)

  if not front_camera_allowed:
    front = None

  return rear, front


if __name__ == "__main__":
  pic, fpic = snapshot()
  if pic is not None:
    print(pic.shape)
    jpeg_write("/tmp/back.jpg", pic)
    if fpic is not None:
      jpeg_write("/tmp/front.jpg", fpic)
  else:
    print("Error taking snapshot")
