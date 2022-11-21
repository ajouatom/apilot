#!/usr/bin/env python3
import argparse
import concurrent.futures
import os
import random
import traceback
from tqdm import tqdm

from selfdrive.test.process_replay.helpers import OpenpilotPrefix
from selfdrive.test.process_replay.regen import regen_and_save
from selfdrive.test.process_replay.test_processes import FAKEDATA, source_segments as segments
from tools.lib.route import SegmentName


def regen_job(segment, upload, disable_tqdm):
  with OpenpilotPrefix():
    sn = SegmentName(segment[1])
    fake_dongle_id = 'regen' + ''.join(random.choice('0123456789ABCDEF') for _ in range(11))
    try:
      relr = regen_and_save(sn.route_name.canonical_name, sn.segment_num, upload=upload, use_route_meta=False,
                            outdir=os.path.join(FAKEDATA, fake_dongle_id), disable_tqdm=disable_tqdm)
      relr = '|'.join(relr.split('/')[-2:])
      return f'  ("{segment[0]}", "{relr}"), '
    except Exception as e:
      err = f"  {segment} failed: {str(e)}"
      err += traceback.format_exc()
      err += "\n\n"
      return err


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Generate new segments from old ones")
  parser.add_argument("-j", "--jobs", type=int, default=1)
  parser.add_argument("--no-upload", action="store_true")
  args = parser.parse_args()

  with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as pool:
    p = pool.map(regen_job, segments, [not args.no_upload] * len(segments), [args.jobs > 1] * len(segments))
    msg = "Copy these new segments into test_processes.py:"
    for seg in tqdm(p, desc="Generating segments", total=len(segments)):
      msg += "\n" + str(seg)
    print()
    print()
    print(msg)
