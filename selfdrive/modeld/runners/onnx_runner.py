#!/usr/bin/env python3

import os
import sys
import numpy as np

os.environ["OMP_NUM_THREADS"] = "4"
os.environ["OMP_WAIT_POLICY"] = "PASSIVE"

import onnxruntime as ort # pylint: disable=import-error

def read(sz, tf8=False):
  dd = []
  gt = 0
  szof = 1 if tf8 else 4
  while gt < sz * szof:
    st = os.read(0, sz * szof - gt)
    assert(len(st) > 0)
    dd.append(st)
    gt += len(st)
  r = np.frombuffer(b''.join(dd), dtype=np.uint8 if tf8 else np.float32).astype(np.float32)
  if tf8:
    r = r / 255.
  return r

def write(d):
  os.write(1, d.tobytes())

def run_loop(m, tf8_input=False):
  ishapes = [[1]+ii.shape[1:] for ii in m.get_inputs()]
  keys = [x.name for x in m.get_inputs()]

  # run once to initialize CUDA provider
  if "CUDAExecutionProvider" in m.get_providers():
    m.run(None, dict(zip(keys, [np.zeros(shp, dtype=np.float32) for shp in ishapes])))

  print("ready to run onnx model", keys, ishapes, file=sys.stderr)
  while 1:
    inputs = []
    for k, shp in zip(keys, ishapes):
      ts = np.product(shp)
      #print("reshaping %s with offset %d" % (str(shp), offset), file=sys.stderr)
      inputs.append(read(ts, (k=='input_img' and tf8_input)).reshape(shp))
    ret = m.run(None, dict(zip(keys, inputs)))
    #print(ret, file=sys.stderr)
    for r in ret:
      write(r)


if __name__ == "__main__":
  print(sys.argv, file=sys.stderr)
  print("Onnx available providers: ", ort.get_available_providers(), file=sys.stderr)
  options = ort.SessionOptions()
  options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
  if 'OpenVINOExecutionProvider' in ort.get_available_providers() and 'ONNXCPU' not in os.environ:
    provider = 'OpenVINOExecutionProvider'
  elif 'CUDAExecutionProvider' in ort.get_available_providers() and 'ONNXCPU' not in os.environ:
    options.intra_op_num_threads = 2
    provider = 'CUDAExecutionProvider'
  else:
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 8
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    provider = 'CPUExecutionProvider'

  try:
    print("Onnx selected provider: ", [provider], file=sys.stderr)
    ort_session = ort.InferenceSession(sys.argv[1], options, providers=[provider])
    print("Onnx using ", ort_session.get_providers(), file=sys.stderr)
    run_loop(ort_session, tf8_input=("--use_tf8" in sys.argv))
  except KeyboardInterrupt:
    pass
