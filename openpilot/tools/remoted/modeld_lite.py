"""modeld_lite: ModelState 的 Windows 安全副本 (供 tools/remoted 在原生 Windows 使用)。

上游 openpilot.selfdrive.modeld.modeld 在模块级 import msgq.visionipc / cereal
(Linux 专属), 原生 Windows 无法导入。本文件复刻 ModelState 中远程推理所需的
全部功能, 且不引入任何 msgq/cereal/hardware 依赖。

与上游同步义务: 本类是 openpilot/selfdrive/modeld/modeld.py ModelState 的镜像,
上游 run()/warmup() 逻辑变化时必须同步修改 (与 remote_modeld_server.run_raw 相同约定)。
"""
import os

import numpy as np

from openpilot.common.file_chunker import open_file_chunked
from openpilot.selfdrive.modeld.compile_modeld import (MODELD_INPUTS, make_input_queues,
                                                       nv12_copy_size)
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.helpers import load_oob, modeld_pkl_path
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

SEND_RAW_PRED = os.environ.get("SEND_RAW_PRED", "0") == "1"


class ModelState:
  """与 modeld.ModelState 行为一致 (不含 visionipc/cereal 依赖)。"""
  prev_desire: np.ndarray

  def __init__(self, cam_w: int, cam_h: int, chestnut: bool):
    jits = load_oob(open_file_chunked(modeld_pkl_path(chestnut)))
    input_devices = jits['input_devices']
    self.model_device = input_devices['model']
    metadata = jits['metadata']
    self.input_shapes = metadata['input_shapes']
    self.vision_input_names = [k for k in self.input_shapes if 'img' in k]
    self.output_slices = metadata['output_slices']

    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    self.chestnut = chestnut

    self.frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ
    self.frame_copy_size = nv12_copy_size(*get_nv12_info(cam_w, cam_h)[:3])
    self.input_queues, self.npy, self.frame_views = make_input_queues(
      self.input_shapes, self.frame_skip, device=self.model_device, frame_copy_size=self.frame_copy_size)
    self.parser = Parser()
    self.run_model = jits['run_model'][(cam_w, cam_h)]

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    return {k: model_outputs[np.newaxis, v] for k, v in output_slices.items()}

  def run(self, bufs: dict, transforms: dict, inputs: dict, after_enqueue=None) -> dict[str, np.ndarray]:
    for key, buf in bufs.items():
      np.copyto(self.frame_views[key], np.frombuffer(buf.data, dtype=np.uint8, count=self.frame_copy_size))

    inputs['desire_pulse'][0] = 0
    self.npy['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']
    self.npy['traffic_convention'][:] = inputs['traffic_convention']
    self.npy['action_t'][:] = inputs['action_t']
    self.npy['tfm'][:, :] = transforms['img'][:, :]
    self.npy['big_tfm'][:, :] = transforms['big_img'][:, :]

    outs, = self.run_model(**{k: self.input_queues[k] for k in MODELD_INPUTS})
    if after_enqueue is not None:
      after_enqueue()
    model_output = outs.numpy()[0]
    if self.chestnut and not np.all(np.isfinite(model_output)):
      raise RuntimeError("model output not finite")
    outputs_dict = self.parser.parse_outputs(self.slice_outputs(model_output, self.output_slices))
    self.npy['prev_feat'][:] = model_output[self.output_slices['hidden_state']]

    if SEND_RAW_PRED:
      outputs_dict['raw_pred'] = model_output.copy()
    return outputs_dict

  def warmup(self) -> None:
    dummy_frames = {k: np.zeros(self.frame_copy_size, dtype=np.uint8) for k in self.vision_input_names}
    eye = np.eye(3, dtype=np.float32)
    dims = {k: v for k, v in {'desire_pulse': (ModelConstants.DESIRE_LEN,), 'traffic_convention': (2,),
                              'action_t': (2,)}.items()}
    self.run(dummy_frames, dict.fromkeys(self.vision_input_names, eye),
             {k: np.zeros(v, dtype=np.float32) for k, v in dims.items()})
    self.input_queues, self.npy, self.frame_views = make_input_queues(
      self.input_shapes, self.frame_skip, device=self.model_device, frame_copy_size=self.frame_copy_size)
    self.prev_desire[:] = 0
