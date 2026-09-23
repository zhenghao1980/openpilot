import io
import pickle
import struct
import sys
from pathlib import Path

from openpilot.common.hardware import AGNOS
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, USB_DEVICES_PATH, is_chestnut_usb_id

MODELS_DIR = Path(__file__).resolve().parent / 'models'


def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def load_oob(path, chestnut=False):
  from tinygrad import Context, Tensor, dtypes
  from tinygrad.device import Buffer, Device
  device = 'USB+AMD:LLVM' if chestnut else 'QCOM' if AGNOS else 'METAL' if sys.platform == 'darwin' else 'CPU:LLVM'
  with Context(DEV=device):
    # vendored from tinygrad examples/openpilot/helpers.py (removed upstream after the
    # f6fc4e3f2 pin): persistent-id out-of-band format produced by compile_onnx.py
    with open(path, "rb") as f:
      opcodes, buffers = f.read(struct.unpack('<q', f.read(8))[0]), Tensor(Path(path))[f.tell():].uop.buffer.ensure_allocated()

    # FIXME: we load in chunks here because hcq_submit for one large copy is very slow to compile
    arena, CHUNK_SIZE = Buffer(Device.DEFAULT, buffers.nbytes, dtypes.uchar, preallocate=True), 32 << 20
    for off in range(0, buffers.nbytes, CHUNK_SIZE):
      size = min(CHUNK_SIZE, buffers.nbytes-off)
      arena.view(size, dtypes.uchar, off).ensure_allocated().copy_from(buffers.view(size, dtypes.uchar, off).ensure_allocated())

    def persistent_load(pid): return arena.view(*pid)

    u = pickle.Unpickler(io.BytesIO(opcodes))
    u.persistent_load = persistent_load
    return u.load()

def chestnut_present() -> bool:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      product = (d / "product").read_text().strip()
      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
        return True
    except Exception:
      pass
  return False

def chestnut_compiled() -> bool:
  path = modeld_pkl_path(chestnut=True)
  return path.is_file() and all(
    (MODELS_DIR / f'big_driving_warp_{size}_tinygrad.pkl').is_file() for size in ('1344x760', '1928x1208'))
