# Remote big model (C3X ↔ NVIDIA host)

Replaces the chestnut co-processor with a network-connected NVIDIA host for the
big driving model, with automatic fallback to the on-device small model.

## Roles

- **C3X (device)**: patched `modeld` + `openpilot/selfdrive/modeld/remote_model.py`.
  Runs the small model in memory as hot standby; drives the vehicle control loop.
- **NVIDIA host (this tool)**: `remote_modeld_server.py` loads the compiled big
  model and serves inference over TCP.

## Protocol

Length-prefixed little-endian binary framing over TCP (see the module docstring
in `openpilot/selfdrive/modeld/remote_model.py` for the exact layout):

1. `HELLO` (cam dims) → `META` (input shapes, output slices) — also used as the
   availability probe, mirroring `chestnut_present() and chestnut_compiled()`.
2. `INFER` (YUV frames + warp matrices + desire/traffic/action inputs) → raw
   float32 model output vector. Temporal hidden state stays on the host per
   connection and resets on reconnect (chestnut semantics).

## Setup on the NVIDIA host

```bash
# compile the big model for the local NVIDIA device (slow, once)
python openpilot/selfdrive/modeld/compile_modeld.py \
  --model-size 256x128 --camera-resolutions 1928x1208 \
  --onnx openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx \
  --output /tmp/big_driving_tinygrad.pkl --frame-skip 4

# serve it
python openpilot/tools/remoted/remote_modeld_server.py --model /tmp/big_driving_tinygrad.pkl
```

## Setup on the C3X

```bash
export REMOTE_MODEL_HOST=<nvidia-host-ip>   # unset = remote disabled, stock behavior
export REMOTE_MODEL_PORT=8571               # optional
export REMOTE_MODEL_TIMEOUT_MS=300          # optional, per-request timeout
# then start openpilot normally
```

## Fallback semantics

- If the host is unreachable at startup, `modeld` starts on the small model
  (exactly like a missing chestnut).
- If the host fails mid-drive, the next `model.run()` raises and the stock
  chestnut fallback in `modeld.py` switches to the small model for the rest of
  the drive (`ChestnutActive=False`, `bigModelFailed` alert in selfdrived).
- No automatic switch-back while driving; restart or go offroad to retry the
  big model.
