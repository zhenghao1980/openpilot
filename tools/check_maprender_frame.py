#!/usr/bin/env python3
"""独立验证 mapRenderFrame 通路（绕过 UI，直接订阅）。

用法（与 UI/maprenderd 同机运行）：
  python tools/check_maprender_frame.py
判定：
  连续收到 3 帧 -> PASS（帧通路正常，问题在 UI 侧解码/贴图）
  一直 waiting  -> FAIL（maprenderd 侧问题，配合 [dbg] 日志定位）
"""
from openpilot.cereal import messaging


def main():
  sm = messaging.SubMaster(["mapRenderFrame"])
  n = 0
  while n < 3:
    sm.update(1000)
    if sm.updated["mapRenderFrame"]:
      f = sm["mapRenderFrame"]
      n += 1
      print(f"[check] frame seq={f.seq} {f.width}x{f.height} img={len(f.img)}B", flush=True)
    else:
      print(f"[check] waiting... valid={sm.valid['mapRenderFrame']} got={n}", flush=True)
  print("[check] PASS：mapRenderFrame 通路正常")


if __name__ == "__main__":
  main()
