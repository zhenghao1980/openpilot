#!/usr/bin/env python3
import os
import time
import sys

from openpilot.cereal import messaging
from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.common.realtime import Priority, config_realtime_process, set_core_affinity
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
# publisher-first 双保险：等 fake_mapd 建好 socket 文件再创建 ui_state 的 SubMaster。
# 背景：msgq shm 文件"谁先连谁创建"；若 SUB 抢先 O_CREAT 会出另一个 inode，
# 之后 publisher 再连 unlink+recreate → UI 的 fd 指向已删文件（lsof 显示 DEL）、
# alive 恒 False。独立进程晚启动所以正常。仅测试模式启用（env 门控）。
import os as _os
import time as _time
if _os.environ.get("MAPD_FORCE_STARTED") == "1":
    _deadline = _time.time() + 30
    while _time.time() < _deadline and not _os.path.exists("/dev/shm/msgq_mapdExtendedOut"):
        _time.sleep(0.2)

from openpilot.selfdrive.ui.ui_state import ui_state

# PC 地图调试：MAPD_FORCE_STARTED=1 时强制 started/ignition（run_ui_tizi_local 设置）。
# 必须在 UI 子进程内 patch：wrapper 顶层 import 本模块会拉起 raylib 模块链
# （无窗口时可能进程级 abort）并在 fork 前创建 msgq socket（跨 fork 会炸）。
if os.environ.get("MAPD_FORCE_STARTED") == "1":
  try:
    from openpilot.selfdrive.ui.ui_state import UIState as _UIState
    _orig_update = _UIState.update
    def _update_force_started(self, *a, **kw):
      _orig_update(self, *a, **kw)
      self.started = True
      self.ignition = True
    _UIState.update = _update_force_started
    print("[ui] UIState.update patched (MAPD_FORCE_STARTED=1)", flush=True)
    # 重建 SubMaster：socket/poller 全部重新诞生于 UI 子进程内。
    # 背景：alive=False 但双方已同 inode——socket 本体处于"注册/等待路径失效"的
    # 废状态（继承自 fork 前、或早连时序异常），重建是对所有此类状态的重置。
    try:
      from openpilot.cereal import messaging as _msg
      _svcs = list(ui_state.sm.sock.keys())
      ui_state.sm = _msg.SubMaster(_svcs)
      print(f"[ui] SubMaster rebuilt in UI child ({len(_svcs)} svcs)", flush=True)
    except Exception as _e:
      print(f"[ui] sm rebuild failed: {_e!r}", flush=True)
  except Exception as _e:
    print(f"[ui] UIState patch failed: {_e}", flush=True)

BIG_UI = gui_app.big_ui()


def main():
  cores = {5, }
  # above plannerd and radard
  config_realtime_process(0, Priority.CTRL_HIGH)

  gui_app.init_window("UI")
  if BIG_UI:
    MainLayout()
  else:
    MiciMainLayout()

  pm = messaging.PubMaster(['uiDebug'])
  for should_render, frame_time, cpu_time in gui_app.render():
    extra_start = time.monotonic()
    ui_state.update()

    if should_render:
      # reaffine after power save offlines our core
      if COMMA_HARDWARE and os.sched_getaffinity(0) != cores:
        try:
          set_core_affinity(list(cores))
        except OSError:
          pass

      extra_cpu = time.monotonic() - extra_start
      msg = messaging.new_message('uiDebug')
      msg.uiDebug.cpuTimeMillis = (cpu_time + extra_cpu) * 1000
      msg.uiDebug.frameTimeMillis = frame_time * 1000
      pm.send('uiDebug', msg)


if __name__ == "__main__":
  main()
