#!/usr/bin/env python3
import os
import time
import sys

# PC dev helper: if a fake_mapd_inproc.py is reachable from PYTHONPATH,
# spawn an in-process thread that publishes synthetic mapd/gps/carState to the
# ui's msgq context. Real-device runs don't ship this module so the import
# silently no-ops.
try:
  from fake_mapd_inproc import spawn as _spawn_fake_mapd
  print(f"[ui] fake_mapd_inproc found at __file__={_spawn_fake_mapd.__module__}; sys.path[0:5]={sys.path[0:5]}", flush=True)
  _spawn_fake_mapd()
  print("[ui] fake_mapd_inproc spawn ok", flush=True)
except Exception as _e:
  print(f"[ui] fake_mapd_inproc not loaded (ok for device): {_e}", flush=True)

from openpilot.cereal import messaging
from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.common.realtime import Priority, config_realtime_process, set_core_affinity
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
from openpilot.selfdrive.ui.ui_state import ui_state

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
