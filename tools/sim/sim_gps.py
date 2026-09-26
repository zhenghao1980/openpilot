#!/usr/bin/env python3
"""WSL 联调用合成数据发布器：让 UI/mapd/maprenderd 在无设备、无摄像头、无 GPS 的环境下跑起来。

发布：gpsLocationExternal（沿 S 弯行驶，喂曲率给 tsc-d）、carState、deviceState
(started=True，点亮 UI 的 started 条件）、selfdriveState、controlsState。
运行：python tools/sim/sim_gps.py    （需先在仓库根目录构建好 cereal/messaging）
"""
import math

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper


def main():
  pm = messaging.PubMaster(["gpsLocationExternal", "carState", "deviceState", "selfdriveState", "controlsState"])
  rk = Ratekeeper(10)
  lat, lon = 39.9830, 116.3070   # 北京中关村一带，需与已下载瓦片区域一致
  bearing = 90.0
  v = 13.9                       # 50 km/h
  t = 0.0
  while True:
    t += 0.1
    bearing = 90.0 + 35.0 * math.sin(t / 25.0)   # S 弯：地图曲率 + 视觉曲率都有料
    lat += math.cos(math.radians(bearing)) * v * 0.1 / 111320.0
    lon += math.sin(math.radians(bearing)) * v * 0.1 / (111320.0 * math.cos(math.radians(lat)))

    g = messaging.new_message("gpsLocationExternal")
    g.gpsLocationExternal.latitude = lat
    g.gpsLocationExternal.longitude = lon
    g.gpsLocationExternal.bearingDeg = (bearing + 360.0) % 360.0
    g.gpsLocationExternal.speed = v
    g.gpsLocationExternal.horizontalAccuracy = 3.0
    g.gpsLocationExternal.verticalAccuracy = 4.0
    g.gpsLocationExternal.bearingAccuracy = 2.0
    pm.send("gpsLocationExternal", g)

    c = messaging.new_message("carState")
    c.carState.vEgo = v
    c.carState.vCruise = 80
    c.carState.standstill = False
    c.carState.gasPressed = False
    c.carState.steeringAngleDeg = 0.0
    pm.send("carState", c)

    d = messaging.new_message("deviceState")
    d.deviceState.started = True
    d.deviceState.screenBrightnessPercent = 100
    d.deviceState.networkType = log.DeviceState.NetworkType.none
    d.deviceState.networkStrength = log.DeviceState.NetworkStrength.unknown
    d.deviceState.batteryPercent = 80
    d.deviceState.batteryCharging = True
    d.deviceState.ambientTempC = 25.0
    pm.send("deviceState", d)

    s = messaging.new_message("selfdriveState")
    s.selfdriveState.started = True
    s.selfdriveState.state = log.SelfdriveState.OpenpilotState.disabled
    s.selfdriveState.enabled = False
    s.selfdriveState.experimentalMode = False
    pm.send("selfdriveState", s)

    k = messaging.new_message("controlsState")
    k.controlsState.longControlState = log.ControlsState.LongControlState.off
    k.controlsState.forceDecel = False
    k.controlsState.curvature = math.radians(35.0 * math.cos(t / 25.0) / 25.0) * v / 25.0
    pm.send("controlsState", k)

    rk.keep_time()


if __name__ == "__main__":
  main()
