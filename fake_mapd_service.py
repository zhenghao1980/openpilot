#!/usr/bin/env python3
"""Fake mapd service for PC UI testing (mapd-sccx-full-v0.7c).

Publishes mapdOut / mapdExtendedOut / gpsLocationExternal / carState with
simulated data for a 2km southbound trip on Changchun Yatai Street,
including one curve near Nanhu Square.

Run in parallel with run_ui_tizi_local.py:
  python fake_mapd_service.py
"""
import math
import time
from openpilot.cereal import messaging, custom


GPS_PATH = [
  (43.81800, 125.27800, 0.0),
  (43.81680, 125.27800, 0.0),
  (43.81560, 125.27800, 0.0),
  (43.81440, 125.27800, 0.0),
  (43.81320, 125.27800, 0.0),
  (43.81200, 125.27800, 0.0),
  (43.81080, 125.27800, 0.0),
  (43.80960, 125.27800, 0.0),
  (43.80840, 125.27800, 0.0),
  (43.80720, 125.27800, 0.0),
  (43.80600, 125.27750, 0.0001),
  (43.80550, 125.27620, 0.0008),
  (43.80540, 125.27470, 0.0008),
  (43.80580, 125.27340, 0.0004),
  (43.80640, 125.27250, 0.0001),
  (43.80760, 125.27220, 0.0),
  (43.80880, 125.27200, 0.0),
  (43.81000, 125.27180, 0.0),
  (43.81120, 125.27160, 0.0),
  (43.81240, 125.27140, 0.0),
]

NEARBY_ROADS = [
  (3, "Nanhu East Rd", "E10", [(43.818, 125.282), (43.798, 125.282)]),
  (5, "Ziyou Rd", "G302", [(43.812, 125.265), (43.812, 125.290)]),
]


def gps_bearing_deg(lat1, lon1, lat2, lon2):
  lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
  dlon = lon2 - lon1
  x = math.sin(dlon) * math.cos(lat2)
  y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
  return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def build_mapd_extended(cur_lat, cur_lon):
  me = custom.MapdExtendedOut.new_message()
  me.position.latitude = float(cur_lat)
  me.position.longitude = float(cur_lon)
  me.loopRateAverage = 1.0
  me.loopRateMin = 0.95
  pts = []
  for plat, plon, pcurve in GPS_PATH:
    p = custom.MapdPathPoint.new_message()
    p.latitude = float(plat)
    p.longitude = float(plon)
    p.curvature = float(pcurve)
    p.targetVelocity = 9.0 if pcurve > 0.0005 else 12.5
    pts.append(p)
  me.path = pts
  segs = []
  for hi_cls, name, ref, segpts in NEARBY_ROADS:
    s = custom.MapdRoadSegment.new_message()
    s.highwayClass = hi_cls
    s.name = name
    s.ref = ref
    rpts = []
    for rlat, rlon in segpts:
      rp = custom.MapdPosition.new_message()
      rp.latitude = float(rlat)
      rp.longitude = float(rlon)
      rpts.append(rp)
    s.points = rpts
    segs.append(s)
  me.nearbyRoads = segs
  return me


def build_mapd_out(cur_curve):
  mo = custom.MapdOut.new_message()
  mo.roadName = "Yatai Street"
  mo.wayRef = "G302"
  mo.tileLoaded = True
  mo.waySelectionType = 0  # current
  mo.mapCurveSpeed = 9.0 if cur_curve > 0.0005 else 12.5
  mo.visionCurveSpeed = 11.0
  mo.speedLimit = 12.5
  mo.lanes = 2
  mo.highwayClass = 5  # primary
  return mo


def main():
  print("[fake-mapd] starting; %d GPS points" % len(GPS_PATH), flush=True)
  pm = messaging.PubMaster(['mapdOut', 'mapdExtendedOut', 'gpsLocationExternal', 'carState'])
  time.sleep(2.0)

  for tick in range(60 * 60):
    idx = tick % len(GPS_PATH)
    cur_lat, cur_lon, cur_curve = GPS_PATH[idx]
    nxt_lat, nxt_lon, _ = GPS_PATH[(idx + 1) % len(GPS_PATH)]
    bearing = gps_bearing_deg(cur_lat, cur_lon, nxt_lat, nxt_lon)

    # gpsLocationExternal
    g = messaging.new_message('gpsLocationExternal')
    g.gpsLocationExternal.latitude = float(cur_lat)
    g.gpsLocationExternal.longitude = float(cur_lon)
    g.gpsLocationExternal.bearingDeg = float(bearing)
    g.gpsLocationExternal.horizontalAccuracy = 5.0
    g.gpsLocationExternal.speed = 13.5
    g.gpsLocationExternal.unixTimestampMillis = int(time.time() * 1e3)
    pm.send('gpsLocationExternal', g)

    # carState
    cs = messaging.new_message('carState')
    cs.carState.vEgo = 13.5
    cs.carState.aEgo = 0.0
    pm.send('carState', cs)

    # mapdOut
    mo = messaging.new_message('mapdOut')
    mo.mapdOut = build_mapd_out(cur_curve)
    pm.send('mapdOut', mo)

    # mapdExtendedOut
    me = messaging.new_message('mapdExtendedOut')
    me.mapdExtendedOut = build_mapd_extended(cur_lat, cur_lon)
    pm.send('mapdExtendedOut', me)

    print(f"[fake-mapd] tick={tick} idx={idx} pos=({cur_lat:.5f},{cur_lon:.5f}) curve={cur_curve}", flush=True)
    time.sleep(1.0)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
