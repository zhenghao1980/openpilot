"""In-process fake mapd for PC UI testing.

Spawn a daemon thread that publishes synthetic GPS / mapdOut / mapdExtendedOut /
carState to the calling process's msgq context. Works in any process that has
`from openpilot.cereal import messaging` importable.

Usage:
  from fake_mapd_inproc import spawn
  spawn()

Idempotent: only spawns one thread per process.
"""
import threading
import time as _t
import math

from openpilot.cereal import messaging, custom


GPS_PATH = [
  (39.9100, 116.4500, 0.0),
  (39.9100, 116.4508, 0.0),
  (39.9100, 116.4516, 0.0),
  (39.9100, 116.4524, 0.0),
  (39.9100, 116.4532, 0.0),
  (39.9100, 116.4540, 0.0),
  (39.9100, 116.4548, 0.0001),
  (39.9100, 116.4556, 0.0008),
  (39.9100, 116.4564, 0.0008),
  (39.9100, 116.4572, 0.0004),
  (39.9100, 116.4580, 0.0001),
  (39.9100, 116.4588, 0.0),
  (39.9100, 116.4596, 0.0),
  (39.9100, 116.4604, 0.0),
  (39.9100, 116.4612, 0.0),
  (39.9100, 116.4620, 0.0),
  (39.9100, 116.4628, 0.0),
  (39.9100, 116.4636, 0.0),
  (39.9100, 116.4644, 0.0),
  (39.9100, 116.4652, 0.0),
]

NEARBY_ROADS = [
  (3, "Chang'an Ave East", "G102", [(39.910, 116.450), (39.910, 116.470)]),
  (5, "Jianguo Outer Ring", "S50", [(39.913, 116.460), (39.901, 116.460)]),
]


# Per-segment road name + ref, keyed by GPS_PATH idx range. Picks the current
# road based on which segment the GPS sits on. roadName changes as GPS advances.
# Format: "中文 / English" so the nav card shows both languages.
ROAD_BY_IDX = [
  (0, 4, "长安街东延 / Chang'an Ave E", "G102"),
  (5, 8, "东大桥路 / Dongdaqiao Rd", "S41"),
  (9, 12, "通惠河北路 / Tonghuihe N Rd", "G1"),
  (13, 16, "亚运村东路 / Yayuncun E Rd", "S32"),
  (17, 19, "来广营西路 / Laiguangying W Rd", "S310"),
]


def _road_at(idx: int):
  for lo, hi, n, r in ROAD_BY_IDX:
    if lo <= idx <= hi:
      return n, r
  return "长安街东延 / Chang'an Ave E", "G102"


def _bearing(lat1, lon1, lat2, lon2):
  lat1, lon1, lat2, lon2 = math.radians(lat1), math.radians(lon1), math.radians(lat2), math.radians(lon2)
  dlon = lon2 - lon1
  x = math.sin(dlon) * math.cos(lat2)
  y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
  return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


_started = False
_lock = threading.Lock()


def spawn():
  global _started
  with _lock:
    if _started:
      return
    _started = True

  def loop():
    pm = messaging.PubMaster(['mapdOut', 'mapdExtendedOut', 'gpsLocationExternal', 'carState'])
    print("[fake-mapd-inproc] thread starting", flush=True)
    _t.sleep(3.0)
    # Block until SubMaster has subscribed (conflate drops messages sent before any subscriber is bound)
    for s in ('mapdOut', 'mapdExtendedOut', 'gpsLocationExternal', 'carState'):
      ok = pm.wait_for_readers_to_update(s, timeout=10)
      print(f"[fake-mapd-inproc] {s} readers_ready={ok}", flush=True)
    tick = 0
    n = len(GPS_PATH)
    # Real commaai cadences: gpsLocationExternal 10Hz, carState 100Hz,
    # mapdOut / mapdExtendedOut 1Hz. We mimic 10Hz for GPS so the on-screen
    # arrow + breadcrumb trail glide along the path instead of jumping 1 grid
    # per second; mapd data refreshes once a second as it does on-device.
    _t_per_gps = 0.1  # 10 Hz
    next_mapd_t = 0.0
    last_gps_t = 0.0
    # Compute path total length (m) once so we can interpolate along arc-length.
    def _seg_m(a_lat, a_lon, b_lat, b_lon):
      R = 6371000.0
      lat1, lat2 = math.radians(a_lat), math.radians(b_lat)
      dlat = lat2 - lat1
      dlon = math.radians(b_lon - a_lon)
      x = dlon * math.cos((lat1 + lat2) / 2.0)
      return R * math.sqrt(dlat * dlat + x * x)
    seg_lens = [(_seg_m(GPS_PATH[i][0], GPS_PATH[i][1], GPS_PATH[(i+1)%n][0], GPS_PATH[(i+1)%n][1])) for i in range(n)]
    total_len = sum(seg_lens)
    seg_starts = []
    acc = 0.0
    for L in seg_lens:
      seg_starts.append(acc)
      acc += L

    def _pos_at(s_m: float):
      """Return (lat, lon, bearing) at arc-length s_m (mod total_len) along GPS_PATH."""
      s_m = s_m % total_len
      # find segment
      for i in range(n):
        if seg_starts[i] <= s_m < seg_starts[i] + seg_lens[i]:
          t = (s_m - seg_starts[i]) / max(seg_lens[i], 1e-6)
          a_lat, a_lon, _ = GPS_PATH[i]
          b_lat, b_lon, _ = GPS_PATH[(i+1) % n]
          lat = a_lat + t * (b_lat - a_lat)
          lon = a_lon + t * (b_lon - a_lon)
          brg = _bearing(a_lat, a_lon, b_lat, b_lon)
          return lat, lon, brg
      return GPS_PATH[0][0], GPS_PATH[0][1], 0.0

    v_ego = 13.5  # m/s — keeps path zoom stable
    s_pos = 0.0   # arc-length position along path
    while True:
      now = _t.monotonic()
      # 1) GPS at 10 Hz: arc-length s_pos advances by v_ego * dt
      if now - last_gps_t >= _t_per_gps:
        dt = now - last_gps_t
        s_pos = (s_pos + v_ego * dt) % total_len
        last_gps_t = now
        clat, clon, brg = _pos_at(s_pos)
        g = messaging.new_message('gpsLocationExternal')
        g.valid = True
        g.gpsLocationExternal.latitude = float(clat)
        g.gpsLocationExternal.longitude = float(clon)
        g.gpsLocationExternal.bearingDeg = float(brg)
        g.gpsLocationExternal.horizontalAccuracy = 5.0
        g.gpsLocationExternal.speed = v_ego
        g.gpsLocationExternal.unixTimestampMillis = int(_t.time() * 1e3)
        try:
          pm.send('gpsLocationExternal', g)
        except Exception as e:
          print(f"[fake-mapd-inproc] gps send fail: {e}", flush=True)
        if tick % 10 == 0:
          print(f"[fake-mapd-inproc] tick={tick} gps s={s_pos:.1f}m ({clat:.5f},{clon:.5f})", flush=True)

      # 2) carState every 100ms (10Hz; UI only reads vEgo, fine)
      cs = messaging.new_message('carState')
      cs.valid = True
      cs.carState.vEgo = v_ego
      cs.carState.aEgo = 0.0
      try:
        pm.send('carState', cs)
      except Exception as e:
        print(f"[fake-mapd-inproc] carState send fail: {e}", flush=True)

      # mapdOut fires once per second (1 Hz) — same as real commaai.
      if now >= next_mapd_t:
        next_mapd_t = now + 1.0
        # current segment curvature = curvature of GPS_PATH[idx]
        idx = int(s_pos / max(seg_lens[0], 1e-6)) % n if seg_lens else 0
        ccrv = GPS_PATH[idx][2] if GPS_PATH[idx] else 0.0
        mo = messaging.new_message('mapdOut')
        mo.valid = True
        road_name, road_ref = _road_at(idx)
        mo.mapdOut.roadName = road_name
        mo.mapdOut.wayRef = road_ref
        mo.mapdOut.tileLoaded = True
        mo.mapdOut.waySelectionType = "current"
        mo.mapdOut.mapCurveSpeed = 9.0 if ccrv > 0.0005 else 12.5
        mo.mapdOut.visionCurveSpeed = 11.0
        mo.mapdOut.speedLimit = 12.5
        mo.mapdOut.lanes = 2
        mo.mapdOut.highwayClass = 5
        mo.mapdOut.hazard = ""
        mo.mapdOut.nextHazard = "traffic_signal"
        mo.mapdOut.nextHazardDistance = 80.0
        try:
          pm.send('mapdOut', mo)
        except Exception as e:
          print(f"[fake-mapd-inproc] mapdOut send fail: {e}", flush=True)

        me = custom.MapdExtendedOut.new_message()
        me.position.latitude = float(clat)
        me.position.longitude = float(clon)
        me.loopRateAverage = 1.0
        me.loopRateMin = 0.95
        me.path = [custom.MapdPathPoint.new_message() for _ in GPS_PATH]
        for i, (plat, plon, pcrv) in enumerate(GPS_PATH):
          me.path[i].latitude = float(plat)
          me.path[i].longitude = float(plon)
          me.path[i].curvature = float(pcrv)
          me.path[i].targetVelocity = 9.0 if pcrv > 0.0005 else 12.5
        me.nearbyRoads = [custom.MapdRoadSegment.new_message() for _ in NEARBY_ROADS]
        for j, (hcls, name, ref, pts) in enumerate(NEARBY_ROADS):
          me.nearbyRoads[j].highwayClass = hcls
          me.nearbyRoads[j].name = name
          me.nearbyRoads[j].ref = ref
          me.nearbyRoads[j].points = [custom.MapdPosition.new_message() for _ in pts]
          for k, (rlat, rlon) in enumerate(pts):
            me.nearbyRoads[j].points[k].latitude = float(rlat)
            me.nearbyRoads[j].points[k].longitude = float(rlon)
        me_msg = messaging.new_message('mapdExtendedOut')
        me_msg.valid = True
        me_msg.mapdExtendedOut = me
        try:
          pm.send('mapdExtendedOut', me_msg)
        except Exception as e:
          print(f"[fake-mapd-inproc] mapdExtendedOut send fail: {e}", flush=True)

      print(f"[fake-mapd-inproc] tick={tick} idx={idx} pos=({clat:.5f},{clon:.5f}) curve={ccrv}", flush=True)
      tick += 1
      _t.sleep(0.05)

  threading.Thread(target=loop, daemon=True, name="fake-mapd-inproc").start()
