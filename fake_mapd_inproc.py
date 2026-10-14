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


# Real-road circuit in Beijing CBD, RESAMPLED along china.mbtiles z14 road
# centerlines (12 m step) so the car icon hugs the rendered roads — the old
# hand-drawn rectangle ran 10-25 m off the centerlines and made the icon look
# persistently off the road:
#   A(39.9132,116.4441) --景华南街(east)--> B(39.9132,116.4517)
#   B --金桐东路(north)--> C(39.9179,116.4517)
#   C --景华北街(west)--> D(39.9179,116.4441)
#   D --东大桥路(south)--> A
GPS_PATH = [
  (39.913135, 116.444280, 0.0),  # 景华南街
  (39.913131, 116.444421, 0.0),
  (39.913133, 116.444561, 0.0),
  (39.913135, 116.444702, 0.0),
  (39.913137, 116.444843, 0.0),
  (39.913139, 116.444983, 0.0),
  (39.913141, 116.445124, 0.0),
  (39.913142, 116.445265, 0.0),
  (39.913144, 116.445405, 0.0),
  (39.913146, 116.445546, 0.0),
  (39.913148, 116.445687, 0.0),
  (39.913150, 116.445827, 0.0),
  (39.913152, 116.445968, 0.0),
  (39.913153, 116.446109, 0.0),
  (39.913155, 116.446249, 0.0),
  (39.913157, 116.446390, 0.0),
  (39.913159, 116.446531, 0.0),
  (39.913161, 116.446671, 0.0),
  (39.913163, 116.446812, 0.0),
  (39.913164, 116.446953, 0.0),
  (39.913166, 116.447093, 0.0),
  (39.913168, 116.447234, 0.0),
  (39.913170, 116.447375, 0.0),
  (39.913172, 116.447516, 0.0),
  (39.913173, 116.447656, 0.0),
  (39.913174, 116.447797, 0.0),
  (39.913175, 116.447938, 0.0),
  (39.913177, 116.448078, 0.0),
  (39.913178, 116.448219, 0.0),
  (39.913179, 116.448360, 0.0),
  (39.913180, 116.448500, 0.0),
  (39.913181, 116.448641, 0.0),
  (39.913182, 116.448782, 0.0),
  (39.913184, 116.448922, 0.0),
  (39.913185, 116.449063, 0.0),
  (39.913186, 116.449204, 0.0),
  (39.913187, 116.449344, 0.0),
  (39.913189, 116.449485, 0.0),
  (39.913192, 116.449626, 0.0),
  (39.913194, 116.449766, 0.0),
  (39.913197, 116.449907, 0.0),
  (39.913200, 116.450048, 0.0),
  (39.913203, 116.450188, 0.0),
  (39.913205, 116.450329, 0.0),
  (39.913208, 116.450470, 0.0),
  (39.913211, 116.450610, 0.0),
  (39.913213, 116.450751, 0.0),
  (39.913216, 116.450892, 0.0),
  (39.913219, 116.451032, 0.0),
  (39.913222, 116.451173, 0.0),
  (39.913224, 116.451314, 0.0),
  (39.913210, 116.451440, 0.0),
  (39.913210, 116.451440, 0.0),  # 金桐东路
  (39.913318, 116.451438, 0.0),
  (39.913426, 116.451435, 0.0),
  (39.913534, 116.451433, 0.0),
  (39.913642, 116.451430, 0.0),
  (39.913750, 116.451428, 0.0),
  (39.913857, 116.451425, 0.0),
  (39.913965, 116.451423, 0.0),
  (39.914073, 116.451420, 0.0),
  (39.914181, 116.451418, 0.0),
  (39.914289, 116.451416, 0.0),
  (39.914397, 116.451415, 0.0),
  (39.914505, 116.451418, 0.0),
  (39.914613, 116.451422, 0.0),
  (39.914721, 116.451425, 0.0),
  (39.914828, 116.451428, 0.0),
  (39.914936, 116.451432, 0.0),
  (39.915044, 116.451435, 0.0),
  (39.915152, 116.451438, 0.0),
  (39.915260, 116.451441, 0.0),
  (39.915368, 116.451445, 0.0),
  (39.915476, 116.451448, 0.0),
  (39.915584, 116.451451, 0.0),
  (39.915692, 116.451454, 0.0),
  (39.915799, 116.451458, 0.0),
  (39.915907, 116.451461, 0.0),
  (39.916015, 116.451464, 0.0),
  (39.916123, 116.451468, 0.0),
  (39.916231, 116.451471, 0.0),
  (39.916339, 116.451474, 0.0),
  (39.916447, 116.451477, 0.0),
  (39.916555, 116.451481, 0.0),
  (39.916663, 116.451484, 0.0),
  (39.916770, 116.451487, 0.0),
  (39.916878, 116.451490, 0.0),
  (39.916986, 116.451493, 0.0),
  (39.917094, 116.451496, 0.0),
  (39.917202, 116.451499, 0.0),
  (39.917310, 116.451502, 0.0),
  (39.917418, 116.451505, 0.0),
  (39.917526, 116.451508, 0.0),
  (39.917634, 116.451511, 0.0),
  (39.917742, 116.451514, 0.0),
  (39.917849, 116.451517, 0.0),
  (39.917945, 116.451520, 0.0),
  (39.917945, 116.451520, 0.0),  # 景华北街
  (39.917945, 116.451379, 0.0),
  (39.917945, 116.451239, 0.0),
  (39.917945, 116.451098, 0.0),
  (39.917945, 116.450957, 0.0),
  (39.917945, 116.450816, 0.0),
  (39.917945, 116.450676, 0.0),
  (39.917945, 116.450535, 0.0),
  (39.917945, 116.450394, 0.0),
  (39.917945, 116.450254, 0.0),
  (39.917945, 116.450113, 0.0),
  (39.917945, 116.449972, 0.0),
  (39.917945, 116.449831, 0.0),
  (39.917945, 116.449691, 0.0),
  (39.917945, 116.449550, 0.0),
  (39.917945, 116.449409, 0.0),
  (39.917946, 116.449269, 0.0),
  (39.917947, 116.449128, 0.0),
  (39.917948, 116.448987, 0.0),
  (39.917949, 116.448847, 0.0),
  (39.917950, 116.448706, 0.0),
  (39.917951, 116.448565, 0.0),
  (39.917951, 116.448424, 0.0),
  (39.917952, 116.448284, 0.0),
  (39.917953, 116.448143, 0.0),
  (39.917954, 116.448002, 0.0),
  (39.917955, 116.447862, 0.0),
  (39.917956, 116.447721, 0.0),
  (39.917957, 116.447580, 0.0),
  (39.917956, 116.447440, 0.0),
  (39.917955, 116.447299, 0.0),
  (39.917953, 116.447158, 0.0),
  (39.917952, 116.447017, 0.0),
  (39.917951, 116.446877, 0.0),
  (39.917950, 116.446736, 0.0),
  (39.917948, 116.446595, 0.0),
  (39.917947, 116.446455, 0.0),
  (39.917946, 116.446314, 0.0),
  (39.917944, 116.446173, 0.0),
  (39.917943, 116.446033, 0.0),
  (39.917942, 116.445892, 0.0),
  (39.917940, 116.445751, 0.0),
  (39.917939, 116.445610, 0.0),
  (39.917938, 116.445470, 0.0),
  (39.917937, 116.445329, 0.0),
  (39.917935, 116.445188, 0.0),
  (39.917934, 116.445048, 0.0),
  (39.917933, 116.444907, 0.0),
  (39.917931, 116.444766, 0.0),
  (39.917930, 116.444626, 0.0),
  (39.917929, 116.444485, 0.0),
  (39.917930, 116.444344, 0.0),
  (39.917930, 116.444330, 0.0),
  (39.917930, 116.444330, 0.0),  # 东大桥路
  (39.917872, 116.444256, 0.0),
  (39.917764, 116.444252, 0.0),
  (39.917656, 116.444249, 0.0),
  (39.917548, 116.444245, 0.0),
  (39.917440, 116.444241, 0.0),
  (39.917332, 116.444238, 0.0),
  (39.917224, 116.444234, 0.0),
  (39.917116, 116.444230, 0.0),
  (39.917009, 116.444227, 0.0),
  (39.916901, 116.444223, 0.0),
  (39.916793, 116.444219, 0.0),
  (39.916685, 116.444215, 0.0),
  (39.916577, 116.444212, 0.0),
  (39.916469, 116.444208, 0.0),
  (39.916361, 116.444204, 0.0),
  (39.916253, 116.444204, 0.0),
  (39.916145, 116.444203, 0.0),
  (39.916038, 116.444202, 0.0),
  (39.915930, 116.444202, 0.0),
  (39.915822, 116.444201, 0.0),
  (39.915714, 116.444201, 0.0),
  (39.915606, 116.444200, 0.0),
  (39.915498, 116.444200, 0.0),
  (39.915390, 116.444199, 0.0),
  (39.915282, 116.444197, 0.0),
  (39.915174, 116.444192, 0.0),
  (39.915066, 116.444188, 0.0),
  (39.914959, 116.444187, 0.0),
  (39.914851, 116.444187, 0.0),
  (39.914743, 116.444186, 0.0),
  (39.914635, 116.444186, 0.0),
  (39.914527, 116.444185, 0.0),
  (39.914419, 116.444185, 0.0),
  (39.914311, 116.444184, 0.0),
  (39.914203, 116.444183, 0.0),
  (39.914095, 116.444183, 0.0),
  (39.913987, 116.444182, 0.0),
  (39.913879, 116.444182, 0.0),
  (39.913771, 116.444181, 0.0),
  (39.913664, 116.444181, 0.0),
  (39.913556, 116.444180, 0.0),
  (39.913448, 116.444180, 0.0),
  (39.913340, 116.444179, 0.0),
  (39.913232, 116.444179, 0.0),
  (39.913124, 116.444178, 0.0),
  (39.913016, 116.444177, 0.0),
  (39.912908, 116.444177, 0.0),
  (39.912800, 116.444176, 0.0),
  (39.912692, 116.444176, 0.0),
  (39.912584, 116.444175, 0.0),
  (39.912476, 116.444175, 0.0),
  (39.912369, 116.444174, 0.0),
  (39.912261, 116.444174, 0.0),
  (39.912153, 116.444173, 0.0),
  (39.912045, 116.444172, 0.0),
  (39.911979, 116.444172, 0.0),
  (39.912087, 116.444174, 0.0),
  (39.912195, 116.444176, 0.0),
  (39.912303, 116.444177, 0.0),
  (39.912411, 116.444179, 0.0),
  (39.912519, 116.444181, 0.0),
  (39.912627, 116.444182, 0.0),
  (39.912735, 116.444184, 0.0),
  (39.912842, 116.444186, 0.0),
  (39.912950, 116.444187, 0.0),
  (39.913058, 116.444189, 0.0),
  (39.913135, 116.444231, 0.0),
  (39.913135, 116.444280, 0.0),
]

NEARBY_ROADS = [
  (3, "东三环", "", [(39.9100, 116.4556), (39.9270, 116.4556)]),
  (3, "神路街", "", [(39.9163, 116.4378), (39.9211, 116.4376)]),
  (2, "朝阳门外大街", "G102", [(39.9200, 116.4460), (39.9185, 116.4552)]),
]


# Per-segment road name + ref, keyed by GPS_PATH idx range. Picks the current
# road based on which segment the GPS sits on. roadName changes as GPS advances.
ROAD_BY_IDX = [
  (0, 51, "景华南街", ""),
  (52, 96, "金桐东路", ""),
  (97, 149, "景华北街", ""),
  (150, 218, "东大桥路", ""),
]


def _road_at(idx: int):
  for lo, hi, n, r in ROAD_BY_IDX:
    if lo <= idx <= hi:
      return n, r
  return "东大桥路", ""


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
        # 当前 segment 索引：按 seg_starts 线性查找（各段长度不同，不能用等长假设）
        idx = 0
        for i in range(n):
          if seg_starts[i] <= s_pos < seg_starts[i] + seg_lens[i]:
            idx = i
            break
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
