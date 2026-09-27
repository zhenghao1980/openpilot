@0xb526ba661d550a59;

# This is a "dictionary" of structs for offroad/custom features.
# The reserved structs are for future use, and should not be changed.
# If you want to add a new struct, add it to the end of the file.

struct CustomReserved0 @0x81c2f05a394cf4af {
}

struct CustomReserved1 @0xaedffd8f31e7b55d {
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

# === mapd (pfeiferj/mapd) integration: reserved slots 17/18/19 in use ======
# upstream ids intentionally reused; see cereal/custom/custom.capnp @ pfeiferj/mapd

enum WaySelectionType {
  current @0;
  predicted @1;
  possible @2;
  extended @3;
  fail @4;
}

enum RoadContext {
  freeway @0;
  city @1;
  unknown @2;
}

enum HighwayClass {
  unknown @0;
  motorway @1;
  motorwayLink @2;
  trunk @3;
  trunkLink @4;
  primary @5;
  primaryLink @6;
  secondary @7;
  secondaryLink @8;
  tertiary @9;
  tertiaryLink @10;
  unclassified @11;
  residential @12;
  livingStreet @13;
}

enum SpeedLimitOffsetType {
  static @0;
  percent @1;
}

enum MapdInputType {
  download @0;
  reloadSettings @1;
  saveSettings @2;
  loadDefaultSettings @3;
  loadRecommendedSettings @4;
  loadPersistentSettings @5;
  cancelDownload @6;
  acceptSpeedLimit @7;
  setJsonPathFloat @8;
  setJsonPathText @9;
  setJsonPathBool @10;
  # DEPRECATED direct setters removed: UI must use the setJsonPath* commands.
}

struct MapdDownloadLocationDetails @0xff889853e7b0987f {
  location @0 :Text;
  totalFiles @1 :UInt32;
  downloadedFiles @2 :UInt32;
}

struct MapdDownloadProgress @0xfaa35dcac85073a2 {
  active @0 :Bool;
  cancelled @1 :Bool;
  totalFiles @2 :UInt32;
  downloadedFiles @3 :UInt32;
  locations @4 :List(Text);
  locationDetails @5 :List(MapdDownloadLocationDetails);
}

struct MapdPathPoint @0xd6f78acca1bc3939 {
  latitude @0 :Float64;
  longitude @1 :Float64;
  curvature @2 :Float32;
  targetVelocity @3 :Float32;
}

struct MapdPosition @0xde9705979aca8339 {
  latitude @0 :Float64;
  longitude @1 :Float64;
}

struct MapdExtendedOut @0xa30662f84033036c {
  downloadProgress @0 :MapdDownloadProgress;
  settings @1 :Text;
  path @2 :List(MapdPathPoint);
  position @3 :MapdPosition;
  loopRateAverage @4 :Float32;
  loopRateMin @5 :Float32;
  # fork extension (mapd 功能说明书 4.3): nearby road network for the map panel
  nearbyRoads @6 :List(MapdRoadSegment);
}

struct MapdRoadSegment @0xf8ea8b5d5454d805 {
  highwayClass @0 :HighwayClass;
  name @1 :Text;
  ref @2 :Text;
  points @3 :List(MapdPosition);
}

struct MapdIn @0xc86a3d38d13eb3ef {
  type @0 :MapdInputType;
  float @1 :Float32;
  str @2 :Text;
  bool @3 :Bool;
  jsonPath @4 :Text;
}

struct MapdOut @0xa4f1eb3323f5f582 {
  wayName @0 :Text;
  wayRef @1 :Text;
  roadName @2 :Text;
  speedLimit @3 :Float32;
  nextSpeedLimit @4 :Float32;
  nextSpeedLimitDistance @5 :Float32;
  hazard @6 :Text;
  nextHazard @7 :Text;
  nextHazardDistance @8 :Float32;
  advisorySpeed @9 :Float32;
  nextAdvisorySpeed @10 :Float32;
  nextAdvisorySpeedDistance @11 :Float32;
  oneWay @12 :Bool;
  lanes @13 :UInt8;
  tileLoaded @14 :Bool;
  speedLimitSuggestedSpeed @15 :Float32;
  suggestedSpeed @16 :Float32;
  estimatedRoadWidth @17 :Float32;
  roadContext @18 :RoadContext;
  distanceFromWayCenter @19 :Float32;
  visionCurveSpeed @20 :Float32;
  mapCurveSpeed @21 :Float32;
  waySelectionType @22 :WaySelectionType;
  speedLimitAccepted @23 :Bool;
  highwayClass @24 :HighwayClass;
  wayId @25 :Int64;
  conditionalSpeedLimit @26 :Text;
}

# SCC-X state message (mapd 功能说明书 3.8): full observability of the curve
# arbitration for road-test analysis and process_replay regression.
struct SccXState @0xe25000ff21448a7e {
  enabled @0 :Bool;
  mapEnabled @1 :Bool;
  state @2 :Text;        # disabled/enabled/overriding/entering/turning/leaving
  source @3 :Text;       # none/vision_a/vision_b/map
  active @4 :Bool;
  vCruiseCap @5 :Float32;
  aTarget @6 :Float32;
  debugVTarget @7 :Float32;
  vMap @8 :Float32;       # MapCurveEstimator.v_target (raw, pre-clamp)
  confMap @9 :Float32;
  confVisionA @10 :Float32;
  confVisionB @11 :Float32;
  vArb @12 :Float32;
  mapReachDistance @13 :Float32;
}

# maprenderd IPC (mapd 功能说明书 B 方案): UI -> 渲染进程相机 / 渲染进程 -> UI 帧
struct MapRenderCam @0x9a3b6c1d2e4f7081 {
  lat @0 :Float64;
  lon @1 :Float64;
  zoom @2 :Float32;
  bearing @3 :Float32;
  width @4 :UInt32;
  height @5 :UInt32;
  pitch @6 :Float32;   # 俯仰角（度）：0=俯视，60=3D 街景
}

struct MapRenderFrame @0x8b2a5d0c3f1e69072 {
  width @0 :UInt32;
  height @1 :UInt32;
  seq @2 :UInt64;
  img @3 :Data;   # QOI encoded RGBA
}
