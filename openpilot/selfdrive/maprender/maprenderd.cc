// maprenderd v3: standalone MapLibre GL Native offscreen renderer (mln API, complete).
//
// Data path (all built-in, no custom file source):
//   style.json vector source -> "tiles": ["mbtiles://<abs path>/{z}/{x}/{y}"]
//   -> engine's built-in mln::MBTilesFileSource (sqlite, vendored core)
//   glyphs -> file:// local dir
// Frames: synchronous mln::HeadlessFrontend::render(Map&) -> unpremultiply -> QOI
//   -> cereal "mapRenderFrame"; camera from "mapRenderCam" (UI, 10Hz).
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <mln/gfx/headless_frontend.hpp>
#include <mln/map/camera.hpp>
#include <mln/map/map.hpp>
#include <mln/map/map_observer.hpp>
#include <mln/map/map_options.hpp>
#include <mln/storage/file_source_manager.hpp>
#include <mln/storage/resource_options.hpp>
#include <mln/style/style.hpp>
#include <mln/util/image.hpp>
#include <mln/util/logging.hpp>
#include <mln/util/run_loop.hpp>

// 引擎内置 mbtiles 源（internal header，vendored 源码树内引用，CMake 已加 src include）
#include <mln/storage/mbtiles_file_source.hpp>

#include "cereal/messaging/messaging.h"

#include "qoi.h"

namespace {

std::string loadFile(const char* path) {
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// style.json 本地化：vector 源指向本地 mbtiles 绝对路径（引擎内置源按 URL 取文件），
// glyphs 指到本地目录。最小字符串手术；如需更稳可换 nlohmann/json。
std::string localizeStyle(const std::string& in, const std::string& mbtiles_abs) {
  std::string out = in;
  // TileJSON-style source: the URL is a TileJSON endpoint. MBTilesFileSource reads
  // the file's metadata table and synthesizes a TileJSON with `tiles: ["mbtiles://<abs>/{z}/{x}/{y}"]`.
  // So we MUST pass the file path (no template) as the source url.
  for (const std::string& alias : {"openmaptiles", "osm"}) {
    const std::string from_plain = "\"url\": \"mbtiles://" + alias + "\"";
    const std::string from_braces = "\"url\": \"mbtiles://{" + alias + "}\"";
    const std::string to = "\"url\": \"mbtiles://" + mbtiles_abs + "\"";
    for (const std::string& from : {from_plain, from_braces}) {
      for (auto pos = out.find(from); pos != std::string::npos; pos = out.find(from))
        out.replace(pos, from.size(), to);
    }
  }
  const std::string gfrom = "\"glyphs\": \"https://fonts.openmaptiles.org/{fontstack}/{range}.pbf\"";
  const std::string gto = "\"glyphs\": \"file:///data/mapd_render/glyphs/{fontstack}/{range}.pbf\"";
  auto gp = out.find(gfrom);
  if (gp != std::string::npos) out.replace(gp, gfrom.size(), gto);
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  // CLI: [style.json] [mbtiles] [--render-once lat lon zoom out.png]
  const char* style_path = argc > 1 && argv[1][0] != '-' ? argv[1] : "/data/mapd_render/style.json";
  const std::string mbtiles_path = argc > 2 && argv[2][0] != '-' ? argv[2] : "/data/mapd_render/china.mbtiles";
  const char* asset_root = "/data/mapd_render";

  // --render-once mode: bypass SubMaster/PubMaster, render one frame, write PNG, exit.
  // Used to verify the engine reads china.mbtiles + style.json outside WSLg.
  bool render_once = false;
  double ro_lat = 43.815, ro_lon = 125.281, ro_zoom = 12.0;
  const char* ro_out = "/tmp/maprenderd_test.png";
  for (int i = 1; i < argc; i++) {
    if (std::strcmp(argv[i], "--render-once") == 0 && i + 4 < argc) {
      render_once = true;
      ro_lat = std::atof(argv[i+1]); ro_lon = std::atof(argv[i+2]);
      ro_zoom = std::atof(argv[i+3]); ro_out = argv[i+4];
      break;
    }
  }

  mln::Log::setObserver(std::make_unique<mln::Log::NullObserver>());  // maplibre v5 用 Observer pattern；NullObserver 继承 Observer，不打印

  // Force stdout/stderr line-buffered so manager-piped logs flush each print,
  // not at process exit. Default for redirect-to-file is fully buffered.
  setvbuf(stdout, nullptr, _IOLBF, 0);
  setvbuf(stderr, nullptr, _IOLBF, 0);

  mln::util::RunLoop runLoop(mln::util::RunLoop::Type::Default);  // mln/util/run_loop.hpp 已核实
  (void)runLoop;

  // 内置 mbtiles 源注册到 FileSourceManager（若引擎已预注册同类，此处为幂等替换）
  mln::FileSourceManager::get()->registerFileSourceFactory(
      mln::FileSourceType::Mbtiles,
      [](const mln::ResourceOptions& ro, const mln::ClientOptions& co) {
        return std::make_unique<mln::MBTilesFileSource>(ro, co);
      });

  const float pixelRatio = 1.0f;
  const std::string style_json = localizeStyle(loadFile(style_path), mbtiles_path);
  mln::ResourceOptions resourceOptions;
  resourceOptions.withCachePath(std::string(asset_root) + "/cache.db")
      .withAssetPath(asset_root);
  mln::MapObserver observer;

  uint32_t cur_w = 960, cur_h = 960;
  std::unique_ptr<mln::HeadlessFrontend> frontend;
  std::unique_ptr<mln::Map> map;
  auto rebuild = [&](uint32_t w, uint32_t h) {
    cur_w = w; cur_h = h;
    frontend = std::make_unique<mln::HeadlessFrontend>(mln::Size{w, h}, pixelRatio);
    mln::MapOptions mo;
    mo.withMapMode(mln::MapMode::Static).withSize(frontend->getSize()).withPixelRatio(pixelRatio);
    map = std::make_unique<mln::Map>(*frontend, observer, mo, resourceOptions);
    map->getStyle().loadJSON(style_json);  // mln/style/style.hpp 已核实
  };
  rebuild(cur_w, cur_h);

  if (render_once) {
    // HeadlessFrontend::render() is the canonical API: it internally calls
    // map.renderStill(callback) and waits until the render thread writes the
    // image via backend->readStillImage(). This is what actually fills pixels.
    map->jumpTo(mln::CameraOptions()
                    .withCenter(mln::LatLng(ro_lat, ro_lon))
                    .withZoom(ro_zoom)
                    .withBearing(0));
    auto res = frontend->render(*map);
    auto img = std::move(res.image);
    if (!img.valid()) {
      fprintf(stderr, "[maprenderd] render-once: invalid image\n");
      return 2;
    }
    const uint32_t W = img.size.width, H = img.size.height;
    const uint8_t* src = img.data.get();
    std::vector<uint8_t> rgba(W * H * 4);
    for (uint32_t i = 0; i < W * H; i++) {
      const uint8_t* p = src + i * 4;
      uint8_t a = p[3];
      rgba[i*4+0] = a ? (uint8_t)(p[0] * 255 / a) : 0;
      rgba[i*4+1] = a ? (uint8_t)(p[1] * 255 / a) : 0;
      rgba[i*4+2] = a ? (uint8_t)(p[2] * 255 / a) : 0;
      rgba[i*4+3] = a;
    }
    std::ofstream f(ro_out, std::ios::binary);
    f.write(reinterpret_cast<const char*>(rgba.data()), rgba.size());
    f.close();
    return 0;
  }

  SubMaster sm({"mapRenderCam"});
  PubMaster pm({"mapRenderFrame"});
  printf("[maprenderd] ready (style=%s mbtiles=%s)\n", style_path, mbtiles_path.c_str());

  uint64_t rendered_seq = 0;
  while (true) {
    sm.update(100);
    if (!sm.updated("mapRenderCam")) continue;

    auto c = sm["mapRenderCam"].getMapRenderCam();
    const uint32_t rw = c.getWidth() ? c.getWidth() : cur_w;
    const uint32_t rh = c.getHeight() ? c.getHeight() : cur_h;
    if (rw != cur_w || rh != cur_h) {
      if (rw >= 256 && rw <= 2048 && rh >= 256 && rh <= 2048) rebuild(rw, rh);
    }
    map->jumpTo(mln::CameraOptions()
                    .withCenter(mln::LatLng(c.getLat(), c.getLon()))
                    .withZoom(c.getZoom())
                    .withBearing(c.getBearing()));

    mln::HeadlessFrontend::RenderResult res;
    for (int i = 0; i < 3 && !map->isFullyLoaded(); i++)
      res = frontend->render(*map);
    res = frontend->render(*map);

    auto& img = res.image;
    if (!img.data.get() || img.size.isEmpty()) continue;

    const size_t n = (size_t)img.size.width * img.size.height;
    std::vector<uint8_t> rgba(n * 4);
    for (size_t i = 0; i < n; i++) {
      const uint8_t* p = img.data.get() + i * 4;
      const uint8_t a = p[3];
      rgba[i*4+0] = a ? (uint8_t)(p[0] * 255 / a) : 0;
      rgba[i*4+1] = a ? (uint8_t)(p[1] * 255 / a) : 0;
      rgba[i*4+2] = a ? (uint8_t)(p[2] * 255 / a) : 0;
      rgba[i*4+3] = a;
    }
    int enc_len = 0;
    qoi_desc qd{(uint32_t)img.size.width, (uint32_t)img.size.height, 4, 0x00};  // named local, addressable
    void* enc = qoi_encode(rgba.data(), &qd, &enc_len);
    if (!enc) continue;

    MessageBuilder msg;
    auto f = msg.initEvent().initMapRenderFrame();
    f.setWidth(img.size.width);
    f.setHeight(img.size.height);
    f.setSeq(rendered_seq++);
    f.setImg(kj::ArrayPtr<const uint8_t>((const uint8_t*)enc, enc_len));
    pm.send("mapRenderFrame", msg);
    free(enc);
  }
  return 0;
}
