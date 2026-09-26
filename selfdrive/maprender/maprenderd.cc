// maprenderd: standalone MapLibre GL Native offscreen renderer for the map panel.
//
// Ported to the mln-namespace API (maplibre-native main >= namespace migration).
// Architecture unchanged: this process owns its own headless GL context (zero
// conflict with raylib UI); consumes "mapRenderCam" camera updates from the UI
// and publishes QOI-encoded RGBA frames on "mapRenderFrame".
//
// Key API facts (verified against main headers 2026-09):
//   - mln::HeadlessFrontend(Size, pixelRatio, ...)  [platform/default/include]
//   - mln::HeadlessFrontend::render(Map&) -> RenderResult{PremultipliedImage,...}
//     (SYNCHRONOUS — no observer/cv needed)
//   - mln::Map(RendererFrontend&, MapObserver&, MapOptions, ResourceOptions, ...)
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <mln/gfx/headless_frontend.hpp>
#include <mln/map/camera.hpp>
#include <mln/map/map.hpp>
#include <mln/map/map_observer.hpp>
#include <mln/map/map_options.hpp>
#include <mln/storage/resource_options.hpp>
#include <mln/util/image.hpp>
#include <mln/util/logging.hpp>
#include <mln/util/run_loop.hpp>

#include "cereal/messaging/messaging.h"

#include "qoi.h"                      // vendored single-file encoder (public domain)
#include "mbtiles_file_source.h"      // sqlite backend; wire into FileSourceFactory (ADAPT)

namespace {

std::string loadFile(const char* path) {
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// style.json surgery: point vector sources at our mbtiles scheme, glyphs local.
std::string localizeStyle(const std::string& in, const std::string& name) {
  std::string out = in;
  const std::string from = "\"url\": \"mbtiles://openmaptiles\"";
  const std::string to = "\"tiles\": [\"mbtiles://" + name + "/{z}/{x}/{y}.pbf\"]";
  for (auto pos = out.find(from); pos != std::string::npos; pos = out.find(from))
    out.replace(pos, from.size(), to);
  const std::string gfrom = "\"glyphs\": \"https://fonts.openmaptiles.org/{fontstack}/{range}.pbf\"";
  const std::string gto = "\"glyphs\": \"file:///data/mapd_render/glyphs/{fontstack}/{range}.pbf\"";
  auto gp = out.find(gfrom);
  if (gp != std::string::npos) out.replace(gp, gfrom.size(), gto);
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  const char* style_path = argc > 1 ? argv[1] : "/data/mapd_render/style.json";
  const char* mbtiles_path = argc > 2 ? argv[2] : "/data/mapd_render/china.mbtiles";
  const char* asset_root = "/data/mapd_render";

  mln::Logging::setPlatformCallback([](mln::Severity sev, const std::string& msg) {
    fprintf(stderr, "[maprenderd] %d: %s\n", (int)sev, msg.c_str());
  });

  // RunLoop must outlive the map (thread pool scheduling). ADAPT: some versions
  // default-construct implicitly; keep explicit.
  mln::util::RunLoop runLoop(mln::util::RunLoop::Type::Default);
  (void)runLoop;

  const float pixelRatio = 1.0f;
  const uint32_t W = 960, H = 960;

  MbtilesFileSource::registerSource("china", mbtiles_path);   // ADAPT: 接入 FileSourceFactory

  mln::HeadlessFrontend frontend(mln::Size{W, H}, pixelRatio);
  mln::MapObserver observer;  // 默认空实现（渲染走同步 render()，不需要回调）

  mln::MapOptions mapOptions;
  mapOptions.withMapMode(mln::MapMode::Static)
      .withSize(frontend.getSize())
      .withPixelRatio(pixelRatio);

  mln::ResourceOptions resourceOptions;
  resourceOptions.withCachePath(std::string(asset_root) + "/cache.db")
      .withAssetPath(asset_root);

  mln::Map map(frontend, observer, mapOptions, resourceOptions);

  auto style = localizeStyle(loadFile(style_path), "china");
  map.getStyle().loadJSON(style);   // ADAPT: 若接口漂移，改 setStyleURL("data:application/json,...")

  SubMaster sm({"mapRenderCam"});
  PubMaster pm({"mapRenderFrame"});
  printf("[maprenderd] ready\n");

  uint64_t rendered_seq = 0;
  while (true) {
    sm.update(100);  // 100ms poll
    if (!sm.updated("mapRenderCam")) continue;

    auto c = sm["mapRenderCam"].getMapRenderCam();
    map.jumpTo(mln::CameraOptions()
                   .withCenter(mln::LatLng(c.getLat(), c.getLon()))
                   .withZoom(c.getZoom())
                   .withBearing(c.getBearing()));

    // 本地瓦片加载快；未 fully loaded 时多渲一两帧让新瓦片就位
    mln::HeadlessFrontend::RenderResult res;
    for (int i = 0; i < 3 && !map.isFullyLoaded(); i++)
      res = frontend.render(map);
    res = frontend.render(map);

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
    void* enc = qoi_encode(rgba.data(), &(qoi_desc){(int)img.size.width, (int)img.size.height, 4, 0x00}, &enc_len);
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
