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
#include <atomic>
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
#include <mln/storage/file_source_manager.hpp>  // for FileSourceManager::registerFileSourceFactory
#include <mln/storage/file_source.hpp>
#include <mln/style/style.hpp>          // for Style::loadJSON()
#include <mln/util/image.hpp>
#include <mln/util/logging.hpp>
#include <mln/util/run_loop.hpp>
#include <mln/util/timer.hpp>

#include "cereal/messaging/messaging.h"

#include "qoi.h"                      // vendored single-file encoder (public domain)
// Built-in MBTilesFileSource from vendor core: register via FileSourceManager
#include <mln/storage/mbtiles_file_source.hpp>

namespace {

std::string loadFile(const char* path) {
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// style.json surgery: rewrite the openmaptiles vector source URL from
// `mbtiles://{openmaptiles}` (or `mbtiles://china` legacy) to
// `mbtiles://<abs path>` so vendor MBTilesFileSource reads china.mbtiles
// directly. glyphs URL is left untouched (style_no_text.json has none).
std::string localizeStyle(const std::string& in, const std::string& mbtiles_abs) {
  std::string out = in;
  for (const std::string& alias : {"{openmaptiles}", "openmaptiles", "{osm}", "osm"}) {
    const std::string from = "\"url\": \"mbtiles://" + alias + "\"";
    const std::string to = "\"url\": \"mbtiles://" + mbtiles_abs + "\"";
    for (auto pos = out.find(from); pos != std::string::npos; pos = out.find(from))
      out.replace(pos, from.size(), to);
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  const char* style_path = argc > 1 ? argv[1] : "/data/mapd_render/style_no_text.json";
  const char* mbtiles_path = argc > 2 ? argv[2] : "/data/mapd_render/china.mbtiles";
  const char* asset_root = "/data/mapd_render";

  mln::Log::setObserver(std::make_unique<mln::Log::NullObserver>());
  setvbuf(stdout, nullptr, _IOLBF, 0);
  setvbuf(stderr, nullptr, _IOLBF, 0);

  const float pixelRatio = 1.0f;
  const uint32_t W = 960, H = 960;

  // Register vendor built-in MBTilesFileSource factory so style sources
  // using mbtiles://<abs path> are read from the local china.mbtiles
  // (mirrors what we did for the vendor mbgl-render tool earlier).
  mln::FileSourceManager::get()->registerFileSourceFactory(
      mln::FileSourceType::Mbtiles,
      [](const mln::ResourceOptions& ro, const mln::ClientOptions& co) {
        return std::make_unique<mln::MBTilesFileSource>(ro, co);
      });

  mln::HeadlessFrontend frontend(mln::Size{W, H}, pixelRatio);
  fprintf(stderr, "[mr] HEADLESS_FRONTEND constructed size=%ux%u pixelRatio=%.1f\n", W, H, pixelRatio);
  mln::MapObserver observer;  // 默认空实现（渲染走同步 render()，不需要回调）

  mln::MapOptions mapOptions;
  mapOptions.withMapMode(mln::MapMode::Static)
      .withSize(frontend.getSize())
      .withPixelRatio(pixelRatio);

  mln::ResourceOptions resourceOptions;
  resourceOptions.withCachePath(std::string(asset_root) + "/cache.db")
      .withAssetPath(asset_root);

  mln::Map map(frontend, observer, mapOptions, resourceOptions);
  fprintf(stderr, "[mr] MAP constructed mapmode=Static observer=%s\n",
          typeid(observer).name());

  const std::string style_raw = loadFile(style_path);
  auto style = localizeStyle(style_raw, mbtiles_path);
  fprintf(stderr, "[mr] style localized: in=%zuB out=%zuB mbtiles=%s\n",
          style_raw.size(), style.size(), mbtiles_path);
  map.getStyle().loadJSON(style);
  fprintf(stderr, "[mr] style loaded OK size=%zuB\n", style.size());

  SubMaster sm({"mapRenderCam"});
  PubMaster pm({"mapRenderFrame"});
  printf("[maprenderd] ready (style=%s mbtiles=%s)\n", style_path, mbtiles_path);

  // RunLoop-driven render loop (mirrors vendor/glfw/glfw_view.cpp::run()):
  //   - stack RunLoop on main thread
  //   - 16ms timer tick -> callback that drains sm.update + jumpTo + render
  //   - runLoop.run() blocks main thread; callback fires on RunLoop thread
  //   - inside callback, RunLoop::Get() returns the current RunLoop, so
  //     HeadlessFrontend::render(Map&) can spin RunLoop::runOnce() safely.
  mln::util::RunLoop runLoop(mln::util::RunLoop::Type::Default);
  uint64_t rendered_seq = 0;
  bool stop_flag = false;
  mln::util::Timer frameTick;

  auto callback = [&] {
    // Reentrancy guard: frontend.render() spins RunLoop::runOnce() while the
    // 16ms timer keeps firing; on slow first renders (tile load + shader
    // compile) the tick re-enters this callback and MapLibre is not
    // reentrant. Skip the tick when a previous invocation is still inside.
    static std::atomic_flag in_callback = ATOMIC_FLAG_INIT;
    if (in_callback.test_and_set()) return;
    struct ClearFlag {
      std::atomic_flag& f;
      ~ClearFlag() { f.clear(); }
    } clear_flag{in_callback};

    sm.update(0);  // non-blocking
    if (sm.updated("mapRenderCam")) {
      auto c = sm["mapRenderCam"].getMapRenderCam();
      static int dbg_cam = 0;
      if (++dbg_cam <= 5 || dbg_cam % 100 == 0)
        fprintf(stderr, "[dbg] cam#%d (%.5f,%.5f) z%.2f b%.0f req=%ux%u\n",
                dbg_cam, c.getLat(), c.getLon(), c.getZoom(), c.getBearing(),
                c.getWidth(), c.getHeight());

      // Skip invalid (0,0) cams from early UI bootstrap — calling renderStill
      // with a bad camera sets stillImageRequest which then blocks all
      // subsequent renders with "Map is currently rendering an image".
      if (c.getLat() == 0.0 && c.getLon() == 0.0) {
        return;
      }

      map.jumpTo(mln::CameraOptions()
                     .withCenter(mln::LatLng(c.getLat(), c.getLon()))
                     .withZoom(c.getZoom())
                     .withBearing(c.getBearing())
                     .withPitch(c.getPitch()));

      static int dbg_render = 0;
      int render_id = ++dbg_render;
      fprintf(stderr, "[mr] render#%d starting fully=%d\n",
              render_id, (int)map.isFullyLoaded());
      mln::HeadlessFrontend::RenderResult res;
      try {
        for (int i = 0; i < 3 && !map.isFullyLoaded(); i++) {
          fprintf(stderr, "[mr] render#%d warmup pass %d\n", render_id, i);
          res = frontend.render(map);
        }
        res = frontend.render(map);
        fprintf(stderr, "[mr] render#%d returned img.data=%p size=%ux%u valid=%d\n",
                render_id, (void*)res.image.data.get(),
                res.image.size.width, res.image.size.height,
                (int)res.image.valid());
      } catch (const std::exception& e) {
        fprintf(stderr, "[mr] render#%d threw typeid=%s what=[%s]\n",
                render_id, typeid(e).name(), e.what());
        return;
      }

      auto& img = res.image;
      if (!img.data.get() || img.size.isEmpty()) {
        fprintf(stderr, "[mr] render#%d skip frame: data=%p size.isEmpty=%d\n",
                render_id, (void*)img.data.get(), (int)img.size.isEmpty());
        return;
      }

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
      qoi_desc qd;
      qd.width = (unsigned int)img.size.width;
      qd.height = (unsigned int)img.size.height;
      qd.channels = 4;
      qd.colorspace = 0x00;
      void* enc = qoi_encode(rgba.data(), &qd, &enc_len);
      if (!enc) {
        fprintf(stderr, "[mr] render#%d qoi_encode FAILED size=%ux%u rgba_b=%p\n",
                render_id, (unsigned)img.size.width, (unsigned)img.size.height,
                (void*)rgba.data());
        return;
      }
      fprintf(stderr, "[mr] render#%d qoi_encode OK size=%ux%u qoi=%dB\n",
              render_id, (unsigned)img.size.width, (unsigned)img.size.height, enc_len);

      MessageBuilder msg;
      auto f = msg.initEvent().initMapRenderFrame();
      f.setWidth(img.size.width);
      f.setHeight(img.size.height);
      f.setSeq(rendered_seq++);
      f.setImg(kj::ArrayPtr<const uint8_t>((const uint8_t*)enc, enc_len));
      pm.send("mapRenderFrame", msg);
      fprintf(stderr, "[mr] render#%d SENT seq=%llu qoi=%dB\n",
              render_id, (unsigned long long)(rendered_seq - 1), enc_len);
      free(enc);
    }
  };

  // 16ms tick (60Hz cap, mirrors vendor/glfw/glfw_view.cpp tickDuration)
  frameTick.start(mln::Duration::zero(), mln::Milliseconds(16), callback);
  (void)stop_flag;  // reserved for future shutdown signal handler
  runLoop.run();
  return 0;
}
