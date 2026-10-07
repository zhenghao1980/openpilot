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
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cxxabi.h>
#include <execinfo.h>
#include <cstdio>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
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
  // 直接替换占位符子串本身（不依赖 JSON 序列化的空格/引号格式，
  // 样式文件被任意工具重写后依然有效）
  std::string out = in;
  for (const std::string& alias : {"{openmaptiles}", "openmaptiles", "{osm}", "osm"}) {
    const std::string from = "mbtiles://" + alias;
    const std::string to = "mbtiles://" + mbtiles_abs;
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

  // unique_ptr: msgq socket 有重建竞态（后连者 unlink 重建，先连者落在被删
  // 的旧 socket 上收不到消息），长时间收不到相机更新时重建 SubMaster 自愈
  auto sm = std::make_unique<SubMaster>(std::vector<const char*>{"mapRenderCam"});
  auto last_cam_tp = std::chrono::steady_clock::now();
  bool ever_got_cam = false;
  PubMaster pm({"mapRenderFrame"});
  printf("[maprenderd] ready (style=%s mbtiles=%s)\n", style_path, mbtiles_path);

  // ---- 编码/发送工作线程：渲染线程只出图，QOI 编码 + msgq 发送在副线程 ----
  // 渲染 12ms + 编码 10ms 串行时流水线余量很小；分离后渲染线程立刻处理
  // 下一相机更新，编码耗时不再占用渲染节拍（积压时丢旧帧只保最新）。
  struct EncJob {
    std::unique_ptr<uint8_t[]> data;
    unsigned w, h;
    uint64_t seq;
  };
  std::mutex enc_mtx;
  std::condition_variable enc_cv;
  std::deque<EncJob> enc_q;
  std::atomic<bool> enc_quit{false};
  std::thread enc_thread([&] {
    while (true) {
      EncJob job;
      {
        std::unique_lock<std::mutex> lk(enc_mtx);
        enc_cv.wait(lk, [&] { return enc_quit.load() || !enc_q.empty(); });
        if (enc_quit.load()) return;
        job = std::move(enc_q.front());
        enc_q.pop_front();
      }
      int enc_len = 0;
      qoi_desc qd;
      qd.width = job.w;
      qd.height = job.h;
      qd.channels = 4;
      qd.colorspace = 0x00;
      void* enc = qoi_encode(job.data.get(), &qd, &enc_len);
      if (!enc) {
        fprintf(stderr, "[mr] qoi_encode FAILED size=%ux%u\n", job.w, job.h);
        continue;
      }
      MessageBuilder msg;
      auto f = msg.initEvent().initMapRenderFrame();
      f.setWidth(job.w);
      f.setHeight(job.h);
      f.setSeq(job.seq);
      f.setImg(kj::ArrayPtr<const uint8_t>((const uint8_t*)enc, enc_len));
      pm.send("mapRenderFrame", msg);
      free(enc);
    }
  });
  enc_thread.detach();

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

    sm->update(0);  // non-blocking
    // 断线自愈：曾收到过相机更新但停滞 >2s → 重建 SubMaster 重挂 socket
    if (sm->updated("mapRenderCam")) {
      ever_got_cam = true;
      last_cam_tp = std::chrono::steady_clock::now();
    } else if (ever_got_cam &&
               std::chrono::steady_clock::now() - last_cam_tp > std::chrono::seconds(2)) {
      sm = std::make_unique<SubMaster>(std::vector<const char*>{"mapRenderCam"});
      last_cam_tp = std::chrono::steady_clock::now();
      return;
    }
    if (sm->updated("mapRenderCam")) {
      auto c = sm->operator[]("mapRenderCam").getMapRenderCam();
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

      // Dynamic render resolution: honor the size requested by the camera.
      // 等比缩放到长边 <= kMaxDim（保持请求宽高比，UI 侧按 cover 裁剪显示
      // 才不会有拉伸/裁错）；C3X 实机测量 2048 全分辨率单帧 render+encode
      // ~90ms CPU 无法维持 20fps，1280 是精细度与帧率的平衡点。
      {
        uint32_t reqW = c.getWidth() ? c.getWidth() : W;
        uint32_t reqH = c.getHeight() ? c.getHeight() : H;
        const uint32_t kMaxDim = 1280;
        if (std::max(reqW, reqH) > kMaxDim) {
          double s = (double)kMaxDim / std::max(reqW, reqH);
          reqW = (uint32_t)(reqW * s);
          reqH = (uint32_t)(reqH * s);
        }
        if (reqW < 64) reqW = 64;
        if (reqH < 64) reqH = 64;
        mln::Size cur = frontend.getSize();
        if (cur.width != reqW || cur.height != reqH) {
          frontend.setSize({reqW, reqH});
          map.setSize({reqW, reqH});
          fprintf(stderr, "[mr] resized to %ux%u\n", reqW, reqH);
        }
      }

      map.jumpTo(mln::CameraOptions()
                     .withCenter(mln::LatLng(c.getLat(), c.getLon()))
                     .withZoom(c.getZoom())
                     .withBearing(c.getBearing())
                     .withPitch(c.getPitch()));

      mln::HeadlessFrontend::RenderResult res;
      try {
        for (int i = 0; i < 3 && !map.isFullyLoaded(); i++) {
          res = frontend.render(map);
        }
        res = frontend.render(map);
      } catch (const std::exception& e) {
        fprintf(stderr, "[mr] render threw typeid=%s what=[%s]\n",
                typeid(e).name(), e.what());
        void* bt[32];
        int nbt = backtrace(bt, 32);
        char** syms = backtrace_symbols(bt, nbt);
        for (int i = 0; i < nbt && syms; i++) {
          std::string line(syms[i]);
          auto lp = line.find(" ("), plus = line.find("+", lp == std::string::npos ? 0 : lp);
          if (lp != std::string::npos && plus != std::string::npos) {
            std::string mangled = line.substr(lp + 2, plus - lp - 2);
            int st = 0;
            char* dem = abi::__cxa_demangle(mangled.c_str(), nullptr, nullptr, &st);
            if (st == 0 && dem) {
              fprintf(stderr, "[bt] %s\n", dem);
              free(dem);
              continue;
            }
          }
          fprintf(stderr, "[bt] %s\n", line.c_str());
        }
        free(syms);
        return;
      }

      auto& img = res.image;
      if (!img.data.get() || img.size.isEmpty()) {
        fprintf(stderr, "[mr] skip frame: data=%p size.isEmpty=%d\n",
                (void*)img.data.get(), (int)img.size.isEmpty());
        return;
      }

      // 地图底图完全不透明：预乘 alpha == 直通 alpha，渲染缓冲可直接编码，
      // 省掉逐像素反预乘转换 + 8MB 拷贝（全分辨率下约 48ms CPU/帧）
      const size_t n = (size_t)img.size.width * img.size.height;
      uint8_t amin = 255;
      for (size_t i = 0; i < n; i++) {
        const uint8_t a = img.data.get()[i * 4 + 3];
        if (a < amin) amin = a;
      }
      static int alpha_warned = 0;
      if (amin < 250 && ++alpha_warned <= 10)
        fprintf(stderr, "[mr] WARN: non-opaque pixel alpha min=%u\n", amin);

      // 交予编码线程：只搬动 unique_ptr，零拷贝
      {
        std::lock_guard<std::mutex> lk(enc_mtx);
        if (enc_q.size() >= 2) enc_q.pop_front();  // 积压时丢旧帧，只保最新
        enc_q.push_back(EncJob{std::move(img.data), (unsigned)img.size.width,
                               (unsigned)img.size.height, rendered_seq++});
      }
      enc_cv.notify_one();
    }
  };

  // 16ms tick (60Hz cap, mirrors vendor/glfw/glfw_view.cpp tickDuration)
  frameTick.start(mln::Duration::zero(), mln::Milliseconds(16), callback);
  (void)stop_flag;  // reserved for future shutdown signal handler
  runLoop.run();
  return 0;
}
