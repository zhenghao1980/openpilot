# maprenderd aarch64 交叉编译（C3X / AGNOS）

在 WSL（Ubuntu 24.04 x86_64）上为 C3X 交叉编译 maprenderd 的完整方法。

## 前置（一次性）

- 交叉工具链：sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
- arm64 sysroot：~/arm64-sysroot（capnp/kj 静态库在 lib/，头文件 include/；
  系统库 deb 解包到 usr/ —— libcurl、libpng、zlib、libgbm、libdrm、GLES/EGL 等）
- cereal capnp 代码用**宿主 capnp 1.3.0** 重新生成到 /tmp/arm64-gen，
  并镜像目录结构到 /tmp/arm64-inc/openpilot/cereal/gen/cpp/
  （messaging.h 用带前缀路径引用生成头，且生成头之间尖括号互引，两个 -I 都需要）
- 静态库 /tmp/arm64-obj/lib{msgq,socketmaster,cereal_gen}.a（AArch64 对象）

## 关键坑（补丁见本目录）

1. adreno EGL 的 eglGetDisplay(0) 走坏掉的 Wayland 子驱动必崩
   → GBM 平台 + dlsym mangled eglGetPlatformDisplayEXT
2. gl 符号不能绑 glvnd libOpenGL（设备上无注册 vendor）
   → libGLESv2.so.2 必须排在 NEEDED 首位（-Wl,--no-as-needed）
3. 首次渲染可能超过 16ms，定时器重入回调导致崩溃
   → maprenderd.cc 已内置重入保护

## 构建命令

cmake -S . -B build_arm64   -DCMAKE_TOOLCHAIN_FILE=/tmp/toolchain-arm64.cmake   -DMAPLIBRE_DIR=$PWD/vendor/maplibre-native   -DMLN_WITH_EGL=ON -DMLN_WITH_X11=OFF -DCMAKE_BUILD_TYPE=Release   -DMAPRENDERD_PLATFORM_LIBS="/tmp/arm64-obj/libsocketmaster.a;/tmp/arm64-obj/libmsgq.a;/tmp/arm64-obj/libcereal_gen.a;/home/zheng/arm64-sysroot/lib/libcapnp.a;/home/zheng/arm64-sysroot/lib/libkj.a;/home/zheng/arm64-sysroot/usr/lib/aarch64-linux-gnu/libgbm.so;/home/zheng/arm64-sysroot/usr/lib/aarch64-linux-gnu/libdrm.so"   -DZLIB_LIBRARY=/home/zheng/arm64-sysroot/usr/lib/aarch64-linux-gnu/libz.so   -DZLIB_INCLUDE_DIR=/home/zheng/arm64-sysroot/usr/include   -DCURL_LIBRARY=/home/zheng/arm64-sysroot/usr/lib/aarch64-linux-gnu/libcurl.so   "-DCMAKE_EXE_LINKER_FLAGS=-Wl,--no-as-needed /home/zheng/arm64-sysroot/usr/lib/aarch64-linux-gnu/libGLESv2.so.2 -Wl,--as-needed -L/home/zheng/arm64-sysroot/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,/home/zheng/arm64-sysroot/usr/lib/aarch64-linux-gnu -Wl,--allow-shlib-undefined"   -DCMAKE_CXX_FLAGS="-I/tmp/arm64-inc -I/tmp/arm64-gen"
make -C build_arm64 maprenderd -j$(nproc)

## 部署

scp build_arm64/maprenderd 到设备 .../selfdrive/maprender/maprenderd.aarch64，
再 cp 到 maprenderd（process_config 启动 ./maprenderd）。
