cmake_minimum_required(VERSION 3.25)
project(maprenderd C CXX)

# 与 maplibre-native 对齐：C++20（引擎硬性要求）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# vendored maplibre-native
#   git submodule add https://github.com/maplibre/maplibre-native vendor/maplibre-native
set(MAPLIBRE_DIR "" CACHE PATH "maplibre-native source dir")
if(NOT MAPLIBRE_DIR)
  message(FATAL_ERROR "Set -DMAPLIBRE_DIR=<path to maplibre-native>")
endif()

# 注释保留设备/通用两种 preset；WSL/PC 当前用 GLX preset（避免 EGL+DRI3 segfault）
#   设备(AGNOS Linux + GLES): -DMLN_WITH_OPENGL=ON -DMLN_WITH_EGL=ON
#   WSL (Plan B 试 GLX):       -DMLN_WITH_OPENGL=ON -DMLN_WITH_EGL=OFF
#   通用:                     -DMLN_WITH_GLFW=OFF -DBUILD_TESTING=OFF
set(MLN_WITH_OPENGL ON CACHE BOOL "")
set(MLN_WITH_EGL    OFF CACHE BOOL "")  # WSL/PC: 关闭 EGL,走 GLX via XWayland + llvmpipe
set(MLN_WITH_GLFW   OFF CACHE BOOL "")
set(BUILD_TESTING   OFF CACHE BOOL "")

add_subdirectory(${MAPLIBRE_DIR} ${CMAKE_BINARY_DIR}/maplibre EXCLUDE_FROM_ALL)

add_executable(maprenderd maprenderd.cc qoi.c qoi.h)

target_include_directories(maprenderd PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/../..                    # openpilot/, root for "cereal/" + "openpilot/" prefixes
  ${CMAKE_SOURCE_DIR}/../../cereal/gen/cpp     # car.capnp.h (capnp generated headers)
  ${CMAKE_SOURCE_DIR}/../../../                # repo root, msgq_repo/
  ${CMAKE_SOURCE_DIR}/../../../msgq_repo         # msgq/ipc.h (repo root + msgq_repo/)
  ${MAPLIBRE_DIR}/include
  ${MAPLIBRE_DIR}/platform/default/include   # mln/gfx/headless_frontend.hpp
  ${MAPLIBRE_DIR}/src                          # mln/storage/mbtiles_file_source.hpp (internal)
  ${CMAKE_BINARY_DIR}/maplibre/include         # generated headers
)

# cereal/messaging/messaging.h transitively pulls in capnp generated headers
# and uses libsocketmaster.a + libmsgq.a. Same link line as the rest of openpilot
# (capnp + capnp-rpc + kj + kj-async + libsocketmaster + libmsgq).
target_link_libraries(maprenderd PRIVATE
  mbgl-core
  /home/zheng/openpilot/.venv/lib/libcapnp.a
  /home/zheng/openpilot/.venv/lib/libcapnp-rpc.a
  /home/zheng/openpilot/.venv/lib/libkj.a
  /home/zheng/openpilot/.venv/lib/libkj-async.a
  /home/zheng/openpilot/openpilot/cereal/libsocketmaster.a
  /home/zheng/openpilot/msgq_repo/libmsgq.a
)
