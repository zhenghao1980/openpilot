cmake_minimum_required(VERSION 3.16)
project(maprenderd CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Vendored maplibre-native: add as submodule and point MAPLIBRE_DIR at it.
# Build example:
#   git submodule add https://github.com/maplibre/maplibre-native vendor/maplibre-native
#   cmake -B build -DMAPLIBRE_DIR=$PWD/vendor/maplibre-native #         -DCMAKE_TOOLCHAIN_FILE=<comma aarch64 toolchain>
#   cmake --build build -j$(nproc)
set(MAPLIBRE_DIR "" CACHE PATH "maplibre-native source dir")
if(NOT MAPLIBRE_DIR)
  message(FATAL_ERROR "Set -DMAPLIBRE_DIR=<path to maplibre-native>")
endif()

# maplibre-native exposes its core as target 'mbgl-core' (platform default).
add_subdirectory(${MAPLIBRE_DIR} ${CMAKE_BINARY_DIR}/maplibre EXCLUDE_FROM_ALL)

find_package(SQLite3 REQUIRED)

add_executable(maprenderd
  maprenderd.cc
  qoi.h
  mbtiles_file_source.h
)
target_include_directories(maprenderd PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${MAPLIBRE_DIR}/include
  ${MAPLIBRE_DIR}/platform/default/include   # mln/gfx/headless_frontend.hpp
  ${CMAKE_BINARY_DIR}/maplibre/include         # generated headers
  ${SQLite3_INCLUDE_DIRS}
)
if(TARGET mln-core)
  set(_mln_core mln-core)
else()
  set(_mln_core mbgl-core)
endif()
if(TARGET mln-vendor)
  set(_mln_vendor mln-vendor)
else()
  set(_mln_vendor mbgl-vendor)
endif()
target_link_libraries(maprenderd PRIVATE ${_mln_core} ${_mln_vendor} ${SQLite3_LIBRARIES})
