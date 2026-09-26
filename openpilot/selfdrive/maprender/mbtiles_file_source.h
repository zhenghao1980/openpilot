#pragma once
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <sqlite3.h>

// Minimal mbtiles backend for mapd-style offline vector tiles.
// Registered by name; serves tiles for URLs:
//   mbtiles://<name>/<z>/<x>/<y>.pbf
// This header keeps it header-only for scons friendliness; productionize by
// plugging into mbgl's FileSourceFactory (see maprenderd.cc ADAPT note).
class MbtilesFileSource {
public:
  static void registerSource(const std::string& name, const std::string& path) {
    std::lock_guard<std::mutex> lk(inst().mu_);
    auto& e = inst().db_[name];
    if (e) sqlite3_close(e);
    if (sqlite3_open_v2(path.c_str(), &e, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
      fprintf(stderr, "[mbtiles] open failed: %s\n", path.c_str());
    }
  }

  // returns tile bytes or empty vector; y is XYZ (this flips to TMS)
  static std::vector<uint8_t> get(const std::string& name, int z, int x, int y) {
    std::lock_guard<std::mutex> lk(inst().mu_);
    auto it = inst().db_.find(name);
    if (it == inst().db_.end() || !it->second) return {};
    const int tms_y = (1 << z) - 1 - y;
    const char* sql = "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?";
    sqlite3_stmt* st = nullptr;
    if (sqlite3_prepare_v2(it->second, sql, -1, &st, nullptr) != SQLITE_OK) return {};
    sqlite3_bind_int(st, 1, z); sqlite3_bind_int(st, 2, x); sqlite3_bind_int(st, 3, tms_y);
    std::vector<uint8_t> out;
    if (sqlite3_step(st) == SQLITE_ROW) {
      const void* blob = sqlite3_column_blob(st, 0);
      const int n = sqlite3_column_bytes(st, 0);
      if (blob && n > 0) out.assign((const uint8_t*)blob, (const uint8_t*)blob + n);
    }
    sqlite3_finalize(st);
    return out;
  }

private:
  static MbtilesFileSource& inst() { static MbtilesFileSource i; return i; }
  std::mutex mu_;
  std::map<std::string, sqlite3*> db_;
};
