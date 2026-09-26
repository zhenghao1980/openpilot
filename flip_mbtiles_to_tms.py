#!/usr/bin/env python3
"""Convert XYZ-tile mbtiles to TMS-tile mbtiles in-place (tile_row flip).

The maplibre-native MBTilesFileSource hardcodes TMS y-axis (y_tms = 2^z - 1 - y_xyz)
when querying tiles_shallow. planetiler produces XYZ schema by default. To make
planetiler output compatible with maplibre-native's MBTilesFileSource without
re-running planetiler, we patch tile_row in tiles_shallow so y is stored TMS.

SQLite quirk: tiles_shallow is `without rowid` with PRIMARY KEY (zoom, col, row).
Updating a PRIMARY KEY column requires DELETE+INSERT, and we have to preserve
tile_data_id (the FK into tiles_data) — so we back up (col, row, tile_data_id)
first, then DELETE+INSERT with the flipped row.

Use:  python3 flip_mbtiles_to_tms.py /data/mapd_render/china.mbtiles
"""
import sqlite3, sys, time

def flip_y(y, z):
    return (1 << z) - 1 - y

def main(path):
    print(f"opening {path}", flush=True)
    con = sqlite3.connect(path)
    cur = con.execute("SELECT DISTINCT zoom_level FROM tiles_shallow ORDER BY zoom_level")
    zooms = [r[0] for r in cur]
    print(f"zoom levels: {zooms}", flush=True)

    for z in zooms:
        t0 = time.time()
        rows = con.execute(
            "SELECT tile_column, tile_row, tile_data_id FROM tiles_shallow WHERE zoom_level=?",
            (z,)).fetchall()
        print(f"  z={z}: {len(rows)} tiles — flip", flush=True)
        con.execute("DELETE FROM tiles_shallow WHERE zoom_level=?", (z,))
        new_rows = [(col, flip_y(row, z), tdid) for col, row, tdid in rows]
        con.executemany(
            "INSERT INTO tiles_shallow(zoom_level, tile_column, tile_row, tile_data_id) VALUES (?,?,?,?)",
            [(z, col, new_row, tdid) for col, new_row, tdid in new_rows])
        con.commit()
        print(f"    z={z} done in {time.time()-t0:.1f}s", flush=True)

    con.execute("DELETE FROM metadata WHERE name='scheme'")
    con.execute("INSERT INTO metadata(name,value) VALUES('scheme','tms')")
    con.commit()
    con.close()
    print("done")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/data/mapd_render/china.mbtiles")