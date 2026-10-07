"""Map panel shared constants.

历史遗留：早期栅格瓦片方案（TileLayer 及 world_px/mpp_at_lat 等）已随
矢量渲染管线（maprenderd + sp_map_panel 投影叠加）取代而删除，仅保留
外部引用的 Web Mercator 常量。
"""
EARTH_R = 6378137.0  # Web Mercator radius
