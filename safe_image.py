"""Monkey-patch openpilot.system.ui.lib.application._load_image_from_path
to return a 1x1 placeholder on failure instead of raising.

This is the PC tizi UI test wrapper — replaces missing LFS images
(button_home.png etc.) with 1x1 transparent pixel so UI can render.

Usage: imported by run_ui_tizi_local.py BEFORE manager.main().
"""
import openpilot.system.ui.lib.application as app_mod
import pyray as rl

_orig_load = app_mod.gui_app._load_image_from_path
def _safe_load(fspath, width, height, alpha_premultiply, keep_aspect_ratio, flip_x):
    try:
        img = _orig_load(fspath, width, height, alpha_premultiply, keep_aspect_ratio, flip_x)
        if img is None or img.width == 0 or img.height == 0:
            raise ValueError("zero-size image")
        return img
    except Exception as e:
        # Fall back to a 1x1 transparent placeholder
        return rl.Image()

app_mod.gui_app._load_image_from_path = _safe_load

# Also patch the public texture() method in case it bypasses _load_image_from_path
_orig_texture = app_mod.gui_app.texture
def _safe_texture(self, *args, **kwargs):
    try:
        return _orig_texture(*args, **kwargs)
    except ZeroDivisionError:
        return rl.Texture()

import types
app_mod.gui_app.texture = types.MethodType(_safe_texture, app_mod.gui_app)

print("[safe_image] monkey-patch loaded: missing LFS images will use 1x1 placeholders", flush=True)