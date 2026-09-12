"""Remote big-model (chestnut) fusion status badge for the Big UI onroad HUD.

Shows whether the remote fusion big model is working and the current fusion
weight (how much of the big model is mixed into the final modelV2 output).

Data channel: params written by modeld/fusion_model.py —
  RemoteModelState        INT    0=CONNECTING 1=ACTIVE 2=WEAK 3=DOWN
  RemoteModelFusionWeight STRING float w in [0,1]
RemoteModelState absent (fusion not enabled) -> badge hidden.

C3X Big UI only; the real-chestnut-hardware ChestnutState machine in
ui_state.py is unrelated (no chestnut USB device on C3X remote setups).
"""
import math

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

REMOTE_CONNECTING = 0
REMOTE_ACTIVE = 1
REMOTE_WEAK = 2
REMOTE_DOWN = 3

ICON_W, ICON_H = 72, 54          # source assets are 144x107
CHIP_HEIGHT = 96
CHIP_PAD_H = 24
CHIP_ICON_TEXT_GAP = 12
CHIP_BUTTON_GAP = 24
PCT_FONT_SIZE = 48

GREEN = rl.Color(53, 208, 94, 255)
AMBER = rl.Color(255, 176, 58, 255)
GRAY = rl.Color(154, 154, 154, 255)
BG = rl.Color(0, 0, 0, 166)
BORDER = rl.Color(255, 255, 255, 56)


class ChestnutBadge:
  """Pill-shaped status chip rendered to the left of the ExpButton."""

  def __init__(self):
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._txt_gray = gui_app.texture('icons_mici/chestnut.png', ICON_W, ICON_H)
    self._txt_green = gui_app.texture('icons_mici/chestnut_green.png', ICON_W, ICON_H)
    self._txt_orange = gui_app.texture('icons_mici/chestnut_orange.png', ICON_W, ICON_H)

  def render_left_of(self, button_rect: rl.Rectangle) -> None:
    state = ui_state.remote_model_state
    if state < 0:
      return  # fusion mode not enabled

    w = ui_state.remote_fusion_weight
    if state == REMOTE_CONNECTING:
      text, text_color = "\u00b7\u00b7\u00b7", GRAY
      icon, icon_alpha = self._txt_gray, 0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0))
    elif state == REMOTE_ACTIVE:
      text, text_color = f"{round(w * 100)}%", GREEN
      icon, icon_alpha = self._txt_green, 1.0
    elif state == REMOTE_WEAK:
      text, text_color = f"{round(w * 100)}%", AMBER
      icon, icon_alpha = self._txt_green, 0.75
    else:  # REMOTE_DOWN
      text, text_color = "--", AMBER
      icon, icon_alpha = self._txt_orange, 1.0

    text_w = measure_text_cached(self._font_bold, text, PCT_FONT_SIZE).x
    chip_w = CHIP_PAD_H + ICON_W + CHIP_ICON_TEXT_GAP + text_w + CHIP_PAD_H
    x = button_rect.x - CHIP_BUTTON_GAP - chip_w
    y = button_rect.y + (button_rect.height - CHIP_HEIGHT) / 2

    chip = rl.Rectangle(x, y, chip_w, CHIP_HEIGHT)
    rl.draw_rectangle_rounded(chip, 0.35, 10, BG)
    rl.draw_rectangle_rounded_lines_ex(chip, 0.35, 10, 4, BORDER)

    icon_pos = rl.Vector2(x + CHIP_PAD_H, y + (CHIP_HEIGHT - ICON_H) / 2)
    rl.draw_texture_ex(icon, icon_pos, 0.0, 1.0, rl.Color(255, 255, 255, int(255 * icon_alpha)))

    text_pos = rl.Vector2(x + CHIP_PAD_H + ICON_W + CHIP_ICON_TEXT_GAP,
                          y + (CHIP_HEIGHT - PCT_FONT_SIZE) / 2 - 4)
    rl.draw_text_ex(self._font_bold, text, text_pos, PCT_FONT_SIZE, 0, text_color)
