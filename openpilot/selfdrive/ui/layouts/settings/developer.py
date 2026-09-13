from openpilot.common.params import Params
from openpilot.selfdrive.ui.widgets.ssh_key import ssh_key_item
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import toggle_item, multiple_button_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult

# Description constants
DESCRIPTIONS = {
  'enable_adb': tr_noop(
    "ADB (Android Debug Bridge) allows connecting to your device over USB or over the network. " +
    "See https://docs.comma.ai/how-to/connect-to-comma for more info."
  ),
  'ssh_key': tr_noop(
    "Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username " +
    "other than your own. A comma employee will NEVER ask you to add their GitHub username."
  ),
  'alpha_longitudinal': tr_noop(
    "<b>WARNING: openpilot longitudinal control is in alpha for this car and may disable Automatic Emergency Braking (AEB).</b><br><br>" +
    "On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. " +
    "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha. " +
    "Changing this setting will restart openpilot if the car is powered on."
  ),
  'dlna_live': tr_noop(
    "Serve the road camera view as a DLNA/UPnP live source over the car WiFi hotspot " +
    "(for the MMI WiFi media player). Starts automatically at every boot while enabled."
  ),
}


class DeveloperLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._is_release = self._params.get_bool("IsReleaseBranch")

    # Build items and keep references for callbacks/state updates
    self._adb_toggle = toggle_item(
      lambda: tr("Enable ADB"),
      description=lambda: tr(DESCRIPTIONS["enable_adb"]),
      initial_state=self._params.get_bool("AdbEnabled"),
      callback=self._on_enable_adb,
      enabled=ui_state.is_offroad,
    )

    # SSH enable toggle + SSH key management
    self._ssh_toggle = toggle_item(
      lambda: tr("Enable SSH"),
      description="",
      initial_state=self._params.get_bool("SshEnabled"),
      callback=self._on_enable_ssh,
    )
    self._ssh_keys = ssh_key_item(lambda: tr("SSH Keys"), description=lambda: tr(DESCRIPTIONS["ssh_key"]))

    self._joystick_toggle = toggle_item(
      lambda: tr("Joystick Debug Mode"),
      description="",
      initial_state=self._params.get_bool("JoystickDebugMode"),
      callback=self._on_joystick_debug_mode,
      enabled=ui_state.is_offroad,
    )

    self._long_maneuver_toggle = toggle_item(
      lambda: tr("Longitudinal Maneuver Mode"),
      description="",
      initial_state=self._params.get_bool("LongitudinalManeuverMode"),
      callback=self._on_long_maneuver_mode,
    )

    self._lat_maneuver_toggle = toggle_item(
      lambda: tr("Lateral Maneuver Mode"),
      description="",
      initial_state=self._params.get_bool("LateralManeuverMode"),
      callback=self._on_lat_maneuver_mode,
    )

    self._alpha_long_toggle = toggle_item(
      lambda: tr("openpilot Longitudinal Control (Alpha)"),
      description=lambda: tr(DESCRIPTIONS["alpha_longitudinal"]),
      initial_state=self._params.get_bool("AlphaLongitudinalEnabled"),
      callback=self._on_alpha_long_enabled,
      enabled=lambda: not ui_state.engaged,
    )

    self._separate_lat_long_toggle = toggle_item(
      lambda: tr("Separate Lateral/Longitudinal Control"),
      description="Use the ALA stalk button to toggle lateral control and the cruise buttons to toggle longitudinal control independently.",
      initial_state=self._params.get_bool("SeparateLatLongControl"),
      callback=self._on_separate_lat_long,
      enabled=lambda: not ui_state.engaged,
    )

    self._dlna_live_toggle = toggle_item(
      lambda: tr("DLNA Live Camera Service"),
      description=lambda: tr(DESCRIPTIONS["dlna_live"]),
      initial_state=self._params.get_bool("DlnaLiveEnabled"),
      callback=self._on_dlna_live,
    )

    self._ui_debug_toggle = toggle_item(
      lambda: tr("UI Debug Mode"),
      description="",
      initial_state=self._params.get_bool("ShowDebugInfo"),
      callback=self._on_enable_ui_debug,
    )
    self._on_enable_ui_debug(self._params.get_bool("ShowDebugInfo"))

    # sunnypilot onroad display toggles (sp_* ports)
    self._sp_turn_signals = toggle_item(
      lambda: tr("Large Turn Signal Icons"),
      description=lambda: tr("Show large turn signal icons at the top of the driving screen"),
      initial_state=self._params.get_bool("ShowTurnSignals"),
      callback=lambda s: self._params.put_bool("ShowTurnSignals", s, block=True),
    )
    self._sp_blindspot = toggle_item(
      lambda: tr("Blind Spot Warning Icon"),
      description=lambda: tr("Show a warning icon when a vehicle is in the blind spot while signaling"),
      initial_state=self._params.get_bool("BlindSpot"),
      callback=lambda s: self._params.put_bool("BlindSpot", s, block=True),
    )
    self._sp_torque_bar = toggle_item(
      lambda: tr("Steering Torque Arc"),
      description=lambda: tr("Show a lateral torque arc at the bottom of the driving screen"),
      initial_state=self._params.get_bool("torqueBar"),
      callback=lambda s: self._params.put_bool("torqueBar", s, block=True),
    )
    self._sp_rocket_fuel = toggle_item(
      lambda: tr("Acceleration Bar"),
      description=lambda: tr("Show actual acceleration as a bar on the left edge (green accel / red braking)"),
      initial_state=self._params.get_bool("RocketFuel"),
      callback=lambda s: self._params.put_bool("RocketFuel", s, block=True),
    )
    self._sp_rainbow = toggle_item(
      lambda: tr("Rainbow Path"),
      description=lambda: tr("Render the driving path with a rainbow gradient"),
      initial_state=self._params.get_bool("RainbowPath"),
      callback=lambda s: self._params.put_bool("RainbowPath", s, block=True),
    )
    self._sp_dev_ui = multiple_button_item(
      lambda: tr("Developer UI"),
      lambda: tr("Show developer metrics on the driving screen (distance, steering, lateral accel)"),
      buttons=[lambda: tr("Off"), lambda: tr("Bottom"), lambda: tr("Right"), lambda: tr("Both")],
      button_width=150,
      selected_index=int(self._params.get("DevUIInfo") or 0),
      callback=lambda idx: self._params.put("DevUIInfo", idx, block=True),
    )
    self._sp_chevron = multiple_button_item(
      lambda: tr("Lead Chevron Metrics"),
      lambda: tr("Show distance/speed/time-to-collision under the lead car marker"),
      buttons=[lambda: tr("Off"), lambda: tr("Distance"), lambda: tr("Speed"), lambda: tr("TTC"), lambda: tr("All")],
      button_width=130,
      selected_index=int(self._params.get("ChevronInfo") or 0),
      callback=lambda idx: self._params.put("ChevronInfo", idx, block=True),
    )

    self._scroller = Scroller([
      self._adb_toggle,
      self._ssh_toggle,
      self._ssh_keys,
      self._joystick_toggle,
      self._long_maneuver_toggle,
      self._lat_maneuver_toggle,
      self._alpha_long_toggle,
      self._separate_lat_long_toggle,
      self._dlna_live_toggle,
      self._ui_debug_toggle,
      self._sp_turn_signals,
      self._sp_blindspot,
      self._sp_torque_bar,
      self._sp_rocket_fuel,
      self._sp_rainbow,
      self._sp_dev_ui,
      self._sp_chevron,
    ], line_separator=True, spacing=0)

    # Toggles should be not available to change in onroad state
    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # Hide non-release toggles on release builds
    # TODO: we can do an onroad cycle, but alpha long toggle requires a deinit function to re-enable radar and not fault
    for item in (self._joystick_toggle, self._long_maneuver_toggle, self._lat_maneuver_toggle, self._alpha_long_toggle, self._separate_lat_long_toggle):
      item.set_visible(not self._is_release)

    # CP gating
    if ui_state.CP is not None:
      alpha_avail = ui_state.CP.alphaLongitudinalAvailable
      if not alpha_avail or self._is_release:
        self._alpha_long_toggle.set_visible(False)
        self._params.remove("AlphaLongitudinalEnabled")
      else:
        self._alpha_long_toggle.set_visible(True)

      long_man_enabled = ui_state.has_longitudinal_control and ui_state.is_offroad()
      self._long_maneuver_toggle.action_item.set_enabled(long_man_enabled)
      self._lat_maneuver_toggle.action_item.set_enabled(ui_state.is_offroad())
    else:
      self._long_maneuver_toggle.action_item.set_enabled(False)
      self._lat_maneuver_toggle.action_item.set_enabled(False)
      self._alpha_long_toggle.set_visible(False)

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for key, item in (
      ("AdbEnabled", self._adb_toggle),
      ("SshEnabled", self._ssh_toggle),
      ("JoystickDebugMode", self._joystick_toggle),
      ("LongitudinalManeuverMode", self._long_maneuver_toggle),
      ("LateralManeuverMode", self._lat_maneuver_toggle),
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      ("SeparateLatLongControl", self._separate_lat_long_toggle),
      ("DlnaLiveEnabled", self._dlna_live_toggle),
      ("ShowDebugInfo", self._ui_debug_toggle),
      ("ShowTurnSignals", self._sp_turn_signals),
      ("BlindSpot", self._sp_blindspot),
      ("torqueBar", self._sp_torque_bar),
      ("RocketFuel", self._sp_rocket_fuel),
      ("RainbowPath", self._sp_rainbow),
    ):
      item.action_item.set_state(self._params.get_bool(key))
    self._sp_dev_ui.action_item.set_selected_button(int(self._params.get("DevUIInfo") or 0))
    self._sp_chevron.action_item.set_selected_button(int(self._params.get("ChevronInfo") or 0))

  def _on_enable_ui_debug(self, state: bool):
    self._params.put_bool("ShowDebugInfo", state, block=True)
    gui_app.set_show_touches(state)
    gui_app.set_show_fps(state)

  def _on_enable_adb(self, state: bool):
    self._params.put_bool("AdbEnabled", state, block=True)

  def _on_enable_ssh(self, state: bool):
    self._params.put_bool("SshEnabled", state, block=True)

  def _on_joystick_debug_mode(self, state: bool):
    self._params.put_bool("JoystickDebugMode", state, block=True)
    self._params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.action_item.set_state(False)
    self._params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.action_item.set_state(False)

  def _on_long_maneuver_mode(self, state: bool):
    self._params.put_bool("LongitudinalManeuverMode", state, block=True)
    self._params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.action_item.set_state(False)
    self._params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.action_item.set_state(False)

  def _on_lat_maneuver_mode(self, state: bool):
    self._params.put_bool("LateralManeuverMode", state, block=True)
    self._params.put_bool("ExperimentalMode", False, block=True)
    self._params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.action_item.set_state(False)
    self._params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.action_item.set_state(False)

  def _on_alpha_long_enabled(self, state: bool):
    if state:
      def confirm_callback(result: DialogResult):
        if result == DialogResult.CONFIRM:
          self._params.put_bool("AlphaLongitudinalEnabled", True, block=True)
          self._params.put_bool("OnroadCycleRequested", True, block=True)
          self._update_toggles()
        else:
          self._alpha_long_toggle.action_item.set_state(False)

      # show confirmation dialog
      content = (f"<h1>{self._alpha_long_toggle.title}</h1><br>" +
                 f"<p>{self._alpha_long_toggle.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      self._params.put_bool("AlphaLongitudinalEnabled", False, block=True)
      self._params.put_bool("OnroadCycleRequested", True, block=True)
      self._update_toggles()

  def _on_separate_lat_long(self, state: bool):
    self._params.put_bool("SeparateLatLongControl", state, block=True)
    self._params.put_bool("OnroadCycleRequested", True, block=True)
    self._update_toggles()

  def _on_dlna_live(self, state: bool):
    self._params.put_bool("DlnaLiveEnabled", state, block=True)
