"""Dashboard: everything the GPS node reports, at a glance."""

from __future__ import annotations

from PySide6.QtWidgets import QHBoxLayout, QScrollArea, QVBoxLayout, QWidget

from gps_desktop import theme
from gps_desktop.gps import enums
from gps_desktop.ui.context import AppContext
from gps_desktop.ui.screens.base import Screen
from gps_desktop.ui.widgets import Tile, button, label, screen_layout, section_label

COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


def compass(degrees: float) -> str:
    return COMPASS[round(degrees / 45) % 8]


def duration(seconds: float) -> str:
    hours, rest = divmod(int(seconds), 3600)
    return f"{hours}:{rest // 60:02d}:{rest % 60:02d}"


class DashboardScreen(Screen):
    title = "Dashboard"
    nav_label = "Dashboard"

    def __init__(self, ctx: AppContext) -> None:
        super().__init__(ctx)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        outer.addWidget(scroll)
        body = QWidget()
        scroll.setWidget(body)
        box, self.header = screen_layout(body, "Dashboard")
        self.tiles: dict[str, Tile] = {}

        def row(title: str, specs: list[tuple[str, str, str | None, bool]]) -> None:
            box.addWidget(section_label(title))
            line = QHBoxLayout()
            line.setSpacing(10)
            for key, caption, cid, badge in specs:
                self.tiles[key] = Tile(ctx, caption, cid, badge)
                line.addWidget(self.tiles[key])
            box.addLayout(line)

        row("Fix", [("fix", "Fix", "GPS_Velocity.fix_type", True),
                    ("sats", "Satellites", "GPS_Velocity.num_sv", False),
                    ("rtk", "RTK", "GPS_Quality.flags", True),
                    ("hacc", "Horizontal accuracy", "GPS_Quality.hacc_mm", False),
                    ("pdop", "PDOP", "GPS_Quality.pdop", False),
                    ("utc", "UTC time", "GPS_Time.itow_ms", False)])
        row("Position and motion", [("lat", "Latitude", "GPS_Position.lat_deg", False),
                                    ("lon", "Longitude", "GPS_Position.lon_deg", False),
                                    ("alt", "Altitude", "GPS_Velocity.alt_m", False),
                                    ("speed", "Speed", "GPS_Velocity.speed_mps", False),
                                    ("course", "Course", "GPS_Velocity.course_deg", False),
                                    ("air", "Airspeed", "GPS_Airspeed.airspeed_mps", False)])
        row("Attitude", [("yaw", "Yaw", "GPS_Attitude.yaw_deg", False),
                         ("pitch", "Pitch", "GPS_Attitude.pitch_deg", False),
                         ("roll", "Roll", "GPS_Attitude.roll_deg", False),
                         ("fusion", "Fusion", "GPS_Attitude.fusion_status", True),
                         ("magcal", "Magnetometer", "GPS_Mag.cal_status", True)])
        row("Lap timing", [("lap_now", "Current lap", "Lap_Status.running_time_ms", False),
                           ("lap", "Laps completed", "Lap_Status.lap", False),
                           ("sector", "Sector", "Lap_Status.sector", False),
                           ("lap_last", "Last lap", None, False),
                           ("lap_best", "Best lap", None, False),
                           ("lap_delta", "Last against best", None, False)])
        row("Inertial sensors", [("accel", "Acceleration (x, y, z)", "GPS_IMU_Accel.ax_mg", False),
                                 ("gyro", "Rotation rate (x, y, z)", "GPS_IMU_Gyro.gz_dps", False),
                                 ("mag", "Magnetic field (x, y, z)", "GPS_Mag.mx_ut", False)])
        row("Node", [("health", "Health", "GPS_Status.fault_bits", True),
                     ("uptime", "Uptime", "GPS_Status.uptime_s", False),
                     ("cpu", "CPU load", "GPS_Status.cpu_load_pct", False),
                     ("t_board", "Board temperature", "GPS_Temp.mcp9800_temp_c", False),
                     ("t_imu", "IMU temperature", "GPS_Temp.imu_temp_c", False),
                     ("t_mcu", "MCU temperature", "GPS_Temp.mcu_temp_c", False)])
        box.addStretch(1)

        self.panel.section("Connection")
        self.connection = self.panel.add(label("", "muted", wrap=True))
        self.panel.section("Recording")
        self.record_status = self.panel.add(label("", "muted", wrap=True))
        self.record_button = self.panel.add(button("Record to a log file…", self._toggle_recording))
        self.panel.note("A recording holds every CAN frame as it arrives, in candump format. Replay it later "
                        "from the start screen.")
        self.panel.section("Tips")
        self.panel.note("Hover any value to see which CAN signal it comes from. Click it to add it to Plots.")
        self.panel.finish()
        ctx.data.lapsChanged.connect(self._on_data)
        ctx.linkChanged.connect(self._refresh_panel)
        ctx.recordingChanged.connect(self._refresh_panel)
        self._refresh_panel()

    def _toggle_recording(self) -> None:
        from PySide6.QtWidgets import QFileDialog

        if self.ctx.recording is not None:
            self.ctx.stop_recording()
            return
        path, _ = QFileDialog.getSaveFileName(self, "Record to", "gps-session.log", "CAN logs (*.log)")
        if path:
            self.ctx.start_recording(path)

    def _refresh_panel(self) -> None:
        data = self.ctx.data
        self.connection.setText(f"Connected to {data.link_name}" if self.ctx.connected else "Not connected")
        recording = self.ctx.recording
        self.record_button.setEnabled(self.ctx.connected)
        self.record_button.setText("Stop recording" if recording else "Record to a log file…")
        self.record_status.setText(f"Recording to {recording.name}" if recording else "Not recording")

    def _show(self, key: str, cid: str, text: str, normal: str | None = None, sub: str | None = None) -> None:
        """Show a value on a tile, or a dash in the right shade when there is none."""
        missing = cid not in self.ctx.data.values
        self.tiles[key].show_value("—" if missing else text, self.color(cid, normal), "" if missing else sub)

    def _number(self, key: str, cid: str, sub: str | None = None) -> None:
        info = self.ctx.data.channels.get(cid)
        self._show(key, cid, info.format(self.ctx.data.value(cid)) if info else "—", sub=sub)

    def refresh(self) -> None:
        data = self.ctx.data
        v = data.values.get

        # Fix
        fix = int(v("GPS_Velocity.fix_type", 0))
        fix_color = theme.OK if fix in (3, 4) else theme.FAULT if fix == 0 else theme.WARN
        self._show("fix", "GPS_Velocity.fix_type", enums.FIX_TYPES.get(fix, f"Unknown ({fix})"), fix_color)
        sats = int(v("GPS_Velocity.num_sv", 0))
        self._show("sats", "GPS_Velocity.num_sv", "15 or more" if sats >= 15 else str(sats))
        quality = data.flags("GPS_Quality.flags")
        carrier = enums.carrier_solution(quality)
        self._show("rtk", "GPS_Quality.flags", enums.CARRIER_SOLUTIONS.get(carrier, "Unknown"),
                   {1: theme.WARN, 2: theme.OK}.get(carrier, theme.NEUTRAL),
                   "Corrections applied" if quality & enums.QUALITY_DIFF_SOLN else "No corrections")
        hacc = v("GPS_Quality.hacc_mm", 0.0)
        self._show("hacc", "GPS_Quality.hacc_mm", f"{hacc / 1000:.2f} m" if hacc >= 1000 else f"{hacc:.0f} mm",
                   sub=f"Speed ±{v('GPS_Quality.sacc_mm_s', 0.0) / 1000:.2f} m/s")
        self._number("pdop", "GPS_Quality.pdop")
        valid = data.flags("GPS_Time.time_flags") & enums.TIME_UTC_VALID
        self._show("utc", "GPS_Time.itow_ms",
                   f"{int(v('GPS_Time.utc_hour', 0)):02d}:{int(v('GPS_Time.utc_min', 0)):02d}:{int(v('GPS_Time.utc_sec', 0)):02d}",
                   None if valid else theme.WARN,
                   f"GPS time of week {v('GPS_Time.itow_ms', 0) / 1000:.0f} s" if valid else "Not valid yet")

        # Position and motion
        self._show("lat", "GPS_Position.lat_deg", f"{v('GPS_Position.lat_deg', 0.0):.7f}°")
        self._show("lon", "GPS_Position.lon_deg", f"{v('GPS_Position.lon_deg', 0.0):.7f}°")
        self._number("alt", "GPS_Velocity.alt_m")
        speed = v("GPS_Velocity.speed_mps", 0.0)
        self._show("speed", "GPS_Velocity.speed_mps", f"{speed * 3.6:.1f} km/h", sub=f"{speed:.2f} m/s")
        course = v("GPS_Velocity.course_deg", 0.0)
        self._show("course", "GPS_Velocity.course_deg", f"{course:.1f}°", sub=compass(course))
        air = data.flags("GPS_Airspeed.air_flags")
        wind = (f"Wind {v('GPS_Airspeed.wind_speed_mps', 0.0):.1f} m/s from {v('GPS_Airspeed.wind_from_deg', 0.0):.0f}°"
                if air & 0x04 else "Wind not estimated yet" if air & 0x01 else "No pitot data")
        self._show("air", "GPS_Airspeed.airspeed_mps", f"{v('GPS_Airspeed.airspeed_mps', 0.0) * 3.6:.1f} km/h", sub=wind)

        # Attitude
        for key, cid in (("yaw", "GPS_Attitude.yaw_deg"), ("pitch", "GPS_Attitude.pitch_deg"),
                         ("roll", "GPS_Attitude.roll_deg")):
            self._show(key, cid, f"{v(cid, 0.0):.1f}°")
        aided = data.flags("GPS_Attitude.fusion_status") & enums.FUSION_GPS_AIDED
        self._show("fusion", "GPS_Attitude.fusion_status", "GPS aided" if aided else "Dead reckoning",
                   theme.OK if aided else theme.WARN)
        cal = int(v("GPS_Mag.cal_status", 0))
        self._show("magcal", "GPS_Mag.cal_status", enums.MAG_CAL.get(cal, f"Unknown ({cal})"),
                   {0: theme.FAULT, 1: theme.WARN}.get(cal, theme.OK))

        # Lap timing
        flags = data.flags("Lap_Status.flags")
        state = "Running" if flags & enums.LAP_RUNNING else "Armed" if flags & enums.LAP_ARMED else "No start/finish gate"
        if flags & enums.LAP_DEGRADED:
            state += " · degraded"
        self._show("lap_now", "Lap_Status.running_time_ms",
                   enums.lap_time(v("Lap_Status.running_time_ms")) if flags & enums.LAP_RUNNING else "—",
                   theme.WARN if flags & enums.LAP_DEGRADED else None, state)
        self._show("lap", "Lap_Status.lap", f"{int(v('Lap_Status.lap', 0))}")
        self._show("sector", "Lap_Status.sector", f"{int(v('Lap_Status.sector', 0)) + 1}")
        last = data.last_lap()
        faint = theme.TEXT_FAINT
        self.tiles["lap_last"].show_value(enums.lap_time(last.time_ms) if last else "—", None if last else faint,
                                          f"Lap {last.number}" if last else "")
        best = data.best_lap_ms
        self.tiles["lap_best"].show_value(enums.lap_time(best), None if best else faint)
        if last is not None and best is not None:
            delta = (last.time_ms - best) / 1000
            self.tiles["lap_delta"].show_value("Best lap" if delta == 0 else f"+{delta:.3f} s",
                                               theme.OK if delta == 0 else None)
        else:
            self.tiles["lap_delta"].show_value("—", faint)

        # Inertial sensors
        self._show("accel", "GPS_IMU_Accel.ax_mg", "  ".join(
            f"{v(f'GPS_IMU_Accel.{axis}_mg', 0.0) / 1000:+.2f}" for axis in ("ax", "ay", "az")) + " g")
        self._show("gyro", "GPS_IMU_Gyro.gz_dps", "  ".join(
            f"{v(f'GPS_IMU_Gyro.{axis}_dps', 0.0):+.1f}" for axis in ("gx", "gy", "gz")) + " °/s")
        self._show("mag", "GPS_Mag.mx_ut", "  ".join(
            f"{v(f'GPS_Mag.{axis}_ut', 0.0):+.1f}" for axis in ("mx", "my", "mz")) + " µT")

        # Node
        bits = data.flags("GPS_Status.fault_bits")
        faults = [name for bit, name, _tip in enums.SYSTEM_BITS if bits & (1 << bit) & enums.SYSTEM_FAULT_MASK]
        self._show("health", "GPS_Status.fault_bits", ", ".join(faults) if faults else "OK",
                   theme.FAULT if faults else theme.OK)
        self._show("uptime", "GPS_Status.uptime_s", duration(v("GPS_Status.uptime_s", 0)))
        self._show("cpu", "GPS_Status.cpu_load_pct", f"{v('GPS_Status.cpu_load_pct', 0):.0f} %")
        for key, cid in (("t_board", "GPS_Temp.mcp9800_temp_c"), ("t_imu", "GPS_Temp.imu_temp_c"),
                         ("t_mcu", "GPS_Temp.mcu_temp_c")):
            self._show(key, cid, f"{v(cid, 0.0):.1f} °C")
