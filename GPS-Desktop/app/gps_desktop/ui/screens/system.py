"""Node: the GPS node's health, every status flag it reports, and the commands it accepts."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QCheckBox, QGridLayout, QHBoxLayout, QScrollArea, QSpinBox, QVBoxLayout, QWidget

from gps_desktop import theme
from gps_desktop.gps import enums
from gps_desktop.ui.context import AppContext
from gps_desktop.ui.screens.base import Screen
from gps_desktop.ui.screens.dashboard import duration
from gps_desktop.ui.widgets import Card, FlagGrid, button, colored, label, screen_layout


class FlagCard(Card):
    """A status word shown as flags, with lines of related values underneath."""

    def __init__(self, title: str, cid: str, bits: list, fault_mask: int = 0) -> None:
        super().__init__(title)
        self.cid = cid
        self.grid = FlagGrid(bits, fault_mask)
        self.box.addWidget(self.grid)
        self.detail = label("", "muted", wrap=True)
        self.detail.setTextFormat(Qt.TextFormat.RichText)
        self.box.addWidget(self.detail)
        self.box.addStretch(1)


class SystemScreen(Screen):
    title = "Node"
    nav_label = "Node"

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
        box, self.header = screen_layout(body, "Node")

        self.cards = {
            "system": FlagCard("Node status", "GPS_Status.fault_bits", enums.SYSTEM_BITS, enums.SYSTEM_FAULT_MASK),
            "fusion": FlagCard("Sensor fusion", "GPS_Attitude.fusion_status", enums.FUSION_BITS),
            "quality": FlagCard("GNSS solution", "GPS_Quality.flags", enums.QUALITY_BITS),
            "time": FlagCard("GPS time", "GPS_Time.time_flags", enums.TIME_BITS),
            "lap": FlagCard("Lap timer", "Lap_Status.flags", enums.LAP_FLAGS, enums.LAP_DEGRADED),
            "air": FlagCard("Airspeed", "GPS_Airspeed.air_flags", enums.AIR_BITS),
        }
        grid = QGridLayout()
        grid.setSpacing(10)
        for i, card in enumerate(self.cards.values()):
            grid.addWidget(card, i // 2, i % 2)
        box.addLayout(grid)
        box.addStretch(1)

        self.panel.section("Magnetometer")
        self.mag_button = self.panel.add(button("Restart calibration", self._restart_mag_cal,
                                                tooltip="Discard the magnetometer calibration and collect it again"))
        self.panel.note("Calibration is continuous. After a restart, drive a full circle so every heading is covered.")
        self.panel.section("Configuration")
        self.save_button = self.panel.add(button("Save to flash", self._save,
                                                 tooltip="The node writes its stored settings when it is next stationary"))
        self.panel.section("NMEA output to the logger")
        rate_row = QHBoxLayout()
        rate_row.addWidget(label("Rate", "muted"))
        self.nmea_rate = QSpinBox()
        self.nmea_rate.setRange(1, 20)
        self.nmea_rate.setValue(10)
        self.nmea_rate.setSuffix(" Hz")
        rate_row.addWidget(self.nmea_rate, 1)
        holder = QWidget()
        holder.setLayout(rate_row)
        rate_row.setContentsMargins(0, 0, 0, 0)
        self.panel.add(holder)
        self.nmea_enabled = self.panel.add(QCheckBox("Send NMEA sentences"))
        self.nmea_enabled.setChecked(True)
        self.nmea_button = self.panel.add(button("Apply", self._apply_nmea))
        self.panel.note("The node does not report its NMEA settings, so these controls show what will be sent, "
                        "not what is set now.")
        self.panel.finish()
        ctx.linkChanged.connect(self._link_changed)
        self._link_changed()

    def _link_changed(self) -> None:
        for widget in (self.mag_button, self.save_button, self.nmea_button):
            widget.setEnabled(self.ctx.connected)

    def _restart_mag_cal(self) -> None:
        self.ctx.command(enums.CMD_MAG_CAL_START, done="Magnetometer calibration restarted")

    def _save(self) -> None:
        self.ctx.command(enums.CMD_CONFIG_SAVE, done="The node will save when it is next stationary")

    def _apply_nmea(self) -> None:
        on = self.nmea_enabled.isChecked()
        self.ctx.command(enums.CMD_NMEA_CFG, self.nmea_rate.value(), int(on),
                         done=f"NMEA output {'on' if on else 'off'} at {self.nmea_rate.value()} Hz")

    def refresh(self) -> None:
        data = self.ctx.data
        v = data.values.get
        for card in self.cards.values():
            value = data.values.get(card.cid)
            card.grid.show_value(None if value is None else int(value), data.is_stale(card.cid))

        def lines(card: str, rows: list[tuple[str, str, str]]) -> None:
            """Rows of (caption, signal id, text). A signal with no value yet shows a dash."""
            self.cards[card].detail.setText("<br>".join(
                f"{caption}: " + colored(text if cid in data.values else "—", self.color(cid))
                for caption, cid, text in rows))

        lines("system", [
            ("Uptime", "GPS_Status.uptime_s", duration(v("GPS_Status.uptime_s", 0))),
            ("CPU load", "GPS_Status.cpu_load_pct", f"{v('GPS_Status.cpu_load_pct', 0):.0f} %"),
            ("GPS link recoveries", "GPS_Status.gps_retry_count", f"{v('GPS_Status.gps_retry_count', 0):.0f}"),
            ("Failed IMU reads", "GPS_Status.imu_retry_count", f"{v('GPS_Status.imu_retry_count', 0):.0f}"),
            ("Temperatures", "GPS_Temp.mcu_temp_c",
             f"board {v('GPS_Temp.mcp9800_temp_c', 0):.1f} °C, IMU {v('GPS_Temp.imu_temp_c', 0):.1f} °C, "
             f"MCU {v('GPS_Temp.mcu_temp_c', 0):.0f} °C"),
        ])
        cal = int(v("GPS_Mag.cal_status", 0))
        lines("fusion", [("Magnetometer calibration", "GPS_Mag.cal_status", enums.MAG_CAL.get(cal, f"Unknown ({cal})"))])
        quality = data.flags("GPS_Quality.flags")
        fix = int(v("GPS_Velocity.fix_type", 0))
        lines("quality", [
            ("Fix", "GPS_Velocity.fix_type", enums.FIX_TYPES.get(fix, f"Unknown ({fix})")),
            ("Carrier solution", "GPS_Quality.flags", enums.CARRIER_SOLUTIONS.get(enums.carrier_solution(quality), "Unknown")),
            ("Horizontal accuracy", "GPS_Quality.hacc_mm", f"{v('GPS_Quality.hacc_mm', 0):.0f} mm"),
            ("Speed accuracy", "GPS_Quality.sacc_mm_s", f"{v('GPS_Quality.sacc_mm_s', 0):.1f} mm/s"),
            ("PDOP", "GPS_Quality.pdop", f"{v('GPS_Quality.pdop', 0):.2f}"),
        ])
        lines("time", [
            ("UTC", "GPS_Time.utc_hour", f"{int(v('GPS_Time.utc_hour', 0)):02d}:{int(v('GPS_Time.utc_min', 0)):02d}:"
                                         f"{int(v('GPS_Time.utc_sec', 0)):02d}"),
            ("GPS time of week", "GPS_Time.itow_ms", f"{v('GPS_Time.itow_ms', 0) / 1000:.0f} s"),
        ])
        lines("lap", [
            ("Laps completed", "Lap_Status.lap", f"{v('Lap_Status.lap', 0):.0f}"),
            ("Gates placed", "Lap_Status.flags", str(len(data.gates))),
        ])
        lines("air", [
            ("Airspeed", "GPS_Airspeed.airspeed_mps", f"{v('GPS_Airspeed.airspeed_mps', 0):.2f} m/s"),
            ("Wind", "GPS_Airspeed.wind_speed_mps",
             f"{v('GPS_Airspeed.wind_speed_mps', 0):.1f} m/s from {v('GPS_Airspeed.wind_from_deg', 0):.0f}°"),
            ("Pitot scale", "GPS_Airspeed.pitot_scale", f"{v('GPS_Airspeed.pitot_scale', 0):.3f}"),
        ])

        bits = data.flags("GPS_Status.fault_bits")
        faults = [name for bit, name, _tip in enums.SYSTEM_BITS if bits & (1 << bit) & enums.SYSTEM_FAULT_MASK]
        if "GPS_Status.fault_bits" not in data.values:
            self.header.summary.setText("")
        elif faults:
            self.header.summary.setText(colored(", ".join(faults), theme.FAULT))
        else:
            self.header.summary.setText(colored("No faults", theme.OK))
