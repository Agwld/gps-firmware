"""Grouped numeric/text readouts for every GPS.dbc message.

One QLabel per signal (or small group of related signals), updated in
place as frames arrive. Staleness (no update recently) is shown by
greying the label out, so a dead sub-system is visible at a glance
rather than just showing its last-known value forever.
"""

from __future__ import annotations

import time

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

from gps_dashboard.enums import (
    CARRIER_SOLUTION_NAMES,
    FIX_TYPE_NAMES,
    LAP_STATUS_RUNNING_BIT,
    MAG_CAL_STATUS_NAMES,
    SYS_EVENT_BITS,
    SYS_EVENT_FAULT_BITS,
    describe_bits,
    lookup,
)

#: A value not refreshed within this long is shown greyed-out. Chosen as
#: a few multiples of the slowest message's nominal period (1 Hz), so it
#: only fires on a genuinely stale/dead message, not normal jitter.
_STALE_AFTER_S = 2.5


class _Field:
    """One labelled value in the grid, tracking when it was last set so
    the refresh timer can grey it out if it goes stale."""

    def __init__(self, layout: QGridLayout, row: int, col: int, caption: str) -> None:
        layout.addWidget(QLabel(f"{caption}:"), row, col)
        self.label = QLabel("-")
        self.label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.label, row, col + 1)
        self.last_set_wall_time: float | None = None

    def set_text(self, text: str) -> None:
        self.label.setText(text)
        self.last_set_wall_time = time.time()
        self._set_stale(False)

    def refresh_staleness(self, now: float) -> None:
        if self.last_set_wall_time is None:
            return
        self._set_stale(now - self.last_set_wall_time > _STALE_AFTER_S)

    def _set_stale(self, stale: bool) -> None:
        self.label.setStyleSheet("color: grey;" if stale else "")


class StatusPanel(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._unknown_frame_count = 0
        self._fields: dict[str, _Field] = {}

        layout = QVBoxLayout(self)
        layout.addWidget(self._build_fix_group())
        layout.addWidget(self._build_position_group())
        layout.addWidget(self._build_attitude_group())
        layout.addWidget(self._build_lap_group())
        layout.addWidget(self._build_imu_group())
        layout.addWidget(self._build_system_group())
        layout.addStretch(1)

        self._stale_timer = QTimer(self)
        self._stale_timer.timeout.connect(self._refresh_staleness)
        self._stale_timer.start(500)

    # -- group builders ---------------------------------------------------

    def _group(self, title: str) -> tuple[QGroupBox, QGridLayout]:
        box = QGroupBox(title)
        grid = QGridLayout(box)
        return box, grid

    def _field(self, grid: QGridLayout, key: str, row: int, col: int, caption: str) -> None:
        self._fields[key] = _Field(grid, row, col, caption)

    def _build_fix_group(self) -> QGroupBox:
        box, grid = self._group("GPS Fix")
        self._field(grid, "fix_type", 0, 0, "Fix type")
        self._field(grid, "num_sv", 0, 2, "Satellites")
        self._field(grid, "carrier_soln", 1, 0, "Carrier solution")
        self._field(grid, "hacc", 1, 2, "Horiz. accuracy")
        self._field(grid, "sacc", 2, 0, "Speed accuracy")
        self._field(grid, "pdop", 2, 2, "PDOP")
        return box

    def _build_position_group(self) -> QGroupBox:
        box, grid = self._group("Position / Velocity")
        self._field(grid, "lat", 0, 0, "Latitude")
        self._field(grid, "lon", 0, 2, "Longitude")
        self._field(grid, "alt", 1, 0, "Altitude")
        self._field(grid, "speed", 1, 2, "Speed")
        self._field(grid, "course", 2, 0, "Course")
        return box

    def _build_attitude_group(self) -> QGroupBox:
        box, grid = self._group("Attitude (fused)")
        self._field(grid, "yaw", 0, 0, "Yaw")
        self._field(grid, "pitch", 0, 2, "Pitch")
        self._field(grid, "roll", 1, 0, "Roll")
        self._field(grid, "fusion_status", 1, 2, "Fusion status")
        return box

    def _build_lap_group(self) -> QGroupBox:
        box, grid = self._group("Lap timing")
        self._field(grid, "lap", 0, 0, "Lap")
        self._field(grid, "sector", 0, 2, "Sector")
        self._field(grid, "running_time", 1, 0, "Running time")
        self._field(grid, "lap_flags", 1, 2, "Status")
        self._field(grid, "last_event", 2, 0, "Last event")
        return box

    def _build_imu_group(self) -> QGroupBox:
        box, grid = self._group("IMU")
        self._field(grid, "accel", 0, 0, "Accel (x,y,z)")
        self._field(grid, "gyro", 1, 0, "Gyro (x,y,z)")
        self._field(grid, "mag", 2, 0, "Mag (x,y,z)")
        self._field(grid, "mag_cal", 2, 2, "Mag cal")
        return box

    def _build_system_group(self) -> QGroupBox:
        box, grid = self._group("System")
        self._field(grid, "uptime", 0, 0, "Uptime")
        self._field(grid, "cpu_load", 0, 2, "CPU load")
        self._field(grid, "retries", 1, 0, "Retries (GPS/IMU)")
        self._field(grid, "events", 1, 2, "Events")
        self._field(grid, "temps", 2, 0, "Temps (board/IMU/MCU)")
        self._field(grid, "link", 3, 0, "Link")
        return box

    # -- update entry points -----------------------------------------------

    def on_message(self, name: str, signals: dict, _timestamp: float) -> None:
        handler = getattr(self, f"_on_{name.lower()}", None)
        if handler is not None:
            handler(signals)

    def on_unknown_frame(self, arbitration_id: int) -> None:
        self._unknown_frame_count += 1
        self._fields["link"].set_text(
            f"{self._unknown_frame_count} unrecognised frame(s) seen "
            f"(last ID 0x{arbitration_id:03X})"
        )

    # -- per-message handlers ----------------------------------------------

    def _on_gps_position(self, s: dict) -> None:
        self._fields["lat"].set_text(f"{s['lat_deg']:.7f} deg")
        self._fields["lon"].set_text(f"{s['lon_deg']:.7f} deg")

    def _on_gps_velocity(self, s: dict) -> None:
        self._fields["fix_type"].set_text(lookup(FIX_TYPE_NAMES, s["fix_type"]))
        self._fields["num_sv"].set_text(str(s["num_sv"]))
        self._fields["alt"].set_text(f"{s['alt_m']:.1f} m")
        self._fields["speed"].set_text(
            f"{s['speed_mps']:.2f} m/s ({s['speed_mps'] * 3.6:.1f} km/h)"
        )
        self._fields["course"].set_text(f"{s['course_deg']:.1f} deg")

    def _on_gps_attitude(self, s: dict) -> None:
        self._fields["yaw"].set_text(f"{s['yaw_deg']:.2f} deg")
        self._fields["pitch"].set_text(f"{s['pitch_deg']:.2f} deg")
        self._fields["roll"].set_text(f"{s['roll_deg']:.2f} deg")
        self._fields["fusion_status"].set_text(f"0x{s['fusion_status']:02X}")

    def _on_gps_quality(self, s: dict) -> None:
        flags = s["flags"]
        carrier_soln = (flags >> 1) & 0x3
        self._fields["carrier_soln"].set_text(lookup(CARRIER_SOLUTION_NAMES, carrier_soln))
        self._fields["hacc"].set_text(f"{s['hacc_mm']:.0f} mm")
        self._fields["sacc"].set_text(f"{s['sacc_mm_s']:.1f} mm/s")
        self._fields["pdop"].set_text(f"{s['pdop']:.2f}")

    def _on_lap_status(self, s: dict) -> None:
        self._fields["lap"].set_text(str(s["lap"]))
        self._fields["sector"].set_text(str(s["sector"]))
        self._fields["running_time"].set_text(_format_ms(s["running_time_ms"]))
        running = "Running" if s["flags"] & LAP_STATUS_RUNNING_BIT else "Stopped"
        extra_bits = s["flags"] & ~LAP_STATUS_RUNNING_BIT
        text = running if extra_bits == 0 else f"{running} (0x{s['flags']:02X})"
        self._fields["lap_flags"].set_text(text)

    def _on_lap_event(self, s: dict) -> None:
        from gps_dashboard.enums import LAP_EVENT_TYPE_NAMES

        kind = lookup(LAP_EVENT_TYPE_NAMES, s["type"])
        self._fields["last_event"].set_text(
            f"{kind}: lap {s['lap']}, {_format_ms(s['time_ms'])}"
        )

    def _on_gps_imu_accel(self, s: dict) -> None:
        self._fields["accel"].set_text(
            f"{s['ax_mg'] / 1000:.3f}, {s['ay_mg'] / 1000:.3f}, "
            f"{s['az_mg'] / 1000:.3f} g"
        )

    def _on_gps_imu_gyro(self, s: dict) -> None:
        self._fields["gyro"].set_text(
            f"{s['gx_dps']:.2f}, {s['gy_dps']:.2f}, {s['gz_dps']:.2f} dps"
        )

    def _on_gps_mag(self, s: dict) -> None:
        self._fields["mag"].set_text(
            f"{s['mx_ut']:.1f}, {s['my_ut']:.1f}, {s['mz_ut']:.1f} uT"
        )
        self._fields["mag_cal"].set_text(lookup(MAG_CAL_STATUS_NAMES, s["cal_status"]))

    def _on_gps_temp(self, s: dict) -> None:
        self._fields["temps"].set_text(
            f"{s['mcp9800_temp_c']:.1f} / {s['imu_temp_c']:.1f} / "
            f"{s['mcu_temp_c']:.1f} degC"
        )

    def _on_gps_status(self, s: dict) -> None:
        hours, rem = divmod(s["uptime_s"], 3600)
        minutes, seconds = divmod(rem, 60)
        self._fields["uptime"].set_text(f"{hours:02d}:{minutes:02d}:{seconds:02d}")
        self._fields["cpu_load"].set_text(f"{s['cpu_load_pct']}%")
        self._fields["retries"].set_text(
            f"{s['gps_retry_count']} / {s['imu_retry_count']}"
        )
        events_text = describe_bits(s["fault_bits"], SYS_EVENT_BITS)
        field = self._fields["events"]
        field.set_text(events_text)
        has_fault = (s["fault_bits"] & SYS_EVENT_FAULT_BITS) != 0
        field.label.setStyleSheet("color: red; font-weight: bold;" if has_fault else "")

    # -- misc ---------------------------------------------------------------

    def _refresh_staleness(self) -> None:
        now = time.time()
        for field in self._fields.values():
            field.refresh_staleness(now)


def _format_ms(total_ms: int) -> str:
    total_s, ms = divmod(total_ms, 1000)
    minutes, seconds = divmod(total_s, 60)
    return f"{minutes:02d}:{seconds:02d}.{ms:03d}"
