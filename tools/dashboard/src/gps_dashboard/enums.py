"""Human-readable labels for the bit/enum fields the firmware documents
as fixed meanings (as opposed to genuinely free-form bytes).

Every table here is transcribed from a specific firmware source of
truth, cited alongside it - not guessed - so if the firmware's bit
layout ever changes, grep for the constant name to find what else needs
updating.
"""

from __future__ import annotations

#: SUFST/Inc/canbus/can_defs.h: GPS_Velocity.fix_type (== UBX-NAV-PVT
#: fixType, per the u-blox interface description).
FIX_TYPE_NAMES: dict[int, str] = {
    0: "No fix",
    1: "Dead reckoning",
    2: "2D",
    3: "3D",
    4: "GNSS + DR",
    5: "Time only",
}

#: SUFST/Inc/canbus/can_defs.h: CAN_GPS_QUALITY_CARR_SOLN_MASK bits.
CARRIER_SOLUTION_NAMES: dict[int, str] = {
    0: "None",
    1: "Float RTK",
    2: "Fixed RTK",
}

#: SUFST/Inc/sys/app.h: SYS_EVT_* bits, as packed verbatim into
#: GPS_Status.fault_bits by sys_task.c (app_get_events()) - despite the
#: CAN signal's name, this is the whole event group, ready+fault bits
#: both, not fault bits alone.
SYS_EVENT_BITS: list[tuple[int, str]] = [
    (0, "GPS ready"),
    (1, "IMU ready"),
    (2, "Mag ready"),
    (3, "Time locked"),
    (4, "Origin set"),
    (5, "GPS fault"),
    (6, "IMU fault"),
    (7, "Mag fault"),
    (8, "PVT tick"),
]
#: Bits whose presence indicates a problem, for colouring the summary.
SYS_EVENT_FAULT_BITS = (1 << 5) | (1 << 6) | (1 << 7)

#: SUFST/Inc/canbus/can_defs.h: GPS_Mag.cal_status.
MAG_CAL_STATUS_NAMES: dict[int, str] = {
    0: "Uncalibrated",
    1: "Collecting",
    2: "Calibrated",
    3: "Validated (vs course)",
}

#: SUFST/Inc/canbus/can_defs.h: CAN_LAP_EVENT_* (Lap_Event.type).
LAP_EVENT_TYPE_NAMES: dict[int, str] = {
    0: "Lap complete",
    1: "Sector crossed",
    2: "Time mark (button/EXTINT)",
}

#: SUFST/Src/imu/imu_task.c: only bit0 (running) is currently defined
#: for Lap_Status.flags; anything else is shown as raw hex.
LAP_STATUS_RUNNING_BIT = 0x01


def describe_bits(value: int, names: list[tuple[int, str]]) -> str:
    """Render a bitmask as a comma-joined list of set-bit names, e.g.
    "GPS ready, IMU ready" - or "(none)" if nothing is set."""
    set_names = [name for bit, name in names if value & (1 << bit)]
    return ", ".join(set_names) if set_names else "(none)"


def lookup(names: dict[int, str], value: int) -> str:
    """`names[value]`, falling back to a labelled raw value instead of
    raising for anything the table doesn't cover."""
    return names.get(value, f"Unknown ({value})")
