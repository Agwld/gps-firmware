"""Names for the enumerations, flag bits and commands the firmware defines.

Each table is copied from the firmware header named beside it. If a bit layout changes there,
it has to change here too.
"""

from __future__ import annotations

# SUFST/Inc/canbus/can_defs.h: GPS_Velocity.fix_type (the UBX-NAV-PVT fixType)
FIX_TYPES = {0: "No fix", 1: "Dead reckoning", 2: "2D", 3: "3D", 4: "GNSS + DR", 5: "Time only"}

# can_defs.h: CAN_GPS_QUALITY_* (GPS_Quality.flags)
QUALITY_FIX_OK = 0x01
QUALITY_DIFF_SOLN = 0x08
CARRIER_SOLUTIONS = {0: "None", 1: "Float", 2: "Fixed"}


def carrier_solution(flags: int) -> int:
    return (flags >> 1) & 0x3


# can_defs.h: GPS_Mag.cal_status
MAG_CAL = {0: "Uncalibrated", 1: "Collecting", 2: "Calibrated", 3: "Validated"}

# can_defs.h: CAN_LAP_EVENT_* (Lap_Event.type)
EVENT_LAP, EVENT_SECTOR, EVENT_MARK = 0, 1, 2
EVENT_TYPES = {EVENT_LAP: "Lap", EVENT_SECTOR: "Sector", EVENT_MARK: "Time mark"}

# can_defs.h: CAN_LAP_FLAG_* (Lap_Status.flags)
LAP_RUNNING, LAP_ARMED, LAP_DEGRADED = 0x01, 0x02, 0x04
LAP_FLAGS = [(0, "Running", "A lap is being timed"),
             (1, "Armed", "The start/finish gate is placed"),
             (2, "Degraded", "No recent GPS fix or no GPS time lock, so crossings are not timed reliably")]

# can_defs.h: CAN_FUSION_STATUS_* (GPS_Attitude.fusion_status)
FUSION_GPS_AIDED = 0x10
FUSION_BITS = [(0, "IMU", "Accelerometer and gyroscope samples are flowing"),
               (1, "Magnetometer", "The magnetometer is responding"),
               (2, "Mag calibrated", "Yaw is a usable heading"),
               (3, "Origin set", "The local east/north frame is anchored"),
               (4, "GPS aided", "A valid fix in the last second; otherwise dead reckoning"),
               (5, "Time locked", "Node time is locked to GPS time by the PPS signal")]

# SUFST/Inc/sys/app.h: SYS_EVT_* (GPS_Status.fault_bits, which carries ready and fault bits)
SYSTEM_BITS = [(0, "GPS ready", "The receiver has a valid fix"),
               (1, "IMU ready", ""),
               (2, "Mag ready", ""),
               (3, "Time locked", "Locked to GPS time by the PPS signal"),
               (4, "Origin set", "The local east/north frame is anchored"),
               (5, "GPS fault", "No position messages from the receiver"),
               (6, "IMU fault", ""),
               (7, "Mag fault", "")]
SYSTEM_FAULT_MASK = 0xE0

# can_defs.h: CAN_GPS_TIME_FLAG_* (GPS_Time.time_flags)
TIME_UTC_VALID = 0x01
TIME_BITS = [(0, "UTC valid", ""), (1, "Fully resolved", "No sub-second ambiguity")]

QUALITY_BITS = [(0, "Fix OK", "The receiver's gnssFixOK flag"), (3, "Differential", "Corrections are applied")]

# can_defs.h: CAN_GPS_AIR_FLAG_* (GPS_Airspeed.air_flags)
AIR_BITS = [(0, "Pitot data", "Frames are arriving from the pressure scanner"),
            (1, "Tared", "Zeroed at a standstill"),
            (2, "Wind converged", "The wind and pitot scale estimates have settled"),
            (3, "Measured density", "Air density from the sensor temperature, otherwise 20 °C is assumed")]

# can_defs.h: GPS_Gate.gate_flags
GATE_VALID = 0x01
GATE_COUNT = 8  # Slot 0 is start/finish, 1 to 7 are sectors

# can_defs.h: CAN_CMD_* (GPS_Command.cmd) and the special gate slots
CMD_GATE_SET = 0x01
CMD_GATE_CLEAR = 0x02
CMD_MAG_CAL_START = 0x10
CMD_CONFIG_SAVE = 0x20
CMD_NMEA_CFG = 0x30
SLOT_NEXT_SECTOR = 0xFE
SLOT_ALL = 0xFF


def gate_name(index: int) -> str:
    return "Start/finish" if index == 0 else f"Sector {index}"


def lap_time(ms: float | None) -> str:
    """1:23.456, or 23.456 under a minute."""
    if ms is None:
        return "—"
    minutes, rest = divmod(int(ms), 60000)
    seconds = rest / 1000
    return f"{minutes}:{seconds:06.3f}" if minutes else f"{seconds:.3f}"
