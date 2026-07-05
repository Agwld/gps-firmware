"""The one shared unit between every CAN source (live/replay/simulate)
and the decoder: a raw, undecoded frame plus a timestamp.

Keeping this the single boundary type means live capture, log replay and
the synthetic simulator can all feed the exact same decode/ingest path -
there is no separate "replay decode" or "sim decode" code to drift out
of sync with what real hardware produces.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class RawFrame:
    """One CAN frame, before DBC decoding.

    Attributes:
        arbitration_id: 11-bit standard CAN ID.
        data: Payload bytes (0-8 for classic CAN).
        timestamp: Seconds, `time.monotonic()`-like - only meaningful
            relative to other frames from the same source, never as a
            wall-clock value.
    """

    arbitration_id: int
    data: bytes
    timestamp: float
