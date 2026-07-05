# Driver's Guide — STAG 12 GPS Timing System

This guide covers how to set up and use the lap timing system during testing and competition.

## Overview

The GPS node automatically measures lap times and segment (sector) times by detecting when you cross invisible gates on the track. Gates are placed using the **steering wheel button** and persist across power cycles—once you set them, they stay set even if the car is turned off and moved.

**The car must be moving at a reasonable speed for the system to work reliably.** If you're parked or moving very slowly, gates won't trigger.

## Setting up gates

### Starting the system

1. **Power on** the GPS node (it boots with the car)
2. **Wait for the first GPS fix** (30 seconds typical, faster if stationary near startup)
   - The dashboard shows "FIX: OK" when ready
   - Gates won't place until you have a fix
3. **The node is now live** and tracking your position in real time

### Creating gates: the button FSM

The steering wheel lap button supports three actions based on press duration:

| Press | Duration | Action | Effect |
|-------|----------|--------|--------|
| **Short** | Release after <1 s | Set sector gate | Place the next segment gate at your current position |
| **Long** | Hold 1–5 s | Set start/finish | Place the start/finish line, **clears all sector gates** |
| **Very long** | Hold >5 s | Clear all gates | Wipes everything (start/finish + all sectors) |

**Example setup:**
1. Position the car at the start/finish point → **long press** (1–5 s) → gates are cleared; S/F is now set at this position
2. Drive to the first sector → **short press** → gate 1 is placed
3. Drive to the second sector → **short press** → gate 2 is placed
4. And so on (up to 7 sector gates per start/finish)

**The dashboard shows "Next sector: N"** to tell you which gate number will be placed on the next short press.

### Clearing gates

- **Clear everything immediately**: very long press (>5 s) on the button
- **Clear one sector**: press the CAN command via the dashboard (if available)
- **Clear and re-set start/finish**: long press (1–5 s) on the button (this also wipes all sectors)

### Timing during a lap

Once gates are set:
- **On first crossing of start/finish**, the system starts a lap timer
- **Each sector gate crossing** records the segment time (the time since the previous gate)
- **Lap times are continuously updated** as you drive
- **The dashboard broadcasts times in real time** (20 Hz) to the in-car display

**Lap times include fractional seconds** (down to ~1 ms resolution due to the fused 104 Hz state) and are stamped in GPS time, so they're comparable across sessions and synchronized to external data loggers.

## Daily pre-event checklist

- [ ] GPS node has power (indicator LED is on)
- [ ] Dashboard can connect to the car's CAN bus
- [ ] Dashboard shows a green "FIX" indicator (GPS is locked)
- [ ] Set gates using the button (or dashboard, if available)
- [ ] Do a test lap—check that sector times are displayed
- [ ] Check that lap time appears on the in-car dash when you cross start/finish the second time

## Troubleshooting

### "FIX: not ready" on the dashboard

**Cause**: The GPS receiver hasn't locked onto satellites yet.

**Fix**: 
- Wait 30–60 seconds (longer if you just powered on)
- Make sure the car is in an open area (not under cover)
- Check that the antenna has a clear view of the sky
- Try moving the car a few metres to a different location

### Gates won't place

**Cause**: The GPS node is waiting for a fix, or you're parked/stationary.

**Fix**:
- Confirm the dashboard shows "FIX: OK"
- Move the car at a walking pace or faster
- Press the button again once moving

### Lap times don't update

**Cause**: The CAN bus connection is loose, or the node hasn't detected a crossing yet.

**Fix**:
- Check the CAN connector on the GPS node
- Verify the dashboard shows incoming position/attitude data (not red)
- Drive through a gate faster and more deliberately (gates are ~4 m wide; the car must cross the line cleanly)

### Gates "disappeared" (cleared when I didn't intend it)

**Cause**: You accidentally long-pressed the button (the "set start/finish" action).

**Fix**: 
- Re-set the gates (short press for sectors, long press for start/finish)
- The gates are stored in flash, so they'll come back after a power cycle if you re-power before manually clearing them

### Erratic lap times or timing jumps

**Cause**: Poor GPS signal, or the fusion filter is still settling.

**Fix**:
- Make sure the antenna has a clear view of the sky
- Wait a full lap after setting gates before trusting the times (the filter needs time to converge)
- If indoors or under heavy foliage, GPS will be unreliable

## On the track

1. **Establish gates** before the first timed run (practice laps)
2. **Monitor the in-car dash** for real-time sector and lap times
3. **Lap times update automatically**—no manual control needed once gates are set
4. **If you need to re-set gates**, long-press the button and re-do the setup

## Technical note: how timing works

The node:
- Runs a **sensor fusion filter** at 104 Hz that blends GPS position (20 Hz) with IMU acceleration to produce a smooth, fast position estimate
- Checks for gate crossings every 1/104 s with linear interpolation for sub-sample timing resolution
- Stamps each crossing time in **GPS time-of-week** (synchronized to the ZED-F9P's PPS clock) so times are repeatable and comparable to external data

Result: timing is accurate to ~1 ms and unaffected by GPS latency or dropouts.

---

**Have questions?** Check the [README](../README.md) or ask the team.
