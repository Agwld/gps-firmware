# Driver's Guide — STAG 12 GPS Timing System

How to set up and use the lap timing system during testing and competition.

## Overview

The GPS node measures lap and sector times by detecting when the car crosses
invisible gates on the track. You place the gates with the **steering-wheel
lap button**. They are saved immediately and survive power cycles — once set,
they stay set even if the car is switched off and moved.

**Placing** a gate works parked or rolling (see *Placing gates while
stationary*). **Crossing** a gate only counts when you drive through it
forwards, in the direction it was set.

## Getting started

1. **Power on** — the node boots with the car.
2. **Wait for a GPS fix** — usually under a minute with a clear sky. The
   board's user LED comes on, and the dash shows a fix, once the node has
   both a fix and a working IMU.
3. **Gates you set last time reappear** as soon as the first fix arrives.

Gates can't be placed until there is a fix — a press before then is ignored.

## The lap button

| Press | Hold time | Action |
|-------|-----------|--------|
| **Short** | under 1 s | Place a **sector** gate here — it fills the next free sector slot (up to 7) |
| **Long** | 1–5 s | Place the **start/finish** line here. This **removes all sector gates** and resets the lap count |
| **Very long** | over 5 s | **Clear all gates** |

The action happens when you let go.

**Typical setup:**
1. At the start/finish line → **long press**.
2. At each sector point, in order → **short press**.

Once all 7 sector slots are full, a further short press moves the last one.

### Placing gates while stationary

A gate is a line across the track, square to the car's direction. When you're
moving faster than walking pace the node uses your actual direction of travel.
When parked or crawling it uses the **compass heading** instead — so point the
**nose straight down the track** before pressing.

That only works once the compass is calibrated:

- It **calibrates itself** as you drive — no special manoeuvre, just a lap or
  two with plenty of different headings.
- The dash's magnetometer status goes **Uncalibrated → Collecting → Calibrated
  → Validated**. From **Calibrated** on, stationary placement is usable;
  **Validated** means it also agrees with the GPS direction of travel.
- The calibration is saved, so after the first good session it starts ready.

**If in doubt, place gates while rolling** — that never needs the compass.

## Timing

- The **first** time you cross start/finish, the lap clock starts.
- Every later crossing completes a lap and starts the next — but crossings less
  than **10 s** apart are ignored, so creeping over the line doesn't log a lap.
- Each sector gate gives a sector time (since the previous gate); the stretch
  from the last sector gate to the finish is the final sector.
- Each sector gate counts once per lap.
- Moving or clearing the start/finish line resets timing — the next crossing
  starts lap 1 again.

Times are measured in GPS time from the node's 104 Hz position estimate.

### When timing pauses

The lap status is flagged **DEGRADED** when the node has had no GPS fix for a
second or isn't yet locked to GPS time. Laps are not started or completed
until GPS time is locked (normally within seconds of the first fix), and
while the fix is lost the position is estimated from the IMU, so crossings
are less accurate. Times from a lap that spent a long stretch without a fix
should not be trusted.

## Pre-event checklist

- [ ] Node powered, user LED on once the car is outside
- [ ] Dash receiving GPS data, fix shown
- [ ] Lap status **Armed** (start/finish placed) and not DEGRADED
- [ ] Gates where you expect them on the dash map
- [ ] Test lap: sector times appear, lap time appears on the second start/finish crossing

## Troubleshooting

### No fix

- Get out from under cover; the antenna needs open sky.
- Allow a minute after power-on.
- If the board's error LED is on, the node itself has a fault — tell the
  electronics team (the dash's GPS_Status shows which).

### Gates won't place

- There is no fix yet — wait for one and press again.
- Parked with the compass still **Uncalibrated**: the gate *is* placed, but
  may point the wrong way. Drive a lap first, or place it while rolling.

### Laps don't count

- Lap status shows **DEGRADED** — wait for the fix / time lock.
- You crossed the line backwards or outside it: gates are 4 m wide (2 m either
  side of where you pressed). Drive straight through near the press point.
- The previous crossing was less than 10 s ago.

### Gates disappeared

- A long press (start/finish) also removes all sector gates; a very long press
  removes everything. Clears are saved immediately — a power cycle does not
  bring them back. Re-place them.

### Erratic times

- Poor sky view (trees, buildings) degrades the GPS.
- Right after power-on the estimate is still settling — give it a lap.

---

Questions? See the [README](../README.md) or ask the electronics team.
