# Motor Control Handoff

This document summarizes the current motor-control state so another agent can continue immediately.

## Scope

- UI: `app_motor.py` (Flet)
- Driver/control: `devices/motor_controller.py`
- Hardware: Autonics PMC-2HSP
- Protocol path in use: Modbus RTU for manual/schedule run; P1 is still present for position moves

## Confirmed User Requirements

1. Communication constraints
   - Baudrate: `9600` or `19200` only
   - Parity: `N` fixed
   - RS-485 only
   - Auto-connect check at app start; status indicator green on success, red on failure

2. Motor mapping (latest required mapping)
   - `slave 2, X` -> upper stage (linear)
   - `slave 2, Y` -> lower stage (linear)
   - `slave 1, X` -> upper stage rotate
   - `slave 1, Y` -> lower stage rotate

3. Mechanical constants
   - Linear stage: 0.72 deg/pulse, 500 pulse/rev, 5 mm/rev, `PULSE_PER_MM = 100`
   - Rotation stage: currently assumed `1.8 deg/pulse` (may change later)

4. Manual UX
   - Speed selection by dropdown (not toggle)
   - Speed must be applied when dropdown changes
   - Press button => start, release => stop (hold-to-run)

## High-Risk Area (Root of current instability)

Speed register path changed multiple times:

- Version that physically moved well (observed logs):
  - used `speed_ratio` path
  - X around `0x0453` (base-0 style in logs), Y around `0x045F`

- Current direction requested by user:
  - use `drive_speed1` only
  - X: `0x0458 - 1 = 0x0457`
  - Y: `0x0464 - 1 = 0x0463`

Most regressions came from mixing these two approaches.

## Current Code Behavior (latest conversation state)

### `devices/motor_controller.py`

- `_write_register()`:
  - FC06 first, fallback to FC16 if FC06 fails
- `move_with_speed()`:
  - `select_speed(1)` then `start_continuous()`
- `set_speed_all()` / `set_speed_for_motor()`:
  - currently aligned to drive-speed-1 path
- `MOTOR_MAP`:
  - adjusted to latest slave/axis mapping request
- RS-485 setup:
  - should use `delay_before_tx=None` for Windows compatibility

### `app_motor.py`

- Manual speed dropdown exists and triggers speed apply
- Manual press/release starts/stops motor
- Graph/command timing split:
  - graph render: 1 s
  - schedule command update: 0.1 s
- There were latency fixes for quick tap-release behavior

## Known Runtime Symptoms Seen in Logs

- Commands often report OK but physical speed response may not match expectation
- Occasional Windows serial contention:
  - `COM7 Permission denied`
- RS-485 setting warning appeared on Windows when unsupported rs485 timing value was used

## Practical Next Steps for Next Agent

1. Lock one speed-address strategy and remove the other:
   - either `speed_ratio`-based or `drive_speed1`-based
2. Add immediate write-readback verification on speed apply:
   - write speed
   - read same register
   - log both values
3. Keep button path minimal:
   - press: select speed1 + start
   - release: stop
4. Ensure startup auto-connect tries `9600 -> 19200` and updates status reliably.

## Quick Validation Checklist

1. Connect status turns green only when both slaves reply.
2. Dropdown speed change prints:
   - target register
   - written value
   - readback value
3. Hold-to-run:
   - press => start command appears
   - release => stop command appears
4. Compare physical behavior for:
   - 0.05 mm/s, 0.5 mm/s, 3 mm/s

