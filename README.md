# Spit-electronics

## Architecture
The SpitTronics uses an Arduino Leonardo to send PWM pulses to an H-bridge, which allows for setting the voltage the motor receives. The Arduino receives encoder pulses on one of its digital pins.

The Arduino in turn is connected to a Raspberry Pi 4 via USB. Using the [SerialTransfer library](https://github.com/PowerBroker2/SerialTransfer/tree/master) on the Arduino side and the corresponding Python code ([pySerial](https://github.com/PowerBroker2/pySerialTransfer/tree/master)), data is send back and forth between the RPi and the Arduino.

The RPi uses MQTT to make the data that is received in the Python script available anywhere on the RPi. Node-red then subscribes to these topics and the data is visualized in a node-red-dashboard.

A lighter alternative dashboard ([`RPi/dashboard/`](RPi/dashboard)) runs alongside Node-Red on port `8080` — a single static HTML/JS page served by nginx, talking directly to Mosquitto over WebSockets (`:9001`). Designed for the tablet, fully offline-capable. See [`RPi/dashboard/README.md`](RPi/dashboard/README.md). `install.sh` sets it up automatically.

!!!!!!TODO: add picture!!!!!

## How the Pi and Arduino communicate
Both sides use the SerialTransfer protocol over `/dev/ttyACM0` at 115200 baud — a framed, CRC-checked packet format where you stuff a binary buffer with `txObj`/`tx_obj` and read it back with `rxObj`/`rx_obj` in the same field order. No JSON, no text — the byte layout must match exactly, in the same order, on both sides.

### Pi → Arduino (commands)
The Pi re-sends the *whole* packet on every incoming MQTT message (`sendData()` in the Python script). So if you only tweak the I-gain in Node-Red, the Arduino still gets the unchanged P, feed-forward, and start/stop values too.

`vel_ref` is also rate-limited on the Pi side before it goes out — see [Setpoint ramping](#setpoint-ramping) below.

| Offset | Field          | Type   | Pi source                | Arduino sink         |
|--------|----------------|--------|--------------------------|----------------------|
| 0      | `vel_ref`      | float  | Node-Red slider          | `vel_ref_receive`    |
| 4      | `P_gain`       | float  | Node-Red (clamped 0–5)   | `P`                  |
| 8      | `I_action`     | float  | Node-Red (clamped 0–15)  | `I_action`           |
| 12     | `feed_forward` | float  | Node-Red                 | `feed_forward`       |
| 16     | `start_stop`   | uint8  | Node-Red bool `"true"`   | `start_stop_RPi`     |
| 17     | `direction`    | uint8  | Node-Red switch (0/1)    | `direction_RPi` → `directionPin` (latched, see below) |
| 18     | `control_mode` | uint8  | Node-Red VEL/POS switch  | `control_mode_RPi` (0=velocity, 1=position) |
| 19     | `pos_ref`      | int32  | Pi (`zero_count + pos_ref_deg/360 × counts_per_rev`) | `pos_ref_counts` |
| 23     | `pos_P_gain`   | float  | Node-Red (clamped 0–10)  | `pos_P_gain`         |
| 27     | `pos_max_vel`  | float  | Node-Red (clamped 0–1000) | `pos_max_vel`       |
| 31     | `pos_max_accel`| float  | Node-Red (clamped 0.1–1000) | `pos_max_accel`    |
| 35     | `pos_I_action` | float  | Node-Red (clamped 0–10)  | `pos_I_action`       |
| 39     | `pos_D_gain`   | float  | Node-Red (clamped 0–100) | `pos_D_gain`         |

`start_stop` and `direction` are sent with `val_type_override='B'` (unsigned byte) — without that override, pySerialTransfer packs a Python `int` as 4 bytes by default, which would shift every following field by 3 bytes.

**Direction latching.** A direction flip is only applied to `directionPin` when both: (a) PWM duty `u` is effectively zero (the H-bridge isn't delivering power), and (b) no encoder pulse has fired for `DIRECTION_FLIP_QUIET_MS` (default `200` ms). The encoder check is the load-bearing one — `control_vel` can't be trusted as a "motor stopped" signal: the ISR only updates `vel` on each pulse, so when pulses stop `vel` is frozen at its last value indefinitely; and line `control_vel = 0.0` force-zeros it whenever `vel_ref < 0.1` regardless of physical motion. The pulse-silence check via `lastPulseTime` (stamped in the ISR) is the only honest "motor is at rest" signal we have. 200 ms of silence at 305 pulses/rev bounds motion below ~6°/sec — effectively stationary.

This makes oscillate mode work without a special protocol: leave `motor_start = true`, command `vel_ref` toward zero near an endpoint, then publish the direction flip — once the spit physically coasts down (a few hundred ms after PWM drops to zero), the latched direction applies and the next `vel_ref > 0` accelerates the other way.

### Arduino → Pi (telemetry)
Every 50 ms the Arduino sends:

| Offset | Field            | Type  | Meaning                                                   |
|--------|------------------|-------|-----------------------------------------------------------|
| 0      | `control_vel`    | float | Moving-average measured velocity (magnitude, always ≥ 0)  |
| 4      | `encoderSend`    | int32 | Signed encoder count — direction-aware (see below)        |
| 8      | `u_duty`         | uint8 | PWM duty 0..100 the controller is commanding ("motor effort"). Logged for cook-time diagnostics. |
| 9      | `error_integral` | float | Velocity-PID integral term — drifts upward under persistent load. |

`encoderSend` is signed: the ISR increments on forward pulses and decrements on reverse pulses, based on the latched `current_direction` set by the main loop when it writes `directionPin`. Since the encoder is single-channel, the ISR has to *infer* direction from what was last commanded — which is reliable because the direction-flip latch only allows changes when the motor is at rest (see Direction latching above).

The Pi keeps two derived quantities from this stream:
- **`num_rounds`** (published on `mqtt/rpi/num_rounds`) = `Σ |delta| / 305`. Total *distance travelled*, accumulating in both directions — so a forward+reverse oscillation still counts up. Right metric for a cook-time counter.
- Signed angular position can be recovered directly from `encoderCount % 305` once a zero is established (for future position-control work).

Velocity (`control_vel`) is unchanged in semantics — it's always a positive magnitude since the ISR's `vel = 1/dt` doesn't care about direction. The PI loop's `error = vel_ref − control_vel` still works because the slider commands a magnitude and direction is handled separately by the `directionPin`.

**Limit worth knowing**: the encoder still can't detect *backdrive*. If the motor is stopped or commanded forward but an unbalanced load gravity-rotates the spit backward, those pulses get counted in whatever the latched direction is — they'll lie by a few degrees for a roughly-balanced load, more for a heavy lopsided one.

### Spit angle (calibration)
The Pi tracks signed angle relative to a user-defined home position. Two new dashboard widgets and a derived telemetry topic:

| Topic                          | Direction | Meaning                                                                 |
|--------------------------------|-----------|-------------------------------------------------------------------------|
| `mqtt/rpi/rx/counts_per_rev`   | Pi ← UI   | Number input — encoder pulses per full rotisserie revolution. Live-tunable. |
| `mqtt/rpi/rx/set_home`         | Pi ← UI   | Button trigger — captures current `encoderCount` as the zero reference. |
| `mqtt/rpi/spit_angle_deg`      | Pi → UI   | `(encoderCount − zero_count) / counts_per_rev × 360`. Multi-turn (no wrap). |

**Calibration procedure** (one-time, after assembling or after major mechanical changes):
1. Set `counts_per_rev` to a starting guess (default `4270` = the existing `305 × 14` from `num_rounds + flows.json` divisor).
2. Press **Set Home** on the dashboard with the spit at any rest position.
3. Rotate the spit *exactly one full turn* — either by hand or by jogging the motor slowly.
4. Read the Angle display. If it reads, say, `380°` after one turn, multiply `counts_per_rev` by `380/360 = 1.056` and try again. Repeat until one physical turn reads `360°` ± a degree.
5. Note the calibrated number somewhere durable (this value isn't persisted across Pi restarts — see below).

**Persistence**: every tunable setting in the dashboard is written to `~/.spit_settings.json` when the user presses **Save Settings**, and loaded on Python script startup. Currently the persisted set is `counts_per_rev`, the velocity-PID gains (`P_gain`, `I_action`, `feed_forward`), and the position-mode tunables (`pos_P_gain`, `pos_max_vel`, `pos_max_accel`) — see `PERSISTED_VARS` in `Spit_RPi_Arduino.py`. Operational state (`control_mode`, `pos_ref_deg`, `vel_ref`, `start_stop`, `direction`) is intentionally *not* persisted — those are "where the spit is right now", not "how it's tuned". `zero_count` is also not persisted; it's tied to the live encoder count and would be meaningless after an Arduino reboot anyway.

**Caveat**: the angle inherits the encoder's backdrive blind spot — a stationary spit gravity-rotated by a lopsided load won't show up in the angle reading.

### Dashboard compasses and the draggable knob
- **Spit angle** — read-only `ui_gauge` (compass-style). Current spit orientation, fed from `mqtt/rpi/spit_angle_deg` via a function node that wraps the signed multi-turn value to 0–360°.
- **Pos target (drag)** — interactive `ui_template` widget rendering an SVG knob. Subscribes to retained `mqtt/rpi/pos_ref_deg_current` to show the current target, and publishes new targets to `mqtt/rpi/rx/pos_ref_deg` when the user clicks/drags. Supports mouse and touch. Output is throttled to once every ~100 ms or per >1° change to keep MQTT traffic sane. **Caveat**: the knob outputs a value in 0–360°, so dragging it on a multi-turn target (e.g., `720°`) snaps the target down to the wrapped value (`0°`). Use the numeric `Pos target (deg)` input for multi-turn precision.

In position mode, dragging the knob should make the spit's compass needle chase to match.

### Dashboard inputs show real values via retained MQTT
The four text inputs on the dashboard (`P gain`, `I gain`, `Feed Forward`, `Counts / rev`) used to be blank on page load because they only published edits and never subscribed to anything. Now the Pi publishes each setting's current value to a corresponding `_current` topic with `retain=True`, and each widget has an `mqtt in` wired to that topic:

| Variable        | Current-state topic               |
|-----------------|-----------------------------------|
| `P_gain`        | `mqtt/rpi/P_gain_current`         |
| `I_action`      | `mqtt/rpi/I_action_current`       |
| `feed_forward`  | `mqtt/rpi/feed_forward_current`   |
| `counts_per_rev`| `mqtt/rpi/counts_per_rev_current` |

The widgets are configured with `passthru: false` so an incoming `_current` message updates the display without re-triggering the output (no feedback loop). A side effect: when the user types a value that the Pi clamps (e.g., a negative `counts_per_rev` clamped to `1`), the Pi republishes the clamped value on `_current`, and the widget snaps to it — visual confirmation that the input got corrected.

### Control modes
The Arduino runs two control modes selected by `control_mode_RPi`:

- **Velocity mode (`control_mode = 0`)** — default and original behavior. `vel_ref_pi` follows the Pi's `vel_ref_receive` directly; `direction_target` follows the Pi's `direction_RPi`. The existing velocity PID tracks `vel_ref`.
- **Position mode (`control_mode = 1`)** — cascaded PID control with a slew-rate-limited inner setpoint. Outer PID loop on the Arduino computes a *signed* velocity from `pos_error = pos_ref_counts − encoderValue`:
  ```
  pos_error_integral += pos_error × dt   (anti-windup: |integral × pos_I_action| ≤ pos_max_vel)
  pos_vel_filt = ewma(measured signed velocity, alpha=0.2)  // velocity-feedback form of D
  vel_signed   = clamp(pos_error × pos_P_gain
                       + pos_error_integral × pos_I_action
                       − pos_D_gain × pos_vel_filt,
                       −pos_max_vel, +pos_max_vel)
  direction_target = sign(vel_signed)
  slew_target  = (current_direction == direction_target) ? |vel_signed| : 0
  vel_mag_smooth  +=/-=  pos_max_accel × dt   (toward slew_target, never overshooting)
  vel_ref_pi   = vel_mag_smooth
  ```
  - **`pos_I_action`** eliminates steady-state error — useful when `pos_P_gain` is small enough to avoid overshoot but the spit settles a few degrees short of target. Anti-windup is built in so the integrator can't accumulate beyond what would push `|I-term| > pos_max_vel`. Integral resets on entry to position mode and whenever mode switches back to velocity.
  - **`pos_D_gain`** dampens oscillation. Implemented as **velocity feedback** (`-pos_D_gain × measured_signed_velocity`), not `d/dt(error)`, for two reasons: (1) we already have a clean velocity estimate from the ISR's `vel`, and (2) differentiating `pos_error` would amplify single-pulse encoder timing jitter. The velocity estimate uses raw `vel/3` from the ISR with a pulse-silence staleness check (so it goes to 0 honestly when the spit is stopped — `control_vel` can't be used here because it's force-zeroed when `vel_ref<0.1`, exactly when the loop tends to oscillate around the setpoint), signed by `current_direction`, EWMA-smoothed (`alpha = 0.2`).
  - **Slew rate** has two practical effects:
    1. *Soft acceleration* — moving the position target far away no longer step-jumps `vel_ref_pi` to `pos_max_vel`; it ramps up at `pos_max_accel`.
    2. *Less switching at the setpoint* — near the target, brief sign flips in `vel_signed` from encoder pulse jitter can't immediately reverse `vel_mag_smooth`, so the motor doesn't twitch. The slew target also goes to 0 whenever the direction pin doesn't match yet, so the motor decelerates cleanly into a turnaround instead of racing the direction-flip latch.

**Mode switching (UX)**: flipping the mode switch from the dashboard snaps the new mode's setpoint to current state so the spit doesn't lurch:
- *Entering position mode*: Pi captures the current spit angle as `pos_ref_deg` (and republishes via the retained `_current` topic so the slider updates). Controller has `pos_error ≈ 0` the moment it takes over → spit holds in place.
- *Entering velocity mode*: Pi resets `vel_ref` and `vel_ref_target` to 0 and cancels any in-flight ramp. The slider on the dashboard does *not* visually follow (no retained-state on it yet); the motor is held at 0 until the user moves the slider.

### Control flow on the Arduino
1. **Two input sources** — a physical pot on `A0` and the Pi over serial. A latching flag `started_from_RPi` decides which `vel_ref` wins. The Pi only "owns" the motor if the start command came from the Pi; pressing the local start button reverts to the pot.
2. **Start/stop edge detection** — `start_stop_RPi` is treated as level, but only acts on the rising/falling edge (`start_stop_RPi && !start_stop_RPi_prev`). Repeated `"true"` payloads from Node-Red don't re-trigger.
3. **PI + feed-forward controller** — `ctrl = P*error + I_action*∫error + feed_forward`, integral anti-windup clamped to ±10, output saturated to 0–100 % duty, written via Timer4 fast PWM on pin D6 at ~23 kHz.
4. **Velocity sensing** — an ISR on pin 7 increments `encoderValue` and computes the instantaneous `vel = 1/dt`; this is smoothed by `velMovingAvg` and divided by 3 before being fed to the controller as `control_vel`.

### Setpoint ramping
To avoid jolts when the motor would otherwise see a step change, the Python script ramps `vel_ref` toward the slider value (`vel_ref_target`) at `RAMP_RATE` units/sec instead of sending the new setpoint immediately. Two triggers:

- **On Start** (`start_stop` rising edge): if the gap between `vel_ref_target` and the measured velocity exceeds `RAMP_TRIGGER_GAP`, `vel_ref` is reset to 0 and ramped up from there. Small gaps step straight to the target.
- **While running**: if a new slider value differs from the current `vel_ref` by more than `RAMP_TRIGGER_GAP`, the script ramps from the current `vel_ref` to the new target (both directions). Small changes step through immediately.

If the slider moves again mid-ramp, only `vel_ref_target` updates — the in-flight ramp re-aims at the new target on the next step instead of restarting.

Constants live near the top of [`Spit_RPi_Arduino.py`](RPi/Spit_RPi_Arduino.py): `RAMP_RATE` (units/sec), `RAMP_TRIGGER_GAP` (deadband, units), `RAMP_STEP_INTERVAL` (seconds between steps). The Arduino itself does no ramping — if the local pot is the source, the controller sees raw pot values.

## Data logging and viewer

The Pi records every cook session to disk and exposes the data through a small web viewer. The control loop is never blocked by logging: the writer runs on its own thread and reads from a bounded queue — if the queue ever fills, telemetry rows get dropped and counted rather than back-pressuring the loop.

### What gets logged

- **Time-series CSV per session** (20 Hz). One row per Arduino telemetry packet: `t_s, vel_ref, vel_measured, vel_error, u_duty, error_integral, encoder_count, spit_angle_deg, direction, control_mode, pos_ref_deg, pos_error_deg`. `u_duty` is the load-bearing one for "how hard is the motor working" — a healthy steady-state cook hovers at a low constant value, climbing under heavier load.
- **`sessions.csv`** — one row per *Python-script lifetime* (script boot → clean shutdown): start/end timestamps, duration, total rounds, direction flips, avg/max `u_duty`, avg/max `|vel_error|`, CRC error count, dropped-row count.
- **`events.csv`** — append-only log of state transitions: tunable changes (`P_gain: 2.0 -> 2.5`), mode flips, direction changes, Set Home presses, start/stop edges, CRC/payload errors.

### Where the files live

```
~/.spit_logs/
  sessions.csv
  events.csv
  ts/session_YYYY-MM-DD_HH-MM-SS.csv
```

Time-series files are capped at **500 MB total** (~80 hours of cooking). On startup and after every session close, the oldest files in `ts/` are deleted until the total is under the cap. `sessions.csv` and `events.csv` are tiny and kept forever.

### How sessions are decided

A session opens on the **first dashboard-Start press** after `Spit_RPi_Arduino.py` boots, and closes in the `finally` block on script shutdown. So one session = one cook, even if you start/stop the rotisserie many times during it — the subsequent start/stop transitions land in `events.csv` and show up as `vel_ref` drops in the time-series. Idle telemetry before the first Start press is dropped at the writer (nothing interesting is happening yet).

**Locally pot-started runs are not captured** — the Pi has no signal that the local button was pressed, so a session is never opened. If this bites you, the easy follow-up is to also open the session when `vel_measured > 0.5` is sustained.

**Power-loss caveat**: if the Pi is yanked rather than shut down cleanly, the time-series CSV survives up to the last flush (≤1 s of data loss), but the summary row in `sessions.csv` never gets written. The session file is still in `ts/` and viewable (it shows as `incomplete` in the viewer's session list).

### The Flask viewer

`RPi/viewer/app.py` runs as `spit-viewer.service` (systemd, enabled by `install.sh`) on port 5000. Reach it at `http://<pi-ip>:5000`:

- `/` — sessions table, newest first, with summary stats and a CSV download link per session. Built by scanning `ts/` directly so the **currently-running session** shows up with a green `running` badge mid-cook; previously-crashed sessions (file present but no summary row) show `incomplete`.
- `/session/<file>` — Plotly charts: `vel_ref` vs `vel_measured`, `u_duty` + `error_integral`, `vel_error`, position (`spit_angle_deg`, `pos_ref_deg`, `pos_error_deg`). Long sessions are downsampled to ~4k points before rendering so the browser stays snappy; the raw CSV is always available via the download link. **Charts auto-refresh every 3 s** by fetching `/session/<file>/data` and calling `Plotly.react()` — zoom/pan position is preserved across redraws via `uirevision`. Toggle off with the `live` checkbox at the top.
- `/events` — newest 500 events.

Plotly is loaded from a CDN — no JS build, no extra Python deps beyond Flask.

### Performance budget

20 Hz × ~80 B/row ≈ **1.6 KB/s** = 5.7 MB/h. A 4-hour cook is ~25 MB. The writer flushes + `fsync`s every 1 s or every 100 rows, so the worst-case data loss on power-cut is roughly the last second. Serial bandwidth headroom for the 5 extra telemetry bytes is fine — 100 B/s extra on a 115200 baud link is rounding error.

### Power-loss safety

Power cuts will happen — the Spit may be plugged into a switched outlet, the cook can run for hours, and yanking the Pi mid-write is a question of *when*, not *if*. What's protected at the code level and what isn't:

**Protected (code-level):**
- **Time-series writes** — the logger calls `flush()` + `os.fsync()` after every batch, so the file on disk is at most ~1 s behind reality. ext4's journal protects the file's metadata, so the worst case after a power cut is a truncated last line (CSV parsers tolerate this).
- **Event log** — every row is `fsync`'d before the file handle closes. An event recorded before the cut is durable.
- **Session summary** — also `fsync`'d. If the cut happens *before* the summary is written, the time-series file is still on disk and shows as `incomplete` in the viewer.
- **Settings file** (`~/.spit_settings.json`) — written atomically via `tmp → fsync → rename → directory fsync`. A power cut at any point either leaves the old file intact or the new file fully durable — never an empty/half-written file.

**Not fully protected by our code, but covered by `harden_pi.sh`:**
- The Pi's FAT32 `/boot` partition holds the bootloader and has no journal — a power cut while it's being written can brick the card. Our code never writes to `/boot`, but background apt updates and log rotation can. Run `./RPi/bash_scripts/harden_pi.sh` once per Pi to apply four SW-only hardening steps (each idempotent, safe to re-run):
  1. **Mount `/boot` read-only** — the single biggest brick-prevention win. After this, `/boot` corruption from a power cut is effectively impossible. To install a kernel/firmware update: `sudo mount -o remount,rw /boot`, do the update, then `sudo mount -o remount,ro /boot`.
  2. **`log2ram`** — moves `/var/log` to RAM, syncs to disk daily. Cuts steady-state SD wear and removes a corruption surface.
  3. **Disable swap** — eliminates swap-induced writes. The Pi 4 has plenty of RAM for this workload.
  4. **Disable unattended apt timers** — no background writes at random times. You still control manual `apt` commands.

**Still needs hardware (not addressable in SW alone):**
- SD card silicon-level corruption from brownouts is fundamentally a hardware problem. Even with all the above, an industrial-grade microSD (SanDisk Industrial, Samsung PRO Endurance) and a UPS HAT (PiJuice, UPS Pack Standard) would close the remaining gap.

The on-disk format itself is just append-only CSVs, so even partial corruption of one file doesn't affect any other session.

### Things to be aware of
- **Field order is load-bearing.** If you add a new gain on the Pi side, you must add the matching `rxObj` on the Arduino in the same position, or every following field shifts and gets garbage.
- **`delay(1 / 500)` in the Arduino main loop is `delay(0)`** — integer division. The loop runs as fast as it can, not at 500 Hz.
- **`config1kHzLoop` is defined but never called** — the 1 kHz timer ISR scaffolding is dead code right now. Control runs in the main loop, not on a timer interrupt.
- **`deltaTmain` uses millisecond resolution** via `millis()`, so the integral term's step size jitters by ±1 ms per loop iteration.
- **No integral reset when switching pot↔Pi** — only on stop and on `vel_ref < 0.1`. Quickly toggling sources mid-run will carry integral history across.

## Arduino Installation
### Install libraries
Using the Arduino IDE library manager, install the following libraries:

- [MovingAverageFloat](https://reference.arduino.cc/reference/en/libraries/movingaveragefloat/) by Pavel Slama.
- [SerialTransfer](https://www.arduino.cc/reference/en/libraries/serialtransfer/) by PowerBroker2.
- [movingAvg](https://www.arduino.cc/reference/en/libraries/movingavg/) by Jack Christensen

### Upload code
The Arduino installation is just uploading the latest version in the [Arduino folder](Arduino).

## Raspberry Pi Installation
For the raspberry pi to run correctly, we need to install a few things.

### Quick start: `install.sh`
After cloning the repo on the Pi, run the install script as the `pi` user:

```bash
cd ~/Documents/Spit-electronics
./RPi/install.sh
```

It installs Mosquitto and the Python deps, points Node-Red at the repo flow file, and adds the `start_spit.sh` boot entry to crontab. It's idempotent — safe to re-run after a `git pull`.

By default it does *not* run `apt upgrade`. Pass `--upgrade` if you want a full system upgrade as part of the install: `./RPi/install.sh --upgrade`.

**Prerequisite**: Node-Red must already be installed. If it isn't, run the [official installer](https://nodered.org/docs/getting-started/raspberrypi#installing-and-upgrading-node-red), then re-run `install.sh`.

The rest of this section documents what the script does, in case you want to do it by hand or debug a step.

### Installing Mosquitto broker
Run the following command to upgrade and update your system:

`sudo apt update && sudo apt upgrade`,

Then install the broker and client using

`sudo apt install -y mosquitto mosquitto-clients`,

And make the mosquitto broker autostart when the RPi starts:

`sudo systemctl enable mosquitto.service`,

Run the following code to test if the broker is running:

`mosquitto -v`.

For more about Mosquitto, such as allowing other devices on the network to access it and setting up passwords, see https://randomnerdtutorials.com/how-to-install-mosquitto-broker-on-raspberry-pi/.

### Installing the Python mqtt library
Install Python3 and pip if it is not installed yet: 
```
sudo apt update
sudo apt install python3-venv python3-pip
```

Use `pip` to install the paho-mqtt library:

`pip install paho-mqtt`

For example on usage, see the [Python script in this repository](RPi/Spit_RPi_Arduino.py). For more information and explanation, see https://medium.com/@potekh.anastasia/a-beginners-guide-to-mqtt-understanding-mqtt-mosquitto-broker-and-paho-python-mqtt-client-990822274923.

### Installing Node-Red
It is best to follow the instructions from the [node-red source](https://nodered.org/docs/getting-started/raspberrypi#installing-and-upgrading-node-red). Be sure to make it [autostart on boot](https://nodered.org/docs/getting-started/raspberrypi#autostart-on-boot). The editor can then be accessed in a [webbrowser](https://nodered.org/docs/getting-started/raspberrypi#opening-the-editor).

#### Point Node-Red at the repo flow
By default Node-Red reads and writes its flow from `~/.node-red/flows.json`, so the [flow checked into this repo](RPi/node-red) is ignored unless you tell Node-Red to use it. Edit `~/.node-red/settings.js` and set:

```js
flowFile: 'pathToRepo/RPi/node-red/flows.json',
credentialsFile: 'pathToRepo/RPi/node-red/flows_cred.json',
```

where `pathToRepo` is the absolute path to this repository. Then restart Node-Red:

`sudo systemctl restart nodered.service`

Every Deploy in the browser now writes straight into the repo file, so `git diff` shows your flow changes. The rest of the Node-Red state (`settings.js`, installed palette nodes, sessions) stays in `~/.node-red/` and out of the repo. Edit `settings.js` over SSH (e.g. with `nano`), not via SFTP — uploading from a local mirror is a known way to silently revert the `flowFile` line.

Node-Red also writes `.flows.json.backup` and `.flows_cred.json.backup` next to the flow file on every Deploy (its one-step undo). Both are `.gitignore`d.

Note: `flows_cred.json` is encrypted, but the key lives in `settings.js`. Only commit `flows_cred.json` if you're certain none of your flow credentials are sensitive.

### Autostart Python script on boot
Use `crontab` to make the RPi run the [start script](RPi/bash_scripts/start_spit.sh) on boot. First open `crontab`:

`$ crontab -e`

Just select `nano` as the editor. Then add the `start_spit.sh` to the boot command:

`@reboot sleep 10 pathToRepo/RPi/bash_scripts/start_spit.sh >> /home/pi/crontab_log.txt 2>&1`

where `pathToRepo` is the path to this repository.

## Network reference
Current addresses for the dev setup. Update if anything is re-IP'd.

| Device                              | MAC                 | IP              |
|-------------------------------------|---------------------|-----------------|
| Raspberry Pi (`eth0`, to dev router) | `DC:A6:32:4C:EF:69` | `192.168.2.100` |
| Dev router (Pi ↔ laptop bridge)      | —                   | `192.168.2.10`  |

The Pi's Wi-Fi interface (`wlan0`) gets a DHCP lease from the Odido router on a different subnet and is the path to the internet. Reach the Pi over Ethernet at `192.168.2.100` (e.g. `ssh pi@192.168.2.100`); reach it over Wi-Fi at whatever its current `wlan0` lease is (`ip a` on the Pi).

## Connecting the Spit
TODO

## Using the Spit
TODO

## Debugging
### Kill running Python scripts
It could happen that the Python script that is ran at startup doesn't exit properly and keeps running in the background, causing all kinds of issues. To kill it (or any other Python process for that manner), type in a terminal:

`ps -ef | grep python`

and find the code of the program you want to kill. Then just simply do

`kill CodeYouFound`

where `CodeYouFound` is of course the code you found in the previous step.

For instance, when running the `grep` command I get
```
pi          1160       1  0 20:42 ?        00:00:00 /usr/bin/python3 /usr/share/system-config-printer/applet.py
pi          2102    2085 67 21:18 pts/0    00:00:04 python3 ../Spit_RPi_Arduino.py
pi          2119    1566  0 21:18 pts/0    00:00:00 grep --color=auto python
```
so if want to kill the Spit program, I would do
`kill 2102`. You can then run the `grep` command again to check if it is indeed killed.

## TODO
- Add sound with speakers
- Add system identification
- Add oscillate mode — sweep back and forth around a user-set center position (e.g. "bottom of the pig" for grilling one side only). Live-tunable from the dashboard: oscillation range (± degrees around center, e.g. 20° = sweeps from −20° to +20°) and center position (captured via a "set home" button at the current encoder angle). Sweep speed reuses the existing `vel_ref` slider — the controller stays in velocity mode and just flips the sign of `vel_ref` whenever the encoder crosses a range endpoint.
- Add voice notifications over the speaker (Piper TTS on the Pi) for milestone events, e.g. "1000 rotations reached" or "done in 7 minutes". Triggered off MQTT, so the same events could later also drive a Telegram/Discord bot.
