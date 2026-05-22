# Spit-electronics

## Architecture
The SpitTronics uses an Arduino Leonardo to send PWM pulses to an H-bridge, which allows for setting the voltage the motor receives. The Arduino receives encoder pulses on one of its digital pins.

The Arduino in turn is connected to a Raspberry Pi 4 via USB. Using the [SerialTransfer library](https://github.com/PowerBroker2/SerialTransfer/tree/master) on the Arduino side and the corresponding Python code ([pySerial](https://github.com/PowerBroker2/pySerialTransfer/tree/master)), data is send back and forth between the RPi and the Arduino.

The RPi uses MQTT to make the data that is received in the Python script available anywhere on the RPi. Node-red then subscribes to these topics and the data is visualized in a node-red-dashboard.

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

`start_stop` and `direction` are sent with `val_type_override='B'` (unsigned byte) — without that override, pySerialTransfer packs a Python `int` as 4 bytes by default, which would shift every following field by 3 bytes.

**Direction latching.** A direction flip is only applied to `directionPin` when both: (a) PWM duty `u` is effectively zero (the H-bridge isn't delivering power), and (b) no encoder pulse has fired for `DIRECTION_FLIP_QUIET_MS` (default `200` ms). The encoder check is the load-bearing one — `control_vel` can't be trusted as a "motor stopped" signal: the ISR only updates `vel` on each pulse, so when pulses stop `vel` is frozen at its last value indefinitely; and line `control_vel = 0.0` force-zeros it whenever `vel_ref < 0.1` regardless of physical motion. The pulse-silence check via `lastPulseTime` (stamped in the ISR) is the only honest "motor is at rest" signal we have. 200 ms of silence at 305 pulses/rev bounds motion below ~6°/sec — effectively stationary.

This makes oscillate mode work without a special protocol: leave `motor_start = true`, command `vel_ref` toward zero near an endpoint, then publish the direction flip — once the spit physically coasts down (a few hundred ms after PWM drops to zero), the latched direction applies and the next `vel_ref > 0` accelerates the other way.

### Arduino → Pi (telemetry)
Every 50 ms the Arduino sends:

| Offset | Field         | Type   | Meaning                                  |
|--------|---------------|--------|------------------------------------------|
| 0      | `control_vel` | float  | Moving-average measured velocity         |
| 4      | `encoderSend` | uint32 | Raw encoder pulse count since boot       |

The Pi divides the encoder count by 305 to get `num_rounds` and republishes both on MQTT (`mqtt/rpi/vel_measured`, `mqtt/rpi/num_rounds`) for Node-Red to graph.

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
- Add data logging and replay
- Add visualization
- Add sound with speakers
- Add system identification
- Add oscillate mode — sweep back and forth around a user-set center position (e.g. "bottom of the pig" for grilling one side only). Live-tunable from the dashboard: oscillation range (± degrees around center, e.g. 20° = sweeps from −20° to +20°) and center position (captured via a "set home" button at the current encoder angle). Sweep speed reuses the existing `vel_ref` slider — the controller stays in velocity mode and just flips the sign of `vel_ref` whenever the encoder crosses a range endpoint.
- Add voice notifications over the speaker (Piper TTS on the Pi) for milestone events, e.g. "1000 rotations reached" or "done in 7 minutes". Triggered off MQTT, so the same events could later also drive a Telegram/Discord bot.
