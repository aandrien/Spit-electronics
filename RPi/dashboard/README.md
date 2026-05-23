# Spit lite dashboard

A static HTML/JS dashboard that talks straight to Mosquitto over WebSockets — no Node-Red, no Node.js, no internet needed after install. Designed for the tablet (1024×600 touch, landscape).

## What it is

- `index.html`, `style.css`, `app.js` — the entire UI. One page, no build step.
- `vendor/mqtt.min.js` — vendored [MQTT.js](https://github.com/mqttjs/MQTT.js) v5 browser bundle (no CDN at runtime).
- `mosquitto-websockets.conf` — Mosquitto listener on `:9001` (`/etc/mosquitto/conf.d/websockets.conf`).
- `nginx-spit-dashboard.conf.template` — nginx site on `:8080`. Installer substitutes the absolute dashboard path.

`RPi/install.sh` installs nginx, drops the configs, and restarts both services. Node-Red on `:1880` is untouched — the two dashboards coexist.

## Architecture

```
 tablet browser ──HTTP:8080──► nginx ──serves──► RPi/dashboard/*
        │
        └─MQTT/WS:9001──► mosquitto ◄──MQTT/1883── Spit_RPi_Arduino.py ──serial──► Arduino
```

All control state lives in MQTT topics; this UI is a thin pub/sub view. Topic layout matches `Spit_RPi_Arduino.py` exactly — every `_current` retained topic seeds the corresponding input on connect.

## Widget parity with Node-Red

Velocity slider + numeric • start/stop • direction • mode (velocity/position) • velocity PID (P, I, FF) • position PID (P, I, D, max vel, max accel) • counts/rev • set home • save settings • spit-angle compass • multi-turn angle readout • draggable position knob (with current target indicator) • num_rounds counter • measured velocity • last sent velocity setpoint.

## Local dev (off the Pi)

```bash
cd RPi/dashboard
python3 -m http.server 8080
```

Then open `http://<host-with-mosquitto>:8080/` — the page connects to `ws://<page-host>:9001/`. To point at a Pi from a laptop dev session, just open the page via the Pi's IP, or temporarily edit `BROKER_URL` in `app.js`.

## Offline guarantees

- No CDN calls — `mqtt.min.js` is vendored.
- No fonts, no analytics, no service worker.
- One TCP connection to `:9001` (MQTT over WS).

## Tweaking

- **Velocity slider range** — `index.html`, the `<input type="range" id="vel-slider">` element (`min`/`max`).
- **Publish throttle** — `velSendThrottle` in `app.js` (default 80 ms ≈ 12 Hz). Node-Red's slider is similarly rate-limited.
- **Knob → MQTT cadence** — `onKnobMove` in `app.js` fires on >1° change or every 100 ms (mirrors the Node-Red SVG knob).
- **Connection target** — `BROKER_URL` at the top of `app.js`. Defaults to the page host on `:9001`.
