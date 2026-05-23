// Spit lite dashboard — talks straight to Mosquitto over WebSockets.
// Topic semantics mirror RPi/node-red/flows.json and Spit_RPi_Arduino.py.

(() => {
  const WS_PORT = 9001;
  const WS_PATH = "/";
  const BROKER_URL = `ws://${location.hostname || "localhost"}:${WS_PORT}${WS_PATH}`;

  // --- Topics -----------------------------------------------------------------
  const TX = {
    vel_ref:        "mqtt/rpi/rx/vel_ref",
    start_stop:     "mqtt/rpi/rx/start_stop",
    direction:      "mqtt/rpi/rx/direction",
    p_gain:         "mqtt/rpi/rx/p_gain",
    i_action:       "mqtt/rpi/rx/i_action",
    feed_forward:   "mqtt/rpi/rx/feed_forward",
    counts_per_rev: "mqtt/rpi/rx/counts_per_rev",
    set_home:       "mqtt/rpi/rx/set_home",
    save_settings:  "mqtt/rpi/rx/save_settings",
    control_mode:   "mqtt/rpi/rx/control_mode",
    pos_ref_deg:    "mqtt/rpi/rx/pos_ref_deg",
    pos_P_gain:     "mqtt/rpi/rx/pos_P_gain",
    pos_max_vel:    "mqtt/rpi/rx/pos_max_vel",
    pos_max_accel:  "mqtt/rpi/rx/pos_max_accel",
    pos_I_action:   "mqtt/rpi/rx/pos_I_action",
    pos_D_gain:     "mqtt/rpi/rx/pos_D_gain",
  };

  const RX = {
    num_rounds:     "mqtt/rpi/num_rounds",
    vel_measured:   "mqtt/rpi/vel_measured",
    vel_ref_sent:   "mqtt/rpi/vel_ref_sent",
    spit_angle_deg: "mqtt/rpi/spit_angle_deg",
    P_gain:         "mqtt/rpi/P_gain_current",
    I_action:       "mqtt/rpi/I_action_current",
    feed_forward:   "mqtt/rpi/feed_forward_current",
    counts_per_rev: "mqtt/rpi/counts_per_rev_current",
    control_mode:   "mqtt/rpi/control_mode_current",
    pos_ref_deg:    "mqtt/rpi/pos_ref_deg_current",
    pos_P_gain:     "mqtt/rpi/pos_P_gain_current",
    pos_max_vel:    "mqtt/rpi/pos_max_vel_current",
    pos_max_accel:  "mqtt/rpi/pos_max_accel_current",
    pos_I_action:   "mqtt/rpi/pos_I_action_current",
    pos_D_gain:     "mqtt/rpi/pos_D_gain_current",
  };

  // --- Connection -------------------------------------------------------------
  const connDot = document.getElementById("conn-dot");
  const connText = document.getElementById("conn-text");
  const setConn = (state, label) => {
    connDot.classList.remove("ok", "bad");
    if (state === "ok") connDot.classList.add("ok");
    if (state === "bad") connDot.classList.add("bad");
    connText.textContent = label;
  };

  const client = mqtt.connect(BROKER_URL, {
    reconnectPeriod: 2000,
    connectTimeout: 4000,
    clientId: "spit-lite-" + Math.random().toString(16).slice(2, 10),
  });

  const markConnected = (ok) => document.body.classList.toggle("disconnected", !ok);
  client.on("connect", () => {
    setConn("ok", "connected"); markConnected(true);
    for (const t of Object.values(RX)) client.subscribe(t);
  });
  client.on("reconnect", () => { setConn("bad", "reconnecting…"); markConnected(false); });
  client.on("close",     () => { setConn("bad", "disconnected");  markConnected(false); });
  client.on("error", (e) => { setConn("bad", "error: " + e.message); markConnected(false); });

  const pub = (topic, payload, opts = {}) => {
    if (!client.connected) return;
    client.publish(topic, String(payload), opts);
  };

  // --- State ------------------------------------------------------------------
  const state = {
    start_stop: 0,
    direction: 0,
    control_mode: 0,
    spit_angle_deg: 0,
    pos_ref_deg: 0,
    vel_ref_target: 0,
    inputFocus: new Set(), // ids of inputs the user is editing — don't clobber
  };

  // --- Helpers ----------------------------------------------------------------
  const $ = (id) => document.getElementById(id);
  const wrap360 = (d) => ((d % 360) + 360) % 360;
  const fmt = (n, p = 1) => (Number.isFinite(n) ? n.toFixed(p) : "—");

  // Mark inputs while focused so retained _current messages don't overwrite mid-edit.
  const protectInput = (el) => {
    el.addEventListener("focus", () => state.inputFocus.add(el.id));
    el.addEventListener("blur",  () => state.inputFocus.delete(el.id));
  };

  // Bind a numeric input to an MQTT setting (publish on change, accept retained _current).
  const bindNumberInput = (id, topic) => {
    const el = $(id);
    protectInput(el);
    el.addEventListener("change", () => {
      if (el.value === "") return;
      pub(topic, el.value);
    });
  };

  bindNumberInput("P_gain",         TX.p_gain);
  bindNumberInput("I_action",       TX.i_action);
  bindNumberInput("feed_forward",   TX.feed_forward);
  bindNumberInput("counts_per_rev", TX.counts_per_rev);
  bindNumberInput("pos_P_gain",     TX.pos_P_gain);
  bindNumberInput("pos_I_action",   TX.pos_I_action);
  bindNumberInput("pos_D_gain",     TX.pos_D_gain);
  bindNumberInput("pos_max_vel",    TX.pos_max_vel);
  bindNumberInput("pos_max_accel",  TX.pos_max_accel);

  $("set-home").addEventListener("click", () => pub(TX.set_home, "1"));
  $("save-settings").addEventListener("click", () => {
    pub(TX.save_settings, "1");
    const b = $("save-settings");
    const orig = b.textContent;
    b.textContent = "Saved ✓";
    setTimeout(() => (b.textContent = orig), 1200);
  });

  // Mode segmented control
  document.querySelectorAll("#mode-seg button").forEach((btn) => {
    btn.addEventListener("click", () => {
      const m = Number(btn.dataset.mode);
      pub(TX.control_mode, String(m));
      setMode(m);
    });
  });
  // Direction segmented control
  document.querySelectorAll("#dir-seg button").forEach((btn) => {
    btn.addEventListener("click", () => {
      const d = Number(btn.dataset.dir);
      pub(TX.direction, String(d));
      setDir(d);
    });
  });

  // Start/stop
  const ss = $("startstop");
  ss.addEventListener("click", () => {
    const next = state.start_stop ? 0 : 1;
    pub(TX.start_stop, next ? "true" : "false");
    setStartStop(next);
  });

  // --- Velocity slider --------------------------------------------------------
  const velSlider = $("vel-slider");
  const velNum    = $("vel-num");
  protectInput(velNum);

  let velSendThrottle = 0;
  const sendVelRef = (v) => {
    state.vel_ref_target = v;
    const now = performance.now();
    if (now - velSendThrottle < 80) return; // ~12 Hz cap
    velSendThrottle = now;
    pub(TX.vel_ref, String(v));
  };
  const velCurrent = $("vel-current");
  const setVelCurrent = (v) => { if (velCurrent) velCurrent.textContent = fmt(v, 1); };
  velSlider.addEventListener("input", () => {
    const v = parseFloat(velSlider.value);
    velNum.value = v;
    setVelCurrent(v);
    sendVelRef(v);
  });
  velSlider.addEventListener("change", () => pub(TX.vel_ref, String(velSlider.value)));
  velNum.addEventListener("change", () => {
    const v = parseFloat(velNum.value);
    if (!Number.isFinite(v)) return;
    velSlider.value = Math.max(velSlider.min, Math.min(velSlider.max, v));
    setVelCurrent(v);
    pub(TX.vel_ref, String(v));
  });

  // --- Position knob (SVG, multi-turn-aware) ----------------------------------
  // Knob shows wrapped 0-360. Numeric input keeps multi-turn precision.
  const knob       = $("knob");
  const knobHandle = $("knob-handle");
  const knobCur    = $("knob-current");
  const posNum     = $("pos-num");
  protectInput(posNum);

  let knobDragging = false;
  let knobSendT = 0;
  let knobLastDeg = null;

  const knobPointToDeg = (clientX, clientY) => {
    const r = knob.getBoundingClientRect();
    const cx = r.left + r.width / 2;
    const cy = r.top + r.height / 2;
    const dx = clientX - cx;
    const dy = clientY - cy;
    // 0° at top (north), increasing clockwise.
    let deg = Math.atan2(dx, -dy) * 180 / Math.PI;
    if (deg < 0) deg += 360;
    return deg;
  };

  const setKnobHandle = (deg) => {
    knobHandle.setAttribute("transform", `rotate(${deg})`);
  };
  const setKnobCurrent = (deg) => {
    knobCur.setAttribute("transform", `rotate(${deg})`);
  };

  const onKnobMove = (clientX, clientY) => {
    const deg = knobPointToDeg(clientX, clientY);
    setKnobHandle(deg);
    const now = performance.now();
    const delta = knobLastDeg == null ? Infinity : Math.abs(deg - knobLastDeg);
    if (delta > 1 || now - knobSendT > 100) {
      knobSendT = now;
      knobLastDeg = deg;
      // Knob outputs wrapped 0-360 (matches Node-Red knob behavior).
      pub(TX.pos_ref_deg, deg.toFixed(1));
      if (!state.inputFocus.has("pos-num")) posNum.value = deg.toFixed(1);
    }
  };

  const handleDown = (e) => {
    knobDragging = true;
    knob.setPointerCapture && e.pointerId != null && knob.setPointerCapture(e.pointerId);
    onKnobMove(e.clientX, e.clientY);
  };
  const handleUp = (e) => {
    if (!knobDragging) return;
    knobDragging = false;
    knobLastDeg = null;
    // Fire one final update so the target lands exactly where the user let go.
    onKnobMove(e.clientX, e.clientY);
  };
  knob.addEventListener("pointerdown", handleDown);
  knob.addEventListener("pointermove", (e) => { if (knobDragging) onKnobMove(e.clientX, e.clientY); });
  knob.addEventListener("pointerup", handleUp);
  knob.addEventListener("pointercancel", handleUp);

  posNum.addEventListener("change", () => {
    const v = parseFloat(posNum.value);
    if (!Number.isFinite(v)) return;
    pub(TX.pos_ref_deg, String(v));
    setKnobHandle(wrap360(v));
  });

  // --- Compass ticks ----------------------------------------------------------
  const buildTicks = (container) => {
    if (!container || container.childElementCount > 0) return; // inline fallback may have drawn them already
    for (let i = 0; i < 36; i++) {
      const major = i % 9 === 0;
      const x1 = 0, y1 = -100, x2 = 0, y2 = major ? -88 : -94;
      const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
      line.setAttribute("x1", x1); line.setAttribute("y1", y1);
      line.setAttribute("x2", x2); line.setAttribute("y2", y2);
      line.setAttribute("transform", `rotate(${i * 10})`);
      if (major) line.classList.add("major");
      container.appendChild(line);
    }
  };
  buildTicks($("ticks"));
  buildTicks($("knob-ticks"));

  // --- UI state setters -------------------------------------------------------
  const ssLbl = ss.querySelector(".lbl");
  const setStartStop = (v) => {
    state.start_stop = v;
    ss.classList.toggle("running", !!v);
    ss.classList.toggle("start", !v);
    if (ssLbl) ssLbl.textContent = v ? "STOP" : "START";
    else ss.textContent = v ? "STOP" : "START";
    ss.setAttribute("aria-pressed", v ? "true" : "false");
  };
  const setDir = (d) => {
    state.direction = d;
    document.querySelectorAll("#dir-seg button").forEach((b) => {
      b.classList.toggle("active", Number(b.dataset.dir) === d);
    });
  };
  const setMode = (m) => {
    state.control_mode = m;
    document.querySelectorAll("#mode-seg button").forEach((b) => {
      b.classList.toggle("active", Number(b.dataset.mode) === m);
    });
    $("ctl-vel").classList.toggle("hidden", m === 1);
    $("ctl-pos").classList.toggle("hidden", m === 0);
    document.getElementById("target-needle").classList.toggle("hidden", m === 0);
  };

  const setAngle = (deg) => {
    state.spit_angle_deg = deg;
    const wrapped = wrap360(deg);
    document.getElementById("needle").setAttribute("transform", `rotate(${wrapped})`);
    $("angle-deg").textContent = deg.toFixed(1);
    const turns = deg / 360;
    $("angle-multi").textContent = `${turns >= 0 ? "+" : ""}${turns.toFixed(2)} turns`;
  };

  const setPosTarget = (deg) => {
    state.pos_ref_deg = deg;
    setKnobCurrent(wrap360(deg));
    document.getElementById("target-needle").setAttribute("transform", `rotate(${wrap360(deg)})`);
    if (!state.inputFocus.has("pos-num")) posNum.value = Number.isInteger(deg) ? deg : deg.toFixed(1);
  };

  const safelySetInput = (id, value) => {
    if (state.inputFocus.has(id)) return;
    const el = $(id);
    if (!el) return;
    if (document.activeElement === el) return;
    el.value = value;
  };

  // --- Incoming MQTT ----------------------------------------------------------
  client.on("message", (topic, buf) => {
    const payload = buf.toString();
    if (payload === "") return;

    switch (topic) {
      case RX.spit_angle_deg: {
        const v = parseFloat(payload);
        if (Number.isFinite(v)) setAngle(v);
        break;
      }
      case RX.num_rounds: {
        const v = parseFloat(payload);
        $("num-rounds").textContent = Number.isFinite(v) ? fmt(v, 1) : payload;
        break;
      }
      case RX.vel_measured: {
        const v = parseFloat(payload);
        $("vel-meas").textContent = fmt(v, 1);
        break;
      }
      case RX.vel_ref_sent: {
        const v = parseFloat(payload);
        $("vel-sent").textContent = fmt(v, 1);
        // Don't fight the user's slider while they're dragging it.
        if (!knobDragging && document.activeElement !== velSlider && document.activeElement !== velNum) {
          if (Number.isFinite(v)) {
            velSlider.value = Math.max(velSlider.min, Math.min(velSlider.max, v));
            velNum.value = v;
            setVelCurrent(v);
          }
        }
        break;
      }
      case RX.control_mode: {
        const m = payload === "1" || payload.toLowerCase() === "position" ? 1 : 0;
        setMode(m);
        break;
      }
      case RX.pos_ref_deg: {
        const v = parseFloat(payload);
        if (Number.isFinite(v)) setPosTarget(v);
        break;
      }
      case RX.P_gain:         safelySetInput("P_gain", payload); break;
      case RX.I_action:       safelySetInput("I_action", payload); break;
      case RX.feed_forward:   safelySetInput("feed_forward", payload); break;
      case RX.counts_per_rev: safelySetInput("counts_per_rev", payload); break;
      case RX.pos_P_gain:     safelySetInput("pos_P_gain", payload); break;
      case RX.pos_I_action:   safelySetInput("pos_I_action", payload); break;
      case RX.pos_D_gain:     safelySetInput("pos_D_gain", payload); break;
      case RX.pos_max_vel:    safelySetInput("pos_max_vel", payload); break;
      case RX.pos_max_accel:  safelySetInput("pos_max_accel", payload); break;
    }
  });

  // Initial UI state
  document.body.classList.add("disconnected");
  setStartStop(0);
  setDir(0);
  setMode(0);
  setAngle(0);
  setKnobHandle(0);
  setKnobCurrent(0);
})();
