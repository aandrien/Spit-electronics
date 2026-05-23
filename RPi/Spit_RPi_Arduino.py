import sys
import os
import json
import math
import time
import paho.mqtt.client as paho
import numpy as np
from pySerialTransfer import pySerialTransfer as txfer

from spit_logger import SpitLogger

logger = SpitLogger()

SETTINGS_PATH = os.path.expanduser("~/.spit_settings.json")

## Definitions ##
hostname = "localhost"
broker_port = 1883
send_measured_num_rounds = "mqtt/rpi/num_rounds"
send_measured_vel_topic = "mqtt/rpi/vel_measured"
send_vel_ref_topic = "mqtt/rpi/vel_ref_sent"
send_spit_angle_topic = "mqtt/rpi/spit_angle_deg"
receive_vel_ref_topic = "mqtt/rpi/rx/vel_ref"
receive_startstop_topic = "mqtt/rpi/rx/start_stop"
receive_p_gain_topic = "mqtt/rpi/rx/p_gain"
receive_i_action_topic = "mqtt/rpi/rx/i_action"
receive_feed_forward_topic = "mqtt/rpi/rx/feed_forward"
receive_direction_topic = "mqtt/rpi/rx/direction"
receive_counts_per_rev_topic = "mqtt/rpi/rx/counts_per_rev"
receive_set_home_topic = "mqtt/rpi/rx/set_home"
receive_save_settings_topic = "mqtt/rpi/rx/save_settings"
receive_control_mode_topic = "mqtt/rpi/rx/control_mode"
receive_pos_ref_deg_topic = "mqtt/rpi/rx/pos_ref_deg"
receive_pos_P_gain_topic = "mqtt/rpi/rx/pos_P_gain"
receive_pos_max_vel_topic = "mqtt/rpi/rx/pos_max_vel"
receive_pos_max_accel_topic = "mqtt/rpi/rx/pos_max_accel"
receive_pos_I_action_topic = "mqtt/rpi/rx/pos_I_action"
receive_pos_D_gain_topic = "mqtt/rpi/rx/pos_D_gain"
receive_topic = "mqtt/rpi/rx/#"

# Retained "current value" topics — the dashboard subscribes to these so the
# text inputs show the real value on load and snap to the clamped value
# when the user types something out of range. Keyed by the Python variable name.
CURRENT_TOPICS = {
    "counts_per_rev": "mqtt/rpi/counts_per_rev_current",
    "P_gain":         "mqtt/rpi/P_gain_current",
    "I_action":       "mqtt/rpi/I_action_current",
    "feed_forward":   "mqtt/rpi/feed_forward_current",
    "control_mode":   "mqtt/rpi/control_mode_current",
    "pos_ref_deg":    "mqtt/rpi/pos_ref_deg_current",
    "pos_P_gain":     "mqtt/rpi/pos_P_gain_current",
    "pos_max_vel":    "mqtt/rpi/pos_max_vel_current",
    "pos_max_accel":  "mqtt/rpi/pos_max_accel_current",
    "pos_I_action":   "mqtt/rpi/pos_I_action_current",
    "pos_D_gain":     "mqtt/rpi/pos_D_gain_current",
}

send_client = paho.Client()
receive_client = paho.Client()


vel_measured = 0.0
encoderCount = 0
u_duty = 0              # latest PWM duty (0..100) the Arduino reported
error_integral = 0.0    # latest velocity-PID integral term the Arduino reported
prev_encoder_count = None  # None until the first telemetry packet arrives
total_abs_pulses = 0       # cumulative |delta| — counts revolutions in both directions equally

# Signed-position tracking — derived purely on the Pi from the signed encoderCount.
# counts_per_rev is live-tunable from Node-Red (default 4270 = 305 * 14 from the
# pre-existing num_rounds + dashboard divisor). zero_count is captured when the
# user presses "Set Home"; spit_angle_deg = (encoderCount - zero_count) / counts_per_rev * 360.
counts_per_rev = 4270.0
zero_count = 0

# Cascaded position-control state. control_mode toggles the inner loop's source:
#   0 = velocity mode (existing behavior, Pi-set vel_ref is tracked)
#   1 = position mode (Arduino computes its own vel_ref from pos_ref - encoderCount)
# pos_ref is set by the user in degrees-from-home on the dashboard, converted to
# absolute encoder counts inside sendData() using zero_count + counts_per_rev.
control_mode = 0
pos_ref_deg = 0.0
pos_P_gain = 0.05
pos_I_action = 0.0   # integral gain on the outer position loop; off by default (set on dashboard)
pos_D_gain = 0.0     # D gain on measured velocity (velocity-feedback form, not error derivative); off by default
pos_max_vel = 30.0
pos_max_accel = 25.0  # vel-units/sec slew on the position-loop output; matches Pi-side RAMP_RATE in velocity mode


if send_client.connect("localhost", broker_port, 60) != 0:
    print("Couldn't connect to the mqtt broker")
    sys.exit(1)

if receive_client.connect("localhost", broker_port, 60) != 0:
    print("Couldn't connect to the mqtt broker")
    sys.exit(1)

message = str([])
send_client.publish(send_measured_vel_topic, message)
send_client.publish(send_vel_ref_topic, str(0.0))

start_stop = 0
vel_ref = 0.0
vel_ref_target = 0.0
P_gain = 2.0
I_action = 10.0
feed_forward = 0.0
direction = 0  # 0 = forward, 1 = reverse — drives Arduino directionPin

# Ramp: on Start, vel_ref ramps from 0 up to vel_ref_target. While running,
# a setpoint change larger than RAMP_TRIGGER_GAP ramps from the current
# vel_ref toward the new target at RAMP_RATE units/sec; smaller changes step
# through immediately.
RAMP_RATE = 25.0
RAMP_TRIGGER_GAP = 1.0
RAMP_STEP_INTERVAL = 0.05

prev_start_stop = 0
ramping = False
last_ramp_time = 0.0

def sendData():
    sendSize = 0

    ###################################################################
    # Send vel_ref
    ###################################################################
    sendSize = link.tx_obj(vel_ref, start_pos=sendSize)

    ###################################################################
    # Send P gain
    ###################################################################
    sendSize = link.tx_obj(P_gain, start_pos=sendSize)

    ###################################################################
    # Send I gain
    ###################################################################
    sendSize = link.tx_obj(I_action, start_pos=sendSize)

    ###################################################################
    # Send feed_forward
    ###################################################################
    sendSize = link.tx_obj(feed_forward, start_pos=sendSize)

    ###################################################################
    # Send start/stop command (uint8 — must match Arduino's uint8_t rxObj)
    ###################################################################
    sendSize = link.tx_obj(start_stop, start_pos=sendSize, val_type_override='B')

    ###################################################################
    # Send direction (0 = forward, 1 = reverse; uint8)
    ###################################################################
    sendSize = link.tx_obj(direction, start_pos=sendSize, val_type_override='B')

    ###################################################################
    # Position-mode fields. pos_ref_counts is computed here so the latest
    # zero_count and counts_per_rev are always reflected without needing
    # to recompute on every config change.
    ###################################################################
    pos_ref_counts = int(zero_count + (pos_ref_deg / 360.0) * counts_per_rev)
    sendSize = link.tx_obj(control_mode,   start_pos=sendSize, val_type_override='B')
    sendSize = link.tx_obj(pos_ref_counts, start_pos=sendSize, val_type_override='l')
    sendSize = link.tx_obj(pos_P_gain,     start_pos=sendSize)
    sendSize = link.tx_obj(pos_max_vel,    start_pos=sendSize)
    sendSize = link.tx_obj(pos_max_accel,  start_pos=sendSize)
    sendSize = link.tx_obj(pos_I_action,   start_pos=sendSize)
    sendSize = link.tx_obj(pos_D_gain,     start_pos=sendSize)

    link.send(sendSize)

def publish_current(var_name):
    """Republish the variable's current value on its _current topic with retain=True.
    Called after every value change so dashboard widgets stay in sync and
    snap back to the clamped value if the user typed something out of range."""
    topic = CURRENT_TOPICS.get(var_name)
    if topic is not None:
        send_client.publish(topic, str(globals()[var_name]), retain=True)

def make_float_handler(var_name, lo=None, hi=None):
    def handler(client, userdata, msg):
        payload = msg.payload.decode().strip()
        if payload == "":
            return
        value = float(payload)
        if lo is not None:
            value = min(max(value, lo), hi)
        old = globals().get(var_name)
        globals()[var_name] = value
        sendData()
        publish_current(var_name)
        # Only log when the value actually changed — the dashboard re-publishes
        # all values on every edit (see sendData()), so without this filter
        # we'd flood events.csv with no-op rows.
        if old != value:
            logger.log_event("tunable_change", f"{var_name}: {old} -> {value}")
    return handler

# (variable name, min, max) for everything persisted to disk. All entries are
# validated the same way on load: must parse as float, must be finite, clamped
# into [lo, hi]. Anything failing those checks logs and falls back to the
# in-code default — the script always reaches the main loop. Operational state
# (control_mode, pos_ref_deg, vel_ref, start_stop, direction) is intentionally
# NOT persisted — those are "where the spit is right now" not "how it's tuned".
PERSISTED_VARS = [
    ("counts_per_rev", 1.0,    1.0e6),
    ("P_gain",         0.0,    5.0),
    ("I_action",       0.0,    15.0),
    ("feed_forward",  -1.0e6,  1.0e6),
    ("pos_P_gain",     0.0,    10.0),
    ("pos_max_vel",    0.0,    1000.0),
    ("pos_max_accel",  0.1,    1000.0),
    ("pos_I_action",   0.0,    10.0),
    ("pos_D_gain",     0.0,    100.0),
]

def load_settings():
    """Load persisted settings from disk. Any failure (missing file, malformed
    JSON, wrong type, out-of-range value) falls back to in-code defaults."""
    try:
        with open(SETTINGS_PATH, "r") as f:
            data = json.load(f)
    except FileNotFoundError:
        return
    except Exception as e:
        print(f"Failed to read {SETTINGS_PATH}: {e} — using defaults")
        return

    if not isinstance(data, dict):
        print(f"{SETTINGS_PATH} doesn't contain a JSON object — using defaults")
        return

    for var_name, lo, hi in PERSISTED_VARS:
        if var_name not in data:
            continue
        raw = data[var_name]
        try:
            value = float(raw)
        except (TypeError, ValueError):
            print(f"Invalid {var_name}={raw!r} in {SETTINGS_PATH} — using default")
            continue
        if not math.isfinite(value):
            print(f"Non-finite {var_name}={value} in {SETTINGS_PATH} — using default")
            continue
        if not (lo <= value <= hi):
            clamped = min(max(value, lo), hi)
            print(f"{var_name}={value} from {SETTINGS_PATH} out of range — clamped to {clamped}")
            value = clamped
        globals()[var_name] = value
        print(f"Loaded {var_name}={value} from {SETTINGS_PATH}")

def save_settings():
    """Persist user-tunable settings to disk atomically (write to .tmp then
    rename) so a crash mid-write can't leave a half-written file behind."""
    tmp_path = SETTINGS_PATH + ".tmp"
    try:
        with open(tmp_path, "w") as f:
            json.dump({name: globals()[name] for name, _, _ in PERSISTED_VARS}, f)
        os.replace(tmp_path, SETTINGS_PATH)
        print(f"Saved settings to {SETTINGS_PATH}")
    except Exception as e:
        print(f"Failed to save {SETTINGS_PATH}: {e}")
        try:
            os.remove(tmp_path)
        except OSError:
            pass

def message_handling_save_settings(client, userdata, msg):
    if msg.payload.decode().strip() == "":
        return
    save_settings()

def publish_vel_ref_sent():
    send_client.publish(send_vel_ref_topic, str(vel_ref))

def message_handling_vel_ref(client, userdata, msg):
    global vel_ref, vel_ref_target, ramping, last_ramp_time
    payload = msg.payload.decode().strip()
    if payload == "":
        return
    vel_ref_target = float(payload)

    if ramping:
        # An in-progress ramp will re-aim at the new target on the next ramp_step.
        return

    if start_stop == 1 and abs(vel_ref_target - vel_ref) > RAMP_TRIGGER_GAP:
        ramping = True
        last_ramp_time = time.time()
    else:
        vel_ref = vel_ref_target
        publish_vel_ref_sent()
        sendData()

def message_handling_start_stop(client, userdata, msg):
    global start_stop, prev_start_stop, vel_ref, ramping, last_ramp_time
    start_stop = 1 if msg.payload.decode() == "true" else 0

    if start_stop == 1 and prev_start_stop == 0:
        if abs(vel_ref_target - vel_measured) > RAMP_TRIGGER_GAP:
            vel_ref = 0.0
            ramping = True
            last_ramp_time = time.time()
        else:
            vel_ref = vel_ref_target
            ramping = False
        publish_vel_ref_sent()
        # Pi-commanded session begins here. The logger ignores rows pushed
        # while no session is open, so we don't lose telemetry by gating
        # session start on this edge.
        logger.start_session()
        logger.log_event("start", f"vel_ref_target={vel_ref_target} mode={control_mode}")
    elif start_stop == 0 and prev_start_stop == 1:
        ramping = False
        logger.log_event("stop", "")
        logger.end_session()
    elif start_stop == 0:
        ramping = False

    prev_start_stop = start_stop
    sendData()

def ramp_step():
    global vel_ref, ramping, last_ramp_time
    if not ramping:
        return
    now = time.time()
    dt = now - last_ramp_time
    if dt < RAMP_STEP_INTERVAL:
        return
    last_ramp_time = now

    step = RAMP_RATE * dt
    diff = vel_ref_target - vel_ref
    if abs(diff) <= step:
        vel_ref = vel_ref_target
        ramping = False
    elif diff > 0:
        vel_ref += step
    else:
        vel_ref -= step
    sendData()
    publish_vel_ref_sent()

message_handling_P_gain        = make_float_handler("P_gain", 0.0, 5.0)
message_handling_I_action      = make_float_handler("I_action", 0.0, 15.0)
message_handling_feed_forward  = make_float_handler("feed_forward")
message_handling_counts_per_rev = make_float_handler("counts_per_rev", 1.0, 1.0e6)
message_handling_pos_ref_deg   = make_float_handler("pos_ref_deg", -36000.0, 36000.0)  # 100 turns either way is plenty
message_handling_pos_P_gain    = make_float_handler("pos_P_gain", 0.0, 10.0)
message_handling_pos_max_vel   = make_float_handler("pos_max_vel", 0.0, 1000.0)
message_handling_pos_max_accel = make_float_handler("pos_max_accel", 0.1, 1000.0)
message_handling_pos_I_action  = make_float_handler("pos_I_action", 0.0, 10.0)
message_handling_pos_D_gain    = make_float_handler("pos_D_gain", 0.0, 100.0)

def message_handling_control_mode(client, userdata, msg):
    """Switch between velocity (0) and position (1) mode. Snaps the new
    mode's target to current state so the spit doesn't lurch on the flip:
      - entering position mode: pos_ref_deg = current angle (spit holds)
      - entering velocity mode: vel_ref = 0 (gentle stop)
    """
    global control_mode, pos_ref_deg, vel_ref, vel_ref_target, ramping
    payload = msg.payload.decode().strip().lower()
    if payload in ("true", "1", "position"):
        new_mode = 1
    elif payload in ("false", "0", "velocity"):
        new_mode = 0
    else:
        return

    if new_mode != control_mode:
        logger.log_event("control_mode", f"{control_mode} -> {new_mode}")
        if new_mode == 1:
            # Snap pos_ref_deg to current spit angle so the controller has
            # error=0 the instant it takes over.
            if counts_per_rev > 0:
                pos_ref_deg = (encoderCount - zero_count) / counts_per_rev * 360.0
                publish_current("pos_ref_deg")
        else:
            # Cancel any in-flight velocity ramp and set the inner loop
            # to coast to zero. The slider on the dashboard won't visually
            # follow this (no retained-state on the slider yet) but the
            # actual motor command is 0 until the user moves it.
            vel_ref = 0.0
            vel_ref_target = 0.0
            ramping = False
            publish_vel_ref_sent()

    control_mode = new_mode
    sendData()
    publish_current("control_mode")

def message_handling_set_home(client, userdata, msg):
    # Any non-empty payload acts as a "press" — captures the current signed
    # encoder count as the new zero so spit_angle_deg reads 0 from here on.
    global zero_count
    if msg.payload.decode().strip() == "":
        return
    zero_count = encoderCount
    logger.log_event("set_home", f"encoder_count={encoderCount}")

def message_handling_direction(client, userdata, msg):
    global direction
    payload = msg.payload.decode().strip().lower()
    if payload in ("true", "1"):
        new_dir = 1
    elif payload in ("false", "0"):
        new_dir = 0
    else:
        return
    if new_dir != direction:
        logger.log_event("direction", f"{direction} -> {new_dir}")
    direction = new_dir
    sendData()

USB_connection_started = False # flag to check USB connection has been made
USB_max_connect_attemps = 20
USB_connect_attemps = 0

receive_client.message_callback_add(receive_vel_ref_topic, message_handling_vel_ref)
receive_client.message_callback_add(receive_startstop_topic, message_handling_start_stop)
receive_client.message_callback_add(receive_p_gain_topic, message_handling_P_gain)
receive_client.message_callback_add(receive_i_action_topic, message_handling_I_action)
receive_client.message_callback_add(receive_feed_forward_topic, message_handling_feed_forward)
receive_client.message_callback_add(receive_direction_topic, message_handling_direction)
receive_client.message_callback_add(receive_counts_per_rev_topic, message_handling_counts_per_rev)
receive_client.message_callback_add(receive_set_home_topic, message_handling_set_home)
receive_client.message_callback_add(receive_save_settings_topic, message_handling_save_settings)
receive_client.message_callback_add(receive_control_mode_topic, message_handling_control_mode)
receive_client.message_callback_add(receive_pos_ref_deg_topic, message_handling_pos_ref_deg)
receive_client.message_callback_add(receive_pos_P_gain_topic, message_handling_pos_P_gain)
receive_client.message_callback_add(receive_pos_max_vel_topic, message_handling_pos_max_vel)
receive_client.message_callback_add(receive_pos_max_accel_topic, message_handling_pos_max_accel)
receive_client.message_callback_add(receive_pos_I_action_topic, message_handling_pos_I_action)
receive_client.message_callback_add(receive_pos_D_gain_topic, message_handling_pos_D_gain)

receive_client.subscribe(receive_topic)

# Load persisted settings (overwrites in-memory defaults if file exists) and
# publish the current state of every dashboard-bound setting as retained MQTT,
# so the UI widgets show real values on connect instead of being blank.
load_settings()
for _var in CURRENT_TOPICS:
    publish_current(_var)

# Start the background data-logging writer thread. Telemetry pushes are
# non-blocking (drop-and-count if the writer falls behind), so this can't
# stall the control loop.
logger.start()

if __name__ == '__main__':
    try:
        while not USB_connection_started and USB_connect_attemps <= USB_max_connect_attemps:
            try:
                link = txfer.SerialTransfer('/dev/ttyACM0')
                link.open()
                USB_connection_started = True
                print("USB connected")
            except:
                print("Retrying USB connection, attempt: ", USB_connect_attemps)
                time.sleep(2)
                USB_connect_attemps += 1
            else:
                break
        
        if not USB_connection_started and USB_connect_attemps >= 1:
            print("Could not connect to Arduino via USB, stopping program.")
            exit(0)
        print("Starting MQTT receive")
        
        time.sleep(2) # allow some time for the Arduino to completely reset
        receive_client.loop_start()
        
        args = sys.argv
        sim_enabled = " "
        if len(args) > 1:
            sim_enabled = sys.argv[1]
            if sim_enabled == "sim":
                print("Simulation mode")
        while True:
            # 1 kHz poll — 20x faster than the Arduino's 50 ms telemetry interval.
            # Without this the loop pegs a CPU core busy-waiting on link.available().
            time.sleep(0.001)
            ramp_step()

            if link.available():
                
                
                if sim_enabled == "sim":
                    vel_measured_tmp = vel_ref + 5 * np.random.rand(1)
                    vel_measured = float(vel_measured_tmp)
                    encoderCount = 10
                else:
                    recSize = 0
                    vel_measured = link.rx_obj(obj_type='f', start_pos=recSize)
                    recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

                    encoderCount = link.rx_obj(obj_type='l', start_pos=recSize)
                    recSize += txfer.STRUCT_FORMAT_LENGTHS['l']

                    u_duty = link.rx_obj(obj_type='B', start_pos=recSize)
                    recSize += txfer.STRUCT_FORMAT_LENGTHS['B']

                    error_integral = link.rx_obj(obj_type='f', start_pos=recSize)
                    recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

                # Accumulate absolute distance travelled so the rounds counter
                # keeps going up regardless of direction. The Arduino's
                # encoderCount is now signed (direction-aware), so a forward
                # then equal reverse sweep would net to zero — undesirable for
                # a cook-time counter. abs(delta) per packet gives total travel.
                if prev_encoder_count is None:
                    prev_encoder_count = encoderCount
                else:
                    total_abs_pulses += abs(encoderCount - prev_encoder_count)
                    prev_encoder_count = encoderCount
                num_rounds = total_abs_pulses / 305

                # Signed angular position relative to the last "Set Home" press.
                # counts_per_rev is live-tunable so the user can calibrate by
                # spinning exactly one full turn after a Set Home and adjusting
                # the constant until the displayed angle reads 360°.
                spit_angle_deg = 0.0
                if counts_per_rev > 0:
                    spit_angle_deg = (encoderCount - zero_count) / counts_per_rev * 360.0
                    send_client.publish(send_spit_angle_topic, str(spit_angle_deg))

                # Push a telemetry row to the background logger. The logger
                # drops rows (and counts them) if the queue fills, so this
                # call never blocks. Only written to disk while a session is
                # open — pushes during idle are harmless no-ops downstream.
                logger.push_telemetry({
                    "vel_ref":        vel_ref,
                    "vel_measured":   vel_measured,
                    "vel_error":      vel_ref - vel_measured,
                    "u_duty":         u_duty,
                    "error_integral": error_integral,
                    "encoder_count":  encoderCount,
                    "spit_angle_deg": spit_angle_deg,
                    "direction":      direction,
                    "control_mode":   control_mode,
                    "pos_ref_deg":    pos_ref_deg,
                    "pos_error_deg":  pos_ref_deg - spit_angle_deg,
                })

                ###################################################################
                # Display the received data
                ###################################################################
                # print('RCVD: {}'.format(testRX.vel_ref))
                # print(' ')

                if isinstance(vel_measured, (float, int)) and not np.isnan(vel_measured):
                    message = str(vel_measured)
                    send_client.publish(send_measured_vel_topic, message)

                if isinstance(num_rounds, (float, int)) and not np.isnan(num_rounds):
                    message = str(num_rounds)
                    send_client.publish(send_measured_num_rounds, message)
                

                ###################################################################
                # Wait for a response and report any errors while receiving packets
                ###################################################################
                    # A negative value for status indicates an error
            elif link.status < 0:
                if link.status == txfer.Status.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                    logger.note_crc_error()
                elif link.status == txfer.Status.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                    logger.note_crc_error()
                elif link.status == txfer.Status.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                    logger.note_crc_error()
                else:
                    print('ERROR: {}'.format(link.status.name))
                    logger.note_crc_error()
            
            ###################################################################
            # Parse response float
            ###################################################################
            # rec_float_ = link.rx_obj(obj_type=type(float_),
            #                          obj_byte_size=2)
            
         
    
    except KeyboardInterrupt:
        try:
            link.close()
        except:
            pass
    
    except:
        import traceback
        traceback.print_exc()
        
        try:
            link.close()
        except:
            pass

    finally:
        print("Disconnecting from the MQTT broker")
        logger.stop()
        send_client.disconnect()
        receive_client.disconnect()