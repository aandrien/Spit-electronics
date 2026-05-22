import sys
import time
import paho.mqtt.client as paho
import numpy as np
from pySerialTransfer import pySerialTransfer as txfer

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
receive_topic = "mqtt/rpi/rx/#"

send_client = paho.Client()
receive_client = paho.Client()


vel_measured = 0.0
encoderCount = 0
prev_encoder_count = None  # None until the first telemetry packet arrives
total_abs_pulses = 0       # cumulative |delta| — counts revolutions in both directions equally

# Signed-position tracking — derived purely on the Pi from the signed encoderCount.
# counts_per_rev is live-tunable from Node-Red (default 4270 = 305 * 14 from the
# pre-existing num_rounds + dashboard divisor). zero_count is captured when the
# user presses "Set Home"; spit_angle_deg = (encoderCount - zero_count) / counts_per_rev * 360.
counts_per_rev = 4270.0
zero_count = 0


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

    link.send(sendSize)

def make_float_handler(var_name, lo=None, hi=None):
    def handler(client, userdata, msg):
        payload = msg.payload.decode().strip()
        if payload == "":
            return
        value = float(payload)
        if lo is not None:
            value = min(max(value, lo), hi)
        globals()[var_name] = value
        sendData()
    return handler

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

def message_handling_set_home(client, userdata, msg):
    # Any non-empty payload acts as a "press" — captures the current signed
    # encoder count as the new zero so spit_angle_deg reads 0 from here on.
    global zero_count
    if msg.payload.decode().strip() == "":
        return
    zero_count = encoderCount

def message_handling_direction(client, userdata, msg):
    global direction
    payload = msg.payload.decode().strip().lower()
    if payload in ("true", "1"):
        direction = 1
    elif payload in ("false", "0"):
        direction = 0
    else:
        return
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

receive_client.subscribe(receive_topic)

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
                if counts_per_rev > 0:
                    spit_angle_deg = (encoderCount - zero_count) / counts_per_rev * 360.0
                    send_client.publish(send_spit_angle_topic, str(spit_angle_deg))

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
                elif link.status == txfer.Status.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif link.status == txfer.Status.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(link.status.name))
            
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
        send_client.disconnect()
        receive_client.disconnect()