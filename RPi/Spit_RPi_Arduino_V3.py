import sys
import time
import paho.mqtt.client as paho
import numpy as np
from pySerialTransfer import pySerialTransfer as txfer

hostname = "localhost"
broker_port = 1883
send_measured_num_rounds = "mqtt/rpi/num_rounds"
send_measured_vel_topic = "mqtt/rpi/vel_measured"
receive_vel_ref_topic = "mqtt/rpi/rx/vel_ref"
receive_startstop_topic = "mqtt/rpi/rx/start_stop"
receive_topic = "mqtt/rpi/rx/#"

send_client = paho.Client()
receive_client = paho.Client()



vel_measured = 0.0
encoderCount = 0


if send_client.connect("localhost", broker_port, 60) != 0:
    print("Couldn't connect to the mqtt broker")
    sys.exit(1)

if receive_client.connect("localhost", broker_port, 60) != 0:
    print("Couldn't connect to the mqtt broker")
    sys.exit(1)

message = str([])
send_client.publish(send_measured_vel_topic, message)

start_stop = 0
vel_ref = 0.0

def sendData():
    sendSize = 0

    ###################################################################
    # Send vel_ref
    ###################################################################
    sendSize = link.tx_obj(vel_ref, start_pos=sendSize)

    ###################################################################
    # Send start/stop command
    ###################################################################
    sendSize = link.tx_obj(start_stop, start_pos=sendSize)

    link.send(sendSize)

def message_handling_start_stop(client, userdata, msg):
    # Define global variable to be able to modify it in function
    global start_stop

    # Decode payload coming from MQTT node
    if msg.payload.decode() == "true":
        start_stop = 1
    else:
        start_stop = 0

    # print("startstop: ", start_stop)

    # Send all data
    sendData()


def message_handling_vel_ref(client, userdata, msg):
    # Define global variable to be able to modify it in function
    global vel_ref

    # Decode payload coming from MQTT node
    vel_ref = float(msg.payload.decode())

    # Send all data
    sendData()

USB_connection_started = False

receive_client.message_callback_add(receive_vel_ref_topic, message_handling_vel_ref)
receive_client.message_callback_add(receive_startstop_topic, message_handling_start_stop)
receive_client.subscribe(receive_topic)

if __name__ == '__main__':
    try:
        while ~USB_connection_started:
            try:
                link = txfer.SerialTransfer('/dev/ttyACM0')
                link.open()
                USB_connection_started = True
                print("USB connected")
            except:
                print("Retrying USB connection")
                time.sleep(2)
            else:
                break

        print("Starting MQTT receive")
        
        time.sleep(2) # allow some time for the Arduino to completely reset
        receive_client.loop_start()

        

        while True:
            ###################################################################
            # Transmit all the data to send in a single packet
            ###################################################################
            
            # send_size = 0
            ###################################################################
            # Receive a float
            ###################################################################
            if link.available():
                recSize = 0
                
                vel_measured = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

                encoderCount = link.rx_obj(obj_type='L', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['L']

                # encoderCount += 50
                
                num_rounds = encoderCount/305

                # print("vel measured: ", vel_measured)
                # print("encoder count: ", encoderCount)

                # print('number rondjes = 302 307')
                ###################################################################
                # Display the received data
                ###################################################################
                # print('RCVD: {}'.format(testRX.vel_ref))
                # print(' ')

                message = str(vel_measured)
                send_client.publish(send_measured_vel_topic, message)

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