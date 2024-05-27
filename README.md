# Spit-electronics

## Architecture
The SpitTronics uses an Arduino Leonardo to send PWM pulses to an H-bridge, which allows for setting the voltage the motor receives. The Arduino receives encoder pulses on one of its digital pins.

The Arduino in turn is connected to a Raspberry Pi 4 via USB. Using the [SerialTransfer library](https://github.com/PowerBroker2/SerialTransfer/tree/master) on the Arduino side and the corresponding Python code ([pySerial](https://github.com/PowerBroker2/pySerialTransfer/tree/master)), data is send back and forth between the RPi and the Arduino.

The RPi uses MQTT to make the data that is received in the Python script available anywhere on the RPi. Node-red then subscribes to these topics and the data is visualized in a node-red-dashboard.

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

For example on usage, see the [Python script in this repository](RPi/Spit_RPi_Arduino_V3.py). For more information and explanation, see https://medium.com/@potekh.anastasia/a-beginners-guide-to-mqtt-understanding-mqtt-mosquitto-broker-and-paho-python-mqtt-client-990822274923.

### Installing Node-Red
It is best to follow the instructions from the [node-red source](https://nodered.org/docs/getting-started/raspberrypi#installing-and-upgrading-node-red). Be sure to make it [autostart on boot](https://nodered.org/docs/getting-started/raspberrypi#autostart-on-boot). The editor can then be accessed in a [webbrowser](https://nodered.org/docs/getting-started/raspberrypi#opening-the-editor).

### Autostart Python script on boot
Use `crontab` to make the RPi run the [start script](RPi/bash_scripts/start_spit.sh) on boot. First open `crontab`:

`$ crontab -e`

Just select `nano` as the editor. Then add the `start_spit.sh` to the boot command:

`@reboot pathToRepo/RPi/bash_scripts/start_spit.sh`

where `pathToRepo` is the path to this repository.