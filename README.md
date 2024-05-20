# Spit-electronics
Alle files voor het elektronische deel van het spit

## Architecture
The SpitTronics uses an Arduino Leonardo to send PWM pulses to an H-bridge, which allows for setting the voltage the motor receives. The Arduino receives encoder pulses on one of its digital pins.

The Arduino in turn is connected to a Raspberry Pi 4 via USB. Using the [SerialTransfer library](https://github.com/PowerBroker2/SerialTransfer/tree/master) on the Arduino side and the corresponding Python code ([pySerial](https://github.com/PowerBroker2/pySerialTransfer/tree/master)), data is send back and forth between the RPi and the Arduino.

The RPi uses MQTT to make the data that is received in the Python script available anywhere on the RPi. Node-red then subscribes to these topics and the data is visualized in a node-red-dashboard.