import wiringpi as wpi
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

Input = 4

GPIO.setup(Input, GPIO.IN, pull_up_down = GPIO.PUD_UP)

## MCP3008
# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
 
# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)
 
# create the mcp object
mcp = MCP.MCP3008(spi, cs)
 
# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P0)
chan_max = 65472

# create an analog input channel on pin 1
chan1 = AnalogIn(mcp,MCP.P1)

## Motor
wpi.wiringPiSetupGpio()

In1 = 12	#GPIO port for In1 on H-bridge board
In2 = 13	#GPIO port for In1 on H-bridge board

motor_max = 1024
dt = 0.1

wpi.pinMode(In1,2) 	#Set GPIO port to hardware PWM mode
wpi.pinMode(In2,2) 	#Set GPIO port to hardware PWM mode

wpi.pwmWrite(In1,motor_max) #Initialize to 1024, which means motor stopped
wpi.pwmWrite(In2,motor_max) #Initialize to 1024, which means motor stopped

## Main code
try:
	while True:
		#Main code
		if(chan.value < chan_max-10):
			motor_com = int(motor_max/chan_max*(chan_max-chan.value))
			wpi.pwmWrite(In1,motor_com)
		time.sleep(dt)
#		print('Raw ADC Value: ', chan.value)
#		print('ADC Voltage: ' + str(chan.voltage) + 'V')
#		print('Commmand to motor: ',motor_com)
		print('Motor PWM ',(motor_max-motor_com)/motor_max*100,'%')
		print("Input", GPIO.input(Input))
		print("Input Analog",chan1.voltage)

except KeyboardInterrupt:
	pass


wpi.pwmWrite(In1,motor_max) #Switch motor off
wpi.pwmWrite(In2,motor_max) #Switch motor off

wpi.pinMode(In1,0)	#Set pin back to input
wpi.pinMode(In2,0)	#Set pin back to input

GPIO.cleanup()

