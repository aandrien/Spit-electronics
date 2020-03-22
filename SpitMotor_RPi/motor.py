import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

In1  = 15
In2  = 18

GPIO.setup(In1,GPIO.OUT)

print "Turning motor on"
GPIO.output(In1,GPIO.HIGH)

sleep(1)

print "Stopping motor"
GPIO.output(In1,GPIO.LOW)

sleep(1)
GPIO.cleanup()
