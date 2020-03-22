import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
In1 = 15
motor_max = 100
GPIO.setup(In1,GPIO.OUT)
p = GPIO.PWM(In1,50) #instance of PWM, portnumber and frequency in Hz
p.start(0)
dt = 0.1

try:
	while True:
		#Main code
		for i in range(motor_max):
			p.ChangeDutyCycle(i)
			time.sleep(dt)
		print "maximum reached"
		for i in range(motor_max):
			p.ChangeDutyCycle(motor_max-i)
			time.sleep(dt)
		print "minimum reached"
except KeyboardInterrupt:
	pass

p.stop()

#for x in range(6):
#	print "LED on"
#	GPIO.output(15,GPIO.HIGH)
#	time.sleep(0.3)
#	print "LED OFF"
#	GPIO.output(15,GPIO.LOW)
#	time.sleep(0.3)
GPIO.cleanup()
