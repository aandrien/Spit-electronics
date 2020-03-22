import wiringpi as wpi
import time

wpi.wiringPiSetupGpio()

In1 = 15

wpi.pinMode(In1,1)

wpi.digitalWrite(In1,1)
time.sleep(1)
print "Motor On"
wpi.digitalWrite(In1,0)
time.sleep(5)
print "Motor off"
wpi.digitalWrite(In1,1)

motor_max = 100

wpi.pinMode(In1,0)

#GPIO.readall
#p = GPIO.PWM(In1,50) #instance of PWM, portnumber and frequency in Hz
#p.start(0)
#dt = 0.1

#try:
#	while True:
#		#Main code
#		for i in range(motor_max):
#			p.ChangeDutyCycle(i)
#			time.sleep(dt)
#		print "maximum reached"
#		for i in range(motor_max):
#			p.ChangeDutyCycle(motor_max-i)
#			time.sleep(dt)
#		print "minimum reached"
#except KeyboardInterrupt:
#	pass

#p.stop()

#for x in range(6):
#	print "LED on"
#	GPIO.output(15,GPIO.HIGH)
#	time.sleep(0.3)
#	print "LED OFF"
#	GPIO.output(15,GPIO.LOW)
#	time.sleep(0.3)
