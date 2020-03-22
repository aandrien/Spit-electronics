import wiringpi as wpi
import time

wpi.wiringPiSetupGpio()

In1 = 12	#GPIO port for In1 on H-bridge board
In2 = 13	#GPIO port for In1 on H-bridge board

motor_max = 1024
dt = 0.01

wpi.pinMode(In1,2) 	#Set GPIO port to hardware PWM mode
wpi.pinMode(In2,2) 	#Set GPIO port to hardware PWM mode

wpi.pwmWrite(In1,motor_max) #Initialize to 1024, which means motor stopped
wpi.pwmWrite(In2,motor_max) #Initialize to 1024, which means motor stopped



try:
	while True:
		#Main code
		for i in range(motor_max):
			wpi.pwmWrite(In1,motor_max-i)
			time.sleep(dt)
		print("maximum reached")
		for i in range(motor_max):
			wpi.pwmWrite(In1,i)
			time.sleep(dt)
		print("minimum reached")
except KeyboardInterrupt:
	pass


wpi.pwmWrite(In1,motor_max) #Switch motor off
wpi.pwmWrite(In2,motor_max) #Switch motor off

wpi.pinMode(In1,0)	#Set pin back to input
wpi.pinMode(In2,0)	#Set pin back to input
