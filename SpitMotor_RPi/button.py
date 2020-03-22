import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

Input = 14
Output = 15

GPIO.setup(Input, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

GPIO.setup(Output, GPIO.OUT)
GPIO.output(Output, 0)


try:
	while True:
		print( GPIO.input(Input))

except KeyboardInterrupt:
	GPIO.cleanup()
