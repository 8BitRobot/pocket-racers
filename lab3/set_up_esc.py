import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(7, GPIO.OUT)

pwm = GPIO.PWM(7, 57)
pwm.start(float(input()))

print(float(input()))

try:
    while True:
        pwm.ChangeDutyCycle(float(input()))
except:
    pass

pwm.stop()
GPIO.cleanup()
