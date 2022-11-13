'''
Use this program when you want to set up your ESC for the first time.

The program assumes you've connected your ESC's PWM input to BOARD pin 7.
Once your ESC is armed, as indicated by 3 beeps, you won't need to touch
this program again.
'''

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(7, GPIO.OUT)

pwm_m = GPIO.PWM(7, 57)

value = input()
pwm_m.start(float(value))

try:
    while True:
        value = input()
        pwm_m.ChangeDutyCycle(float(value))
except:
    pass

pwm_m.stop()
GPIO.cleanup()
