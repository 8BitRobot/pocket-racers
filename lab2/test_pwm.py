import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(7, GPIO.OUT)
p = GPIO.PWM(7, 50)
p.start(30)

try:
    while True:
        c = float(input("signal> "))
        p.ChangeDutyCycle(c)
except:
    pass

p.stop()
GPIO.cleanup()
