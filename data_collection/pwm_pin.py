# import the necessary packages
from picamera.array import PiRGBArray
import numpy as np
import time
import RPi.GPIO as GPIO
import time

class PWMPin:

    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(pin, GPIO.IN)
        self.data = []

    def collect_data_point(self):
        GPIO.wait_for_edge(self.pin, GPIO.RISING)
        start_time = time.time_ns()
        while(GPIO.input(self.pin)):
            pass
        end_time = time.time_ns()
        self.data.append(end_time-start_time)
        return self.data[-1]

    def collect_data_point2(self):
        GPIO.wait_for_edge(self.pin, GPIO.RISING)
        start_time = time.time_ns()
        GPIO.wait_for_edge(self.pin, GPIO.FALLING)
        end_time = time.time_ns()
        self.data.append(end_time-start_time)
        return self.data[-1]

    def get_data(self):
        return np.array(self.data)

    def clear_data(self):
        self.data = []
