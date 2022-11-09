import time
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
import threading
import numpy as np
#import matplotlib.pyplot as plt

current_width = 0
#thread_id = threading.get_ident()
#clock_id = time.pthread_getcpuclockid(thread_id)
#print(thread_id)
#print(clock_id)
#print(time.clock_getres(clock_id))
#print(time.clock_gettime(clock_id))
#print(time.clock_gettime_ns(clock_id))
#print(time.time_ns())
print("starting")

def get_width(pin):
    start = time.time_ns()
    while GPIO.input(pin) == GPIO.HIGH:
        pass
    end = time.time_ns()
    global current_width
    current_width = (end-start)/10**6

inputs = []
pin = 10
GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
#GPIO.add_event_detect(pin, GPIO.RISING)
#GPIO.add_event_callback(pin, get_width)

last_print = 0
while True:
#        time.sleep(1/10000)
#        inputs.append(GPIO.input(pin))
    GPIO.wait_for_edge(pin, GPIO.RISING)
    start = time.time_ns()
    GPIO.wait_for_edge(pin, GPIO.FALLING)
    end = time.time_ns()
    current_width = (end-start)/10**6
#    get_width(pin)
    if time.time() >= last_print+1:
        print(current_width)
        last_print = time.time()
#    print(GPIO.input(pin))
#    if GPIO.event_detected(pin):
#        start = time.time_ns()
#        while GPIO.input(pin) == GPIO.HIGH:
#            pass
#        end = time.time_ns()
#        current_width = (end-start)/10**6
#        print(current_width)
    x = [x*(x-2) for x in range(1000)]
    for i in range(len(x)-1):
        x[i] = x[i+1]
    x[len(x)-1] = 0

#except KeyboardInterrupt:
#    print(len(inputs))
#    plt.scatter(range(len(inputs)), inputs)
#    print("plotted")
#    np.save("inputs.npy", np.array(inputs))
#    print("svaedarray")
#    plt.savefig("inputs.png")
