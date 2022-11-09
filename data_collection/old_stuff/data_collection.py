# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import pickle
import numpy as np
import time
import cv2
import time
from pwm_pin import PWMPin
import sys
import os
from system_monitor import monitor_system
from multiprocessing import Pipe, Process

RES = (320, 240)
TOTAL_FRAMES = 100

motor = PWMPin(10)
servo = PWMPin(10)

camera = PiCamera()
camera.vflip = True
camera.resolution = RES
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=RES)

camera.start_preview()

pipe_out, pipe_in = Pipe()
monitor = Process(target=monitor_system, args=(pipe_in, os.getpid()), daemon=True)
monitor.start()

print("Running")
time.sleep(2)

print("Recording Started")
video = np.empty(shape=(TOTAL_FRAMES, RES[1], RES[0], 3))
f_time = time.time_ns()

for fnum, frame in enumerate(camera.capture_continuous(rawCapture, format='bgr', use_video_port=True)):

#    motor.collect_data_point()
#    servo.collect_data_point()

    video[fnum] = frame.array.astype("uint8")
    rawCapture.truncate(0)
    new_time = time.time_ns()
    if(pipe_out.poll()):
        print("Period: " + str((new_time-f_time)/10**9) )
        vitals = pipe_out.recv()
        print("memory percent: " + str(vitals[0]))
        print("bytes storage available: " + str(vitals[1]) )
    f_time = new_time
    if fnum >= TOTAL_FRAMES-1:
        break
print("done recording")

if(len(sys.argv) == 1):
    name = "data/driving_data_1"
    while os.path.exists(name+".npz"):
        name = name[:-1]+str(int(name[-1])+1)
else:
    name = sys.argv[1]
print(f"saving to {name}.npz")
np.savez_compressed(name, video=video, motor=motor.get_data(), servo=servo.get_data())

monitor.join()
print("done!")

#data = {}
#data["video"] = video
#data["motor"] = motor_speed
#f = open("vid_and_motor.pkl", "wb")
#print("dumpt started")
#pickle.dump(data, f)
#f.close()
#np.save("vid_as_array3", video)
#np.save("motor_speed3", motor_speed)
