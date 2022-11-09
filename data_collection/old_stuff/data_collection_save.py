# import the necessary packages
from pickletools import uint8
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

LINE_UP = '\033[1A'
LINE_CLEAR = '\x1b[2K'
RES = (320, 240)
VIDEO_LENGTH = 60
FRAMERATE = 10
TOTAL_FRAMES = VIDEO_LENGTH * FRAMERATE

motor = PWMPin(10)
servo = PWMPin(3)

camera = PiCamera()
camera.vflip = True
camera.resolution = RES
camera.framerate = FRAMERATE
rawCapture = PiRGBArray(camera, size=RES)

camera.start_preview()

pipe_out, pipe_in = Pipe()
monitor = Process(target=monitor_system, args=(pipe_in, os.getpid()), daemon=True)
monitor.start()
lines = 0

points_taken = 0
print("Running")
time.sleep(2)

low_storage = False
print("Recording Started")
video = np.empty(shape=(TOTAL_FRAMES, RES[1], RES[0], 3), dtype="uint8")
f_time = time.time_ns()
while True:
    for fnum, frame in enumerate(camera.capture_continuous(rawCapture, format='bgr', use_video_port=True)):
        motor.collect_data_point()
        servo.collect_data_point()

        video[fnum] = frame.array.astype("uint8")
        rawCapture.truncate(0)
        new_time = time.time_ns()

        if(pipe_out.poll()):
            for i in range(lines):
                print(LINE_UP, end=LINE_CLEAR)
            lines = 0
            print(f"current video progress: {fnum}/{TOTAL_FRAMES}")
            print("Period: " + str((new_time-f_time)/10**9) + " expected: " + str(1/FRAMERATE))
            lines += 2
            vitals = pipe_out.recv()
            mem_perc = vitals[0]
            storage = vitals[1]
            print(f"storage: {round(storage/1048576, 2)}MB \npercent memory used: {round(mem_perc,2)}")
            lines += 2
            if storage < 2000000000:
                print("Warning, low storage. finish last run")
                lines += 1
                if storage < 1158782464:
                    print("storage too low")
                    camera.close()
                    exit()
        f_time = new_time
        if fnum >= TOTAL_FRAMES-1:
            if(len(sys.argv) == 1):
                name = "data/driving_data_1"
                while os.path.exists(name):
                    name = name[:-1]+str(int(name[-1])+1)
            else:
                name = f"data/{sys.argv[1]}"
                while os.path.exists(name):
                    try:
                        name = name[:-1]+str(int(name[-1])+1)
                    except:
                        name = name + "_1"
            print(f"datapoint taken : saving to {name}.npy")
            lines += 1
            os.mkdir(name)
            # np.savez_compressed(name+"/zipped_data", video=video, motor=motor.get_data(), servo=servo.get_data())
            np.save(name+"/video", video)
            np.save(name+"/motor", motor.get_data())
            np.save(name+"/servo", servo.get_data())
            print("starting next datapoint collection")
            lines += 1
            motor.clear_data()
            servo.clear_data()
            del video
            video = np.empty(shape=(TOTAL_FRAMES, RES[1], RES[0], 3), dtype="uint8")
            if storage < 2000000000:
                print("Storage low, terminating program")
                camera.close()
                exit()
            while pipe_out.poll():
                pipe_out.recv()
            break
