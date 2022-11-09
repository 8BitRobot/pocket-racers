# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import pickle
import numpy as np
import time
import cv2
import RPi.GPIO as GPIO
import time

pin = 10
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin, GPIO.IN)
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.vflip = True
camera.resolution = (640, 480)
camera.framerate = 15
rawCapture = PiRGBArray(camera, size=(640, 480))

camera.start_preview()
print("Running")
time.sleep(2)

#camera.start_recording("10_sec_vid.h264")
#time.sleep(10)
#camera.stop_recording()
print("Recording Started")
motor_speed = np.empty(30)
video = np.empty(shape=(30, 480, 640, 3))
f_time = time.time_ns()

for fnum, frame in enumerate(camera.capture_continuous(rawCapture, format='bgr', use_video_port=True)):
    new_time = time.time_ns()
    print(new_time-f_time)
    f_time = new_time
    GPIO.wait_for_edge(pin, GPIO.RISING)
    start = time.time_ns()
    GPIO.wait_for_edge(pin, GPIO.FALLING)
    end = time.time_ns()
    motor_speed[fnum] = (end-start)/10**9
#    print(motor_speed[fnum])
#    print(frame.array.shape)
    video[fnum] = frame.array.astype("uint8")
    #cv2.imshow("Frame", frame.array)
    #key = cv2.waitKey(1)
    rawCapture.truncate(0)
    if fnum >= 29:
        break
print("done recording")

np.savez_compressed("savez_compressed_vidAndMotor", video=video, motor=motor_speed)
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
