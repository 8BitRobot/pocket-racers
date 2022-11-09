# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.vflip = True
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640, 480))

camera.start_preview()
print("Running")
time.sleep(2)

#camera.start_recording("10_sec_vid.h264")
#time.sleep(10)
#camera.stop_recording()
print("Recording Started")

video = np.empty(shape=(30, 480, 640, 3))
for fnum, frame in enumerate(camera.capture_continuous(rawCapture, format='bgr', use_video_port=True)):
    print(frame.array.shape)
    video[fnum] = frame.array.astype("uint8")
    #cv2.imshow("Frame", frame.array)
    #key = cv2.waitKey(1)
    rawCapture.truncate(0)
    if fnum >= 29:
        break

np.save("vid_as_array2", video)

