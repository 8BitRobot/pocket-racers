import cv2
import numpy as np
import RPi.GPIO as GPIO
from threading import Thread
from queue import Queue
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

class PiVideoStream:
    def __init__(self, resolution=(640, 480), framerate=32):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
            format="bgr", use_video_port=True)
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False


    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self
    

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)
            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def read(self):
        # return the frame most recently read
        return self.frame


    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


print("[INFO] sampling THREADED frames from `picamera` module...")
vs = PiVideoStream(resolution=(640,480)).start()
time.sleep(2.0)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(7, GPIO.OUT)
pi_pwm = GPIO.PWM(7, 50)
pi_pwm.start(30)

def output_result(dot_count):
    global pi_pwm
    duty_cycles = range(0, 97, 8)
    try:
        pi_pwm.ChangeDutyCycle(duty_cycles[dot_count])
    except:
        pi_pwm.ChangeDutyCycle(0)


def white_dice(result):
    result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    result = cv2.GaussianBlur(result, (13, 13), cv2.BORDER_DEFAULT)
    result = cv2.adaptiveThreshold(result, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 35, 32)
    result = np.bitwise_not(result)
    result[0:100, 0:640] = 0
    return result


def black_dice(result):
    result = cv2.cvtColor(result, cv2.COLOR_BGR2HLS)
    lowerBound = (0, 0, 0)
    upperBound = (255, 189, 255)
    mask = cv2.inRange(result, lowerBound, upperBound)
    mask = cv2.erode(mask, np.ones((7,7), dtype=np.uint8), 3)
    mask = cv2.erode(mask, np.ones((7,7), dtype=np.uint8), 3)
    mask = cv2.erode(mask, np.ones((7,7), dtype=np.uint8), 3)
    cv2.imshow("Mask", mask)
    mask[0:100, 0:640] = 255
    result = cv2.cvtColor(cv2.cvtColor(result, cv2.COLOR_HLS2BGR), cv2.COLOR_BGR2GRAY)
    result = cv2.GaussianBlur(result, (13, 13), cv2.BORDER_DEFAULT)
    result = np.bitwise_and(mask, result)
    result = cv2.adaptiveThreshold(result, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 35, -20)
    return result

def red_blue_dice(result):
    result = cv2.GaussianBlur(result, (13, 13), cv2.BORDER_DEFAULT)
    result = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
    lowerBoundRed1 = (0, 0, 0)
    upperBoundRed1 = (40, 255, 200)
    lowerBoundRed2 = (215, 0, 0)
    upperBoundRed2 = (255, 255, 200)
    mask = np.bitwise_or(cv2.inRange(result, lowerBoundRed1, upperBoundRed1),
                         cv2.inRange(result, lowerBoundRed2, upperBoundRed2))
    return cv2.bitwise_and(result, result, mask=mask)

bdp_white = cv2.SimpleBlobDetector_Params()
bdp_white.filterByArea = False
bdp_white.filterByConvexity = False
bdp_white.filterByCircularity = False
bdp_white.filterByInertia = False
bdp_white.filterByColor = True
bdp_white.blobColor = 255
detector_white = cv2.SimpleBlobDetector_create(bdp_white)

bdp_black = cv2.SimpleBlobDetector_Params()
bdp_black.filterByArea = False
bdp_black.filterByConvexity = False
bdp_black.filterByCircularity = True
bdp_black.minCircularity = 0.75
bdp_black.filterByInertia = False
bdp_black.filterByColor = True
bdp_black.blobColor = 255
detector_black = cv2.SimpleBlobDetector_create(bdp_black)

frame_count = 0

try:
    while True:
        result = vs.read()
        frame_count += 1
        result = cv2.rotate(result, cv2.ROTATE_180)

        ''' PART 1 '''

        ''' GENERAL FILTERING '''

        og = result

        ''' WHITE DIE '''
        # result = white_dice(result)
        # points = detector_white.detect(result)

        ''' BLACK DIE '''
        # result = black_dice(result)
        # points = detector_black.detect(result)

        # result = cv2.drawKeypoints(result, points, 0, (0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
        # result = cv2.putText(result, str(len(points)), (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)

        # Display the processed image with window title "Capture"
        # cv2.imshow("Original", og)

        ''' RED DIE '''
        result = red_blue_dice(result)

        # cv2.imshow("OG", og)
        cv2.imshow("Capture", result)
        cv2.waitKey(3)
        
        # if frame_count % 3 == 0:
        #     output_result(len(points))

        # k = cv2.waitKey(25) # Allow keyboard inputs in the OpenCV window
        # if k == ord('q'):
        #     # If you press 'q' in the OpenCV window, the program will stop running.
        #     break
        # elif k == ord('p'):
        #     # If you press 'p', the camera feed will be paused until you press
        #     # <Enter> in the terminal.
        #     input()
        # else: # If the camera read is unsuccessful, stop the program
        #     break
except KeyboardInterrupt:
    pass

# Clean-up: stop running the camera and close any OpenCV windows
cv2.destroyAllWindows()
vs.stop()
