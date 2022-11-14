import cv2
import numpy as np
import RPi.GPIO as GPIO
from threading import Thread
from multiprocessing import Process, Pipe
from queue import Queue
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

class PiVideoStream:
    def __init__(self, resolution=(640, 480), framerate=32):
        self.process = None
        self.resolution = resolution
        self.framerate = framerate


    def start(self):
        pipe_in, self.pipe_out = Pipe()
        # start the thread to read frames from the video stream
        self.process = Process(target=self.update, args=(pipe_in,), daemon=True)
        self.process.start()
        return self
    

    def update(self, pipe_in):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = self.resolution
        self.camera.framerate = self.framerate
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False
        self.rawCapture = PiRGBArray(self.camera, size=self.resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
            format="bgr", use_video_port=True)
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            pipe_in.send([self.frame])
            self.rawCapture.truncate(0)
            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                self.process.join()
                return

    def read(self):
        # return the frame most recently read
        if self.pipe_out.poll():
            return self.pipe_out.recv()[0]
        else:
            return None


    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


print("[INFO] sampling MULTIPROCESSED frames from `picamera` module...")
vs = PiVideoStream(resolution=(640,480)).start()
time.sleep(2.0)

def part2_checkoff(img, contours, contour_index, moment, midline, instruction):
    img = cv2.drawContours(img, contours, contour_index, (0,0,255), 3)
    img = cv2.circle(img, (moment[0], moment[1]), 3, (0,255,0), 3)
    
    img = cv2.line(img,
                   midline[0],
                   midline[1],
                   (0, 0, 255),
                   3)
    
    img = cv2.putText(img,
                      instruction,
                      (50, 50),
                      cv2.FONT_HERSHEY_SIMPLEX,
                      2,
                      (0,0,255),
                      2,
                      cv2.LINE_AA)

    return img

def detect_shape(color_img):
    '''
    PART 1
    Isolate (but do not detect) the arrow/stop sign using image filtering techniques.

    Checkoffs: None for this part!
    '''
    img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    img = cv2.GaussianBlur(img,ksize=(5, 5), sigmaX=0)
    ret, img = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY)
    img[0:100] = 0
    
    h, w = img.shape[:2]
    mask = np.zeros((h + 2, w + 2), np.uint8)
    cv2.floodFill(img, mask, (0, 0), 255)

    cv2.imshow("img", img)
    '''
    END OF PART 1
    '''

    color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return
    
    '''
    PART 2
    1. Identify the contour with the largest area.
    2. Find the centroid of that contour.
    3. Determine whether the contour represents a stop sign or an arrow.
    4. Use the format_contour() helper function to produce the formatted image.

    Checkoffs: Send this formatted image to your leads in your team's Discord group chat.
    '''

    lc_i = -1 # largest contour index
    lc_area = 0 # largest contour area

    for i in range(len(contours)):
        hierarchy_entry = hierarchy[0][i]
        if hierarchy_entry[3] == -1:
            continue
        if (area := cv2.contourArea(contours[i])) > lc_area:
            lc_i = i
            lc_area = area

    print(lc_area)

    lc_moment = cv2.moments(contours[lc_i])
    try:
        mx = int(lc_moment['m10'] / lc_moment['m00']) # X-coord of contour's moment of inertia
        my = int(lc_moment['m01'] / lc_moment['m00']) # Y-coord of contour's moment of inertia
    except:
        return

    moment = (mx, my)
    
    x,y,w,h = cv2.boundingRect(contours[lc_i])
    middle = x + w//2
    midline = [(middle, y), (middle, y+h)]
  
    convexity = lc_area / cv2.contourArea(cv2.convexHull(contours[lc_i]))
    if (lc_area < 10000  and lc_area > 3000) or \
       (lc_area < 30000 and lc_area > 18000):
        if convexity > 0.9:
            instruction = "stop"
        else:
            if mx < middle - 20:
                instruction = "left"
            elif mx > middle + 20:
                instruction = "right"
            else:
                instruction = "straight"
    else:
        instruction = "idle"
    
    color_img = part2_checkoff(color_img, contours, lc_i, moment, midline, instruction)

    cv2.imshow("Capture", color_img)
    return instruction


GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(37, GPIO.OUT)

pwm_m = GPIO.PWM(7, 57)
pwm_s = GPIO.PWM(37, 57)

pwm_s.start(7.5)
pwm_m.start(8)
print("arming")
time.sleep(3)
print("armed")

pwm_m.ChangeDutyCycle(5.7)

frame_count = 0
left_count = 0
right_count = 0
last_instruction = None

try:
    while True:
        if vs.pipe_out.poll():
            result = vs.read()
            frame_count += 1
            img = cv2.rotate(result, cv2.ROTATE_180)
            if frame_count == 1:
                print(img.shape)

            instruction = detect_shape(img)

            if instruction == "idle":
                instruction = last_instruction

            if instruction == "left":
                left_count += 1
                if left_count == 3:
                    pwm_s.ChangeDutyCycle(10.3)
                    left_count = 0
            elif instruction == "right":
                right_count += 1
                if right_count == 3:
                    pwm_s.ChangeDutyCycle(5.7)
                    right_count = 0
            elif instruction == "stop":
                pwm_s.ChangeDutyCycle(7.5)
                pwm_m.ChangeDutyCycle(8)
            elif instruction == "straight":
                pwm_s.ChangeDutyCycle(7.5)
                pwm_m.ChangeDutyCycle(5.7)

            last_instruction = instruction

            k = cv2.waitKey(3)
            if k == ord('q'):
                # If you press 'q' in the OpenCV window, the program will stop running.
                break
            elif k == ord('p'):
                # If you press 'p', the camera feed will be paused until you press
                # <Enter> in the terminal.
                input()
except KeyboardInterrupt:
    pass

# Clean-up: stop running the camera and close any OpenCV windows
pwm_m.stop()
pwm_s.stop()
GPIO.cleanup()
cv2.destroyAllWindows()
vs.stop()
