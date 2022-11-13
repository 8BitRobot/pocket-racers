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
'''
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(7, GPIO.OUT)
pi_pwm = GPIO.PWM(7, 50)
pi_pwm.start(0)

def output_result(dot_count):
    global pi_pwm
    duty_cycles = range(0, 97, 8)
    try:
        pi_pwm.ChangeDutyCycle(duty_cycles[dot_count])
    except:
        pi_pwm.ChangeDutyCycle(0)
'''

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
    if convexity > 0.9:
        instruction = "stop"
    else:
        if mx < middle - 20:
            instruction = "left"
        elif mx > middle + 20:
            instruction = "right"
        else:
            instruction = "straight"
    
    color_img = part2_checkoff(color_img, contours, lc_i, moment, midline, instruction)

    cv2.imshow("Capture", color_img)
    return 0

frame_count = 0
try:
    while True:
        result = vs.read()
        frame_count += 1
        img = cv2.rotate(result, cv2.ROTATE_180)
        if frame_count == 1:
            print(img.shape)
        ''' PART 1 '''

        points = detect_shape(img)

 
        # Uncomment these two lines when getting checked off.

        # if frame_count % 3 == 0:
        #     output_result(len(points))

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
cv2.destroyAllWindows()
vs.stop()
