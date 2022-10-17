import cv2
import numpy as np
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)
pi_pwm = GPIO.PWM(7, 50)
pi_pwm.start(0)

def output_result(dot_count):
    duty_cycles = range(0, 97, 8)
    try:
        pi_pwm.ChangeDutyCycle(0)
    except:
        pi_pwm.ChangeDutyCycle(0)

bdp = cv2.SimpleBlobDetector_Params()

bdp.filterByArea = False
bdp.filterByConvexity = False
bdp.filterByCircularity = False
bdp.filterByInertia = False
bdp.filterByColor = True
bdp.blobColor = 255

detector = cv2.SimpleBlobDetector_create(bdp)

capture = cv2.VideoCapture(0) # Read camera input

frame_count = 0
while capture.isOpened():
    ret, frame = capture.read()
    if ret: # If the camera read is successful, process the image
        frame += 1
        result = cv2.rotate(frame, cv2.ROTATE_180)

        ''' PART 1 '''

        ''' GENERAL FILTERING '''

        result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        result = cv2.GaussianBlur(result, (13, 13), cv2.BORDER_DEFAULT)
        og = result

        ''' WHITE DICE '''
        result = cv2.adaptiveThreshold(result, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 35, 32)
        result = np.bitwise_not(result)
        result[0:100, 0:640] = 0
        # result = cv2.erode(result, np.ones((2,2), np.uint8), iterations=3)

        points = detector.detect(result)

        # result = cv2.drawKeypoints(result, points, 0, (0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
        # result = cv2.putText(result, str(len(points)), (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)

        # Display the processed image with window title "Capture"
        # cv2.imshow("Original", og)
        # cv2.imshow("Capture", result)

        if frame_count % 3 == 0:
            output_result(len(points))

        k = cv2.waitKey(25) # Allow keyboard inputs in the OpenCV window
        if k == ord('q'):
            # If you press 'q' in the OpenCV window, the program will stop running.
            break
        elif k == ord('p'):
            # If you press 'p', the camera feed will be paused until you press
            # <Enter> in the terminal.
            input()
    else: # If the camera read is unsuccessful, stop the program
        break

# Clean-up: stop running the camera and close any OpenCV windows
capture.release()
cv2.destroyAllWindows()
