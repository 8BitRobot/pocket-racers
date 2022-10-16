import cv2
import numpy as np

capture = cv2.VideoCapture(0) # Read camera input

while capture.isOpened():
    ret, frame = capture.read()
    if ret: # If the camera read is successful, process the image
        result = cv2.rotate(frame, cv2.ROTATE_180)

        '''
        YOUR CODE GOES HERE
        '''

        # Display the processed image with window title "Capture"
        cv2.imshow("Capture", result)

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
