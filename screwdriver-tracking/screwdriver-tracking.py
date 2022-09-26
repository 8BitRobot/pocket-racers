import cv2
import numpy as np
import sys

capture = cv2.VideoCapture("screwdriver.mp4")

bdp = cv2.SimpleBlobDetector_Params()

# bdp.filterByArea = True
# bdp.minArea = 400
bdp.filterByColor = True
bdp.blobColor = 255

# detector = cv2.SimpleBlobDetector_create(bdp)

frame_count = 1
while capture.isOpened():
    ret, frame = capture.read()
    frame_count += 1

    if ret:
        new_dims = (int(frame.shape[1] * 0.5), int(frame.shape[0] * 0.5))

        downscale = cv2.resize(frame, new_dims)

        hsv = cv2.cvtColor(downscale, cv2.COLOR_RGB2HSV)
        hsv = cv2.GaussianBlur(hsv, (15, 15), cv2.BORDER_DEFAULT)

        mask_upper_bound = (100, 255, 255)
        mask_lower_bound = (90, 111, 92)

        mask = cv2.inRange(hsv, mask_lower_bound, mask_upper_bound)

        kernel = np.ones((3,3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=8)
        mask = cv2.dilate(mask, kernel, iterations=16)
        mask = cv2.erode(mask, kernel, iterations=8)

        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        # keypoints = detector.detect(mask)
        # im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        total_contour_area = 0
        for i in range(len(contours)):
            M = cv2.moments(contours[i])
            if(M['m00'] == 0.0):
                continue
            total_contour_area += cv2.contourArea(contours[i])

        im_with_keypoints = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        result = cv2.putText(im_with_keypoints,
                             str(total_contour_area),
                             (50, 50),
                             cv2.FONT_HERSHEY_SIMPLEX,
                             2,
                             (0,0,255),
                             2,
                             cv2.LINE_AA)
       
        cv2.imshow("Original", downscale)
        cv2.imshow("Result", result)

        k = cv2.waitKey(25)
        if k == ord('q'):
            break
        elif k == ord('p'):
            print(frame_count)
            input()
    else:
        break

capture.release()
cv2.destroyAllWindows()
