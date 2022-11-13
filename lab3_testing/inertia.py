import cv2
import numpy as np
import sys

capture = cv2.imread(sys.argv[1])

bdp = cv2.SimpleBlobDetector_Params()

# bdp.filterByArea = True
# bdp.minArea = 400
bdp.filterByColor = True
bdp.blobColor = 255

# detector = cv2.SimpleBlobDetector_create(bdp)

frame_count = 1

ret = True
frame = capture
frame_count += 1

if ret:
    new_dims = (int(frame.shape[1] * 0.2), int(frame.shape[0] * 0.2))

    downscale = cv2.resize(frame, new_dims)

    gray = cv2.cvtColor(downscale, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (15,15), cv2.BORDER_DEFAULT)
    
    mask = cv2.adaptiveThreshold(gray, maxValue=255, adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C, thresholdType=cv2.THRESH_BINARY_INV, blockSize=101, C=40)

    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # keypoints = detector.detect(mask)
    # im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    im_with_keypoints = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # print(im_with_keypoints.shape)

    largest_contour = -1
    largest_contour_area = 0

    for i in range(len(contours)):
        M = cv2.moments(contours[i])
        if(M['m00'] == 0.0):
            continue
        if (area := cv2.contourArea(contours[i])) > largest_contour_area:
            largest_contour = i
            largest_contour_area = area
    
    largest_contour_moment = cv2.moments(contours[largest_contour])
    moment_x = int(largest_contour_moment['m10'] / largest_contour_moment['m00'])
    moment_y = int(largest_contour_moment['m01'] / largest_contour_moment['m00']) # centroid of contour ("blob")

    im_with_keypoints = cv2.drawContours(im_with_keypoints, contours, largest_contour, (0,0,255), 3)
    cv2.circle(im_with_keypoints, (moment_x, moment_y), 3, (0,255,0), 3)
    
    x,y,w,h = cv2.boundingRect(contours[largest_contour])
    middle = x + w//2

    result = cv2.line(im_with_keypoints,
                      (middle, y),
                      (middle, y + h),
                      (0, 0, 255),
                      3)

    if moment_x < middle - 20:
        direction = "left"
    elif moment_x > middle + 20:
        direction = "right"
    else:
        direction = "straight"

    result = cv2.putText(im_with_keypoints,
                         direction,
                         (50, 50),
                         cv2.FONT_HERSHEY_SIMPLEX,
                         2,
                         (0,0,255),
                         2,
                         cv2.LINE_AA)

    cv2.imshow("Result", result)

    k = cv2.waitKey(0)

cv2.destroyAllWindows()
