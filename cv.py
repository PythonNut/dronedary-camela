import cv2
import numpy as np

DEBUG = True

red_angle_width = 10
red_min_brightness = 20
red_min_saturation = 15
red_lower1 = np.array([0,red_min_brightness,red_min_saturation])
red_upper1 = np.array([red_angle_width,255,255])

red_lower2 = np.array([180-red_angle_width,red_min_brightness,red_min_saturation])
red_upper2 = np.array([180,255,255])

cam = cv2.VideoCapture(1)

while True:
    ret_val, img = cam.read()
    #red = img.copy()[:,:,0]
    output = img.copy()

    img = cv2.GaussianBlur(img,(5,5),0)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    mask2 = cv2.inRange(hsv, red_lower2, red_upper2)

    mask = mask1 | mask2

    ff = mask.copy()
    h, w = mask.shape[:2]
    mask2 = np.zeros((h+2, w+2), np.uint8)
    cv2.floodFill(ff, mask2, (0,0), 255)

    #ff = cv2.bitwise_not(ff)

    #mask = mask | cv2.bitwise_not(ff)
    mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    img = img & mask_rgb

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray,(5,5),0)
    #gray = cv2.medianBlur(gray,5)

    kernel = np.ones((3,3),np.uint8)

    for _ in range(2):
        gray = cv2.erode(gray,kernel,iterations = 1)
        gray = cv2.dilate(gray,kernel,iterations = 1)

    c = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 0.5, 41, param1=70, param2=10, minRadius=5,maxRadius=25)
    print(c)
    if c is not None and DEBUG:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(c[0, :]).astype("int")

        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

    cv2.imshow("output", output)
    cv2.imshow("gray", gray)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
