import cv2
import numpy as np
import client

DEBUG = True
ID_BYTE = 0x1
TARGET_IP = "134.173.55.124"

s = client.set_up_client_socket(TARGET_IP, 8081)

red_angle_width = 10
red_min_brightness = 20
red_min_saturation = 15
red_lower1 = np.array([0,red_min_brightness,red_min_saturation])
red_upper1 = np.array([red_angle_width,255,255])

red_lower2 = np.array([180-red_angle_width,red_min_brightness,red_min_saturation])
red_upper2 = np.array([180,255,255])

blue_angle_width = 10
blue_min_brightness = 0
blue_min_saturation = 40

blue_lower = np.array([120-blue_angle_width, blue_min_brightness, blue_min_saturation])
blue_upper = np.array([120+blue_angle_width, 255, 255])

cam = cv2.VideoCapture(0)

def iterative_refine(img, iterations=1):
    for _ in range(2):
        img = cv2.erode(img,kernel,iterations=iterations)
        img = cv2.dilate(img,kernel,iterations=iterations)
    return img

while True:
    ret_val, img = cam.read()
    #red = img.copy()[:,:,0]
    output = img.copy()

    img = cv2.GaussianBlur(img,(5,5),0)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    rmask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    rmask2 = cv2.inRange(hsv, red_lower2, red_upper2)

    rmask = rmask1 | rmask2

    #ff = mask.copy()
    #h, w = mask.shape[:2]
    #mask2 = np.zeros((h+2, w+2), np.uint8)
    #cv2.floodFill(ff, mask2, (0,0), 255)

    #ff = cv2.bitwise_not(ff)

    #mask = mask | cv2.bitwise_not(ff)
    rmask_rgb = cv2.cvtColor(rmask, cv2.COLOR_GRAY2BGR)
    rimg = img & rmask_rgb

    rgray = cv2.cvtColor(rimg, cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray,(5,5),0)
    #gray = cv2.medianBlur(gray,5)

    #kernel = np.ones((3,3),np.uint8)

    c = cv2.HoughCircles(rmask, cv2.HOUGH_GRADIENT, 0.5, 41, param1=70, param2=10, minRadius=5,maxRadius=25)
    #print(c)
    if c is not None and DEBUG:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(c[0, :]).astype("int")

        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            pass
        
        x, y, r = circles[0]
        #client.send_data(s, bytearray([ID_BYTE, 0, 0, x//256, x%256, 0, 0, y//256, y%256]))
        client.send_data(s,','.join(map(str, [ID_BYTE, x, y])))

    #bmask = cv2.inRange(hsv, blue_lower, blue_upper)
    # c = cv2.HoughCircles(img[:,:,2], cv2.HOUGH_GRADIENT, 0.5, 41, param1=30, param2=15, minRadius=5,maxRadius=15)
    # print(c)
    # if c is not None and DEBUG:
    #     # convert the (x, y) coordinates and radius of the circles to integers
    #     circles = np.round(c[0, :]).astype("int")

    #     max_x = 0
    #     # loop over the (x, y) coordinates and radius of the circles
    #     for (x, y, r) in circles:
    #         print(img[x, y,:])
    #         # draw the circle in the output image, then draw a rectangle
    #         # corresponding to the center of the circle
    #         cv2.circle(output, (x, y), r, (0, 255, 0), 4)
    #         cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)


    cv2.imshow("output", output)
    cv2.imshow("gray", rmask)
    if cv2.waitKey(1) == 27:
        break






cv2.destroyAllWindows()
