#!/usr/bin/env python3

import cv2
import numpy as np

img = cv2.imread('ariac_img.png')

# img_bin_tr = img[28:221, 355:550]
# img_bin_tl = img[28:221, 117:310]
# img_bin_br = img[265:460, 355:550]
# img_bin_bl = img[265:460, 117:310]
img_bin_tr = img[26:221, 354:550]
img_bin_tl = img[26:221, 116:310]
img_bin_br = img[265:460, 354:550]
img_bin_bl = img[265:460, 115:310]

# cv2.imshow('Top Left',img_bin_tl)
# cv2.imshow('Top Right',img_bin_tr)
# cv2.imshow('Bottom Left',img_bin_bl)
# cv2.imshow('Bottom Right',img_bin_br)
# cv2.waitKey(0)

image = [img_bin_tr, img_bin_tl, img_bin_br, img_bin_bl]

for img in image:
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ret, thresh = cv2.threshold(img_gray, 127, 200, 0)
    # thresh_blur = cv2.GaussianBlur(thresh, (7,7), 0)
    # blur = cv2.GaussianBlur(img_gray,(3,3), 0)
    ret, thresh = cv2.threshold(img_gray, 86, 255, 0)
    # ret,thresh = cv2.threshold(img_gray,200,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # kernel = np.ones((5,5),np.uint8)
    # thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for c in contours:
        # area = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)
        area = w*h
        if area > 500 and area < 10000:
            
            # x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(img, (x, y), (x + w, y + h), (255,0,0), 2)
            ((x_c,y_c), r) = cv2.minEnclosingCircle(c)
            bgr= img[int(y_c),int(x_c)]
            # print(bgr)
            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            hsv = img_hsv[int(y_c),int(x_c)]

            if (hsv[0]>10 and hsv[0]<25):
                print("Orange")
                # print(area)
            elif(hsv[0]>=130 and hsv[0]<170):
                print("Purple") 
                # print(area) # 1716 to 2112 for regulator
            elif(hsv[0]>90 and hsv[0]<130):
                print("Blue")
                # print(area) # 2601 to 3360 for cylinder (2365 for bad cylinder)
            elif(hsv[0]>=0 and hsv[0]<=10): #and hsv[1]>=100 and hsv[2]>=100):
                print("Red") 
                # print(area) # 900 to 2173 for battery
            elif(hsv[0]>36 and hsv[0]<89):
                print("Green")  
                # print(area)  # 2392 - 2499 for sensor

            cv2.circle(img, (int(x_c), int(y_c)), int(2), (255,255,255), 2)
          
    cv2.imshow('Contours', img)
    cv2.waitKey(0)
    print("")

cv2.destroyAllWindows()