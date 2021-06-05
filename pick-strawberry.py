import time 
import cv2 as cv
import numpy as np
# lib.Arm7Bot is a Python class for PineCone.ai robotic arm
# You should change the following line of code to your own robotic arm driver
from lib.Arm7Bot import Arm7Bot

region_rows = 64
region_cols = 64
def findStrawberry( bgr_image ):
    rows, cols, _ = bgr_image.shape
    #crop the center region of the image
    bgr_region = frame[int(rows/2)-region_rows:int(rows/2)+region_rows,
                   int(cols/2)-region_cols:int(cols/2)+region_cols]
    
    img_hsv=cv.cvtColor(bgr_region, cv.COLOR_BGR2HSV)

    # red mask1 (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv.inRange(img_hsv, lower_red, upper_red)
    # dark red mask2
    lower_red = np.array([160,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv.inRange(img_hsv, lower_red, upper_red)
    # join two masks
    maskRed = mask0+mask1
    
    # yellow-green mask
    lower_green = np.array([20,50,50])
    upper_green = np.array([60,255,255])
    maskGreen = cv.inRange(img_hsv, lower_green, upper_green)

    red_ratio = cv.sumElems(maskRed)
    green_ratio = cv.sumElems(maskGreen)
    red_ratio = red_ratio[0]/255/region_rows/region_cols/4
    green_ratio = green_ratio[0]/255/region_rows/region_cols/4
    
    #draw a rectange on the image to guide 
    cv.rectangle(bgr_image,
                  (int(cols/2)-region_cols, int(rows/2)-region_rows),
                  (int(cols/2)+region_cols, int(rows/2)+region_rows), (255, 0, 0), 3)    

    # Display the frame
    cv.imshow('Camera',frame)
    cv.imshow('maskRed', maskRed)
    cv.imshow('maskGreen', maskGreen)

    if(red_ratio > 0.6):
        return 'red'
    elif( green_ratio > 0.6):
        return 'green'
    else:
        return 'no strawberry'

# assign serial port to 7Bot. 
# ATTENTION: Change the parameter "/dev/cu.SLAB_USBtoUART" below 
#            according to your own computer OS system. Open whatever port is 7Bot on your computer.
# Usually:  "/dev/cu.SLAB_USBtoUART" on Mac OS
#           "/dev/ttyUSB0" on Linux
#           'COM1' on Windows
arm = Arm7Bot("/dev/ttyUSB0") #please adjust according to your own robotic arm

# capture frames from a camera with device index=0 by OpenCV
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_BUFFERSIZE, 1); #don't buffer too many images
#the main loop
while True:
    # reads frame from a camera
    ret, frame = cap.read() #the buffered one, read and throw it
    ret, frame = cap.read() #this one

    object_color = findStrawberry(frame)
    print(object_color)
        
    # Wait for 1ms, press q to exit
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    
    if(object_color == 'no strawberry'):
        continue

    # arm.setIK6() is a function to control PineCone.ai robotic arm,
    # you should change to functions of your own robotic arm
    arm.setIK6([0, 200, 150], [0, 0, -1]) #move to [x=0,y=200,z=150]
    time.sleep(1)
    arm.setIK6([0, 200, 40], [0, 0, -1]) #down
    time.sleep(1)
    
    arm.setAngle(6,90) #open hand
    time.sleep(1)
    arm.setAngle(6,15) #close hand
    time.sleep(1)
    
    arm.setIK6([0, 200, 150], [0, 0, -1]) #up
    time.sleep(1)
    
    if(object_color == 'red'):
        arm.setIK6([-200, 1, 150], [0, 0, -1]) #move to red busket
        time.sleep(1)
        arm.setIK6([-200, 1, 80], [0, 0, -1]) #down
        time.sleep(1)
    elif(object_color == 'green'):        
        arm.setIK6([-140, 140, 150], [0, 0, -1]) #move to green busket
        time.sleep(1)
        arm.setIK6([-140, 140, 80], [0, 0, -1]) #down
        time.sleep(1)

    arm.setAngle(6,90) #open hand
    time.sleep(1)
    arm.setIK6([0, 200, 150], [0, 0, -1]) #move back to [x=0,y=200,=150]
    time.sleep(1)
