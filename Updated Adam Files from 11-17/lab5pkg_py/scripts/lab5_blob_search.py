#!/usr/bin/env python

import cv2
import numpy as np
from geometry_msgs.msg import Point
from numpy.linalg import inv

'''
# Given params for online lab camera calibration
theta = -0.09
beta = 730.0
tx =-0.297435628054
ty =-0.0763807206584
tz = 0
'''
#Parameters for camera calibration
#OpenCV coordinate format = [columns, rows] = [y, x]
cen = [(513.0565795898438, 128.88107299804688),  (196.5128936767578, 128.91629028320312)]
#cen is the pair of coordinates to be used for both beta and theta calculation
#cen is separated by a known distance of 40 cm, and is already aligned with the y-axis
#1st coordinate is the farther point (further to the right in the camera frame), 2nd coordinate is the closer point (further to the left in the camera frame)
beta = np.sqrt( (cen[0][0]-cen[1][0])**2 + (cen[0][1]-cen[1][1])**2 )/0.4
drow = cen[0][1] - cen[1][1] #second element == rows, row 1 - row 2 == coordinate 1 - coordinate 2
dcol = cen[0][0] - cen[1][0] #first element == columns, row 1 - row 2 == coordinate 1 - coordinate 2
theta = np.arctan2(drow,dcol)  #theta 

#world coords and pixel coords will be given for the ball farther to the right
worldcoords = np.array([[0.35,0.15]]) # == [yw, xw] in m  world coords of test case to calculate tx ty
pixelcoords = np.array([[513.0565795898438, 128.88107299804688]]) # == [c, r] in pixels
xc_t = (pixelcoords[0][1]-240)/beta
yc_t = (pixelcoords[0][0]-320)/beta
xc_t_p = xc_t*np.cos(theta) - yc_t*np.sin(theta)
#yc_t_p = xc_t*np.sin(thw) + yc_t*np.cos(thw) 
yc_t_p = xc_t_p*np.tan(theta) + yc_t/np.cos(theta) #refer to orange derivation box on page 7 of "Lab 5 Calculations"
tx =  xc_t_p - worldcoords[0][1]  #in terms of theta
ty = yc_t_p - worldcoords[0][0]
tz = 0

'''
#In-Person params for camera calibration
#OpenCV coordinate format = [columns, rows] = [y, x]
cen = [[436.9295349121094, 295.558349609375],[363.28912353515625, 294.6224365234375]]
beta = np.sqrt( (cen[0][0]-cen[1][0])**2 + (cen[0][1]-cen[1][1])**2 )/0.1
#print(beta)
y_aligned = [(405.1206970214844, 141.53045654296875), (334.5589599609375, 141.5653839111328)] # y_aligned = [farther_point, closer_point]
drow = y_aligned[0][1] - y_aligned[1][1] #first element, row 2 - row 1
dcol = y_aligned[0][0] - y_aligned[1][0] #second element, row 2 - row 1
thw = np.arctan2(drow,dcol)  #theta 
#print(thw)
worldcoords = np.array([[0.09,0.3135]]) #yw, xw in m  world coords of test case to calculate tx ty
pixelcoords = np.array([[361.7375, 295.0371]]) #c, r in pixels
xc_t = (pixelcoords[0][1]-240)/beta
yc_t = (pixelcoords[0][0]-320)/beta
xc_t_p = xc_t*np.cos(thw) - yc_t*np.sin(thw)
#yc_t_p = xc_t*np.sin(thw) + yc_t*np.cos(thw) 
yc_t_p = xc_t_p*np.tan(thw) + yc_t/np.cos(thw) #refer to orange derivation box on page 7 of "Lab 5 Calculations"
tx =  xc_t_p - worldcoords[0][1]  #in terms of theta
ty = yc_t_p - worldcoords[0][0]
tz = 0
'''

# Params for camera
w = 640
h = 480

# Params for block
block_height = 0.015

# Function that converts image coord to world coord
def IMG2W(x,y):
    global theta
    global beta
    global tx
    global ty
    global tz

    ################################ Your Code Start Here ################################
    # Given theta, beta, tx, ty, tz, calculate the world coordinate of r,c namely xw, yw
    r = y       #pixels     
    c = x       #pixels
    OR = 240    #pixles
    OC = 320    #pixels
    #check how rotation is done - the theta given here is Rotation from camera to world
    Rwc= np.array([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta), 0],[0, 0, 1]])
    Rcw = inv(Rwc)
    xc = (r-OR)/beta  #m
    yc = (c-OC)/beta  #m
    zc = 0.06 #this is the size of the scaled basketballs
    c_coordinates = np.array([[xc],[yc],[zc]])  #xc yc zc in camera frame in m
    t_coordinates = c_coordinates - np.array([[tx],[ty],[tz]]) #translated to world origin 
    w_coordinates = np.matmul(Rcw,t_coordinates)  #rotated in world origin   ?Order?

    return w_coordinates
    pass

def blob_search(image_raw, color):

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    
    #THE VALUES BELOW STILL NEED TO BE EDITED TO ACCOUNT FOR GREEN BLOCK PARALLAX
    #Additionally, can also just alter HSV for basketballs, and work on parallax for that case specifically

    # Edit Below
    ##################################
    # Replace these with your values for each color
    #lower = (15, 50, 50)   # basketball lower
    #upper = (55, 255, 235) # basketbal upper
    #lower = (110, 50, 50)   # Blue lower
    #upper = (130, 255, 255) # Blue upper
    #lower = (50,145,125)     # test green lower
    #upper = (70,255,255)   # test green upper 
    #lower = (50,150,100)     # OG green lower
    #upper = (70,255,255)   # OG green upper 
    #lower = (10/2,0,0)     # basketball lower EDGES
    #upper = (30/2,255,200)   # basketball upper EDGES
    lower = (0,30,30)     # basketball lower
    upper = (50/2,255,255)   # basketball upper
    
    # Edit Above
    ##################################

    mask_image =  cv2.inRange(hsv_image, lower, upper)

    # Edit Below
    ##################################
    # Setup SimpleBlobDetector parameters by editing the following code:
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True   
    #basketball is about 55 pixel radius. pi*r^2 ~= 9500 pixels
    #basketball surface is split into 8 sections from point of view of camera == 1000 pixels per blob
    params.minArea = 1000  #block
    params.maxArea = 12000   #block

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inertia
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False


    # Edit Above
    ##################################

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)
    '''
    # Detect keypoints
    keypoints = detector.detect(mask_image)
    i = len(keypoints)
    if i == 0:
        print("No blobs found... ")
        r = None
        c = None
    elif i == 1:
        print("One blob found... Yay!")
        keypoint = keypoints[0]
        c = keypoint.pt[0]
        r = keypoint.pt[1]
    else:
        print("{} blobs found, only passing the first...".format(i) )
        keypoint = keypoints[0]
    '''
    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    xw_yw = []
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    if(num_blobs == 0):
        print("No balls found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
        if(num_blobs == 1):
            print(str(num_blobs) + " ball found!")
        else:
            print(str(num_blobs) + " balls found!")

    im_with_keypoints = image_raw
    if len(keypoints) == 0:
        im_with_keypoints = image_raw
    else:
        draw_color = 0
        # Feel free to use these as the color that you draw your keypoints and circle
        if color == 'yellow':
            draw_color = (255, 0, 0)
        else:
            draw_color = (255, 0, 255)

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0, draw_color, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    

    '''        
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    '''


    
    # Edit Below
    ##################################
    # Edit below to mark keypoints and draw a circle around the block.
        # Draw a circle around the detected block
        # im_with_keypoints = cv2.circle(image_raw, (int(c), int(r)), ???)

        # Draw the keypoints on the detected block
        # im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, keypoints, ???)
        #print("keypoint = ",keypoints)
        #Draw keypoints that are the size of the object, centroid-focused
        #im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0, (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # Edit Above
    ##################################
    
    # Show masked image
    im_mask = cv2.cvtColor(mask_image, cv2.COLOR_GRAY2BGR)
    cv2.namedWindow("Masked Image")
    cv2.imshow("Masked Image", im_mask)
    '''
    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

        xw_yw = []


    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    # Note to students for pressing enter to continue
    im_with_keypoints = cv2.putText(im_with_keypoints, 'Press Enter to Continue', (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    '''

    cv2.namedWindow("Press Enter to Continue")
    cv2.imshow("Press Enter to Continue", im_with_keypoints)
    #print(blob_image_center)
    #print(xw_yw)
    while True:
        key = cv2.waitKey(0)
        if key == 13:
            cv2.destroyAllWindows()
            break
    return xw_yw
