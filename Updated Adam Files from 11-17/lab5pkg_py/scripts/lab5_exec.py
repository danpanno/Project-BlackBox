#!/usr/bin/env python

import sys, time
import numpy as numpy
import roslib
import rospy
import rospkg
import os
import random
import cv2
import argparse
import sys



from sensor_msgs.msg import CompressedImage
from scipy.ndimage import filters
from geometry_msgs.msg import Point
from lab5_spawn_block import *
from lab5_blob_search import *
from lab5_header import *
from lab5_func import *


# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 130*PI/180.0]

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

num_configs = 5

#place holder for camera feed and image
resized_image = 0

camera_image = 0

# Place holder for block world positions
green_world_position = None
yellow_world_position = None

# Final goal positions
# PLEASE USE THE FOLLOWING GOAL
goal1 = np.array([[-0.40,0.40,0.1205]])
goal2 = np.array([[-0.40,0.15,0.1205]])
goal3 = np.array([[-0.40,-0.10,0.1205]])
goal4 = np.array([[-0.55,0.275,0.1205]])
goal5 = np.array([[-0.55,0.025,0.1205]])

############## Your Code Start Here ##############

"""
TODO: define ROS topic callback funtions for getting the position of blocks
Whenever block_[color]/world publishes info these callback functions are called.
"""
# image (camera) position publisher
# world position subscriber

############### Your Code End Here ###############
"""
Here images are read and processed.
Defines the callback function linked to the subscriber function
"""
def camera_callback(msg):
        global resized_image
        
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        resized_image = cv2.resize(image_np, (640,480), interpolation=cv2.INTER_NEAREST)
        """
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        resized_image = cv2.resize(image_np, (320,240), interpolation=cv2.INTER_NEAREST)
        
        cv2.imshow('cv_img', resized_image)
        cv2.waitKey(2)
        """


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################

############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates (Point)
    target_xw_yw_zw: where to place the block in global coordinates (Point)

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """

    global digital_in_0

    start = np.hstack((start_xw_yw_zw.T,[[0]]))
    s_height = start + np.array([0, 0, 0.15, 0])
    target = np.hstack((target_xw_yw_zw,[[0]]))
    t_height = target + np.array([0, 0, 0.15, 0])
    error = 0

    move_arm(pub_cmd, loop_rate, lab_invk(s_height[0][0], s_height[0][1], s_height[0][2], s_height[0][3]), 4.0, 4.0)

    move_arm(pub_cmd, loop_rate, lab_invk(start[0][0],start[0][1], start[0][2], start[0][3]), 4.0, 4.0)

    gripper(pub_cmd, loop_rate, suction_on) #turn on suction
    # Delay to make sure suction cup has grasped the block
    time.sleep(0.5) #sleeps the function for 0.5 seconds to ensure block is fully gripped before moving on

    if ((digital_in_0) == 1): # PROPER SETUP IF = 1, ELIF = 0, INDENT THE IF COMMANDS

        move_arm(pub_cmd, loop_rate, lab_invk(s_height[0][0], s_height[0][1], s_height[0][2], s_height[0][3]), 4.0, 4.0)
        
        move_arm(pub_cmd, loop_rate, lab_invk(t_height[0][0], t_height[0][1], t_height[0][2], t_height[0][3]), 4.0, 4.0)

        move_arm(pub_cmd, loop_rate, lab_invk(target[0][0], target[0][1], target[0][2], target[0][3]), 4.0, 4.0)

        gripper(pub_cmd, loop_rate, suction_off) #turn off suction
        # Delay to make sure suction cup has grasped the block
        time.sleep(0.5) #sleeps the function for 0.5 seconds to ensure block is fully released before moving on
        
        move_arm(pub_cmd, loop_rate, lab_invk(t_height[0][0], t_height[0][1], t_height[0][2], t_height[0][3]), 4.0, 4.0)

    elif ((digital_in_0) == 0):

        gripper(pub_cmd, loop_rate, suction_off) #turn off suction

        move_arm(pub_cmd, loop_rate, lab_invk(s_height[0][0], s_height[0][1], s_height[0][2], s_height[0][3]), 4.0, 4.0) #move back to home position

        print("Ball was expected, but not found.")
        pass

    # global variable1
    # global variable2

    error = 0

    return error

############### Your Code End Here ###############

if __name__ == '__main__':

    # Parser
    parser = argparse.ArgumentParser(description='Please specify if a block is taken away or not')
    parser.add_argument('--missing', type=str, default='False')
    parser.add_argument('--image', type=int, default=0)
    args = parser.parse_args()

    # Check parser
    if args.missing.lower() == 'true':
        missing_block = True
    elif args.missing.lower() == 'false':
        missing_block = False
    else:
        print("Invalid argument for missing block, enter True of False")
        sys.exit()

    if args.image < 0 or args.image > 4:
        print("Invalid argument for image.  Enter a number from 0 to 4")
        sys.exit()
    else:
        config_idx = args.image

    # Initialize ROS node
    rospy.init_node('lab5_exec', anonymous=True)


    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    #Define the subscriber function for the compressed raw image topic
    sub_camera = rospy.Subscriber('/cv_camera_node/image_raw/compressed', CompressedImage, camera_callback, queue_size=1)

    ############## Your Code Start Here ##############

    # Define image position publisher
    # rospy.Publisher(?)

    # Define world position subscriber
    # rospy.Subscriber(?)

    ############## Your Code End Here ###############

    # Initialize ROS pack
    rospack = rospkg.RosPack()

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    # Spawn block
    spawn_block(config_idx, missing_block)

    # Get path to image
    #lab5_path = rospack.get_path('lab5pkg_py')
    #image_path = os.path.join(lab5_path, 'scripts', 'configs', str(config_idx) + '.jpg')
    # Read image
    #raw_image = cv2.imread(image_path)
    #raw_image = cv2.imread(resized_image)
    
    #wait for 15 seconds before setting image to the instantaneous value of resized_image
    #allows time for arm to move and blocks to spawn

    # Flip the image (Legacy webcam config)
    #assigns the value of resized image AT THAT INSTANT
    #put a wait timer to allow for arm to move out of the way and for blocks to spawn and settle
    time.sleep(2.5)
    cv_image = cv2.flip(resized_image, -1)
    #cv_image = resized_image

    # Call blob search (row, col)
    ball_coords = blob_search(cv_image, 'basketball')
    
    if (ball_coords != 0):
        loop_count = 1
    else:
        print('No basketballs were found!')


    while(loop_count > 0):
    
        if (len(ball_coords) == 1):
            move_block(pub_command, loop_rate, ball_coords[0], goal1, 4.0, 4.0)

        elif (len(ball_coords) == 2):
            move_block(pub_command, loop_rate, ball_coords[0], goal1, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[1], goal2, 4.0, 4.0)

        elif (len(ball_coords) == 3):
            move_block(pub_command, loop_rate, ball_coords[0], goal1, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[1], goal2, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[2], goal3, 4.0, 4.0)
        
        elif (len(ball_coords) == 4):
            move_block(pub_command, loop_rate, ball_coords[0], goal1, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[1], goal2, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[2], goal3, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[3], goal4, 4.0, 4.0)

        else:
            move_block(pub_command, loop_rate, ball_coords[0], goal1, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[1], goal2, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[2], goal3, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[3], goal4, 4.0, 4.0)
            move_block(pub_command, loop_rate, ball_coords[4], goal5, 4.0, 4.0)

        move_arm(pub_command, loop_rate, go_away, vel, accel)
        rospy.loginfo("Task Completed!")
        loop_count = loop_count - 1

    '''
    #COMMENTED OUT - TEST PROCESS TO CONFIRM CALIBRATION PARAMETERS
    xball = ball_coords[0][0]
    yball = ball_coords[0][1]
    zball = ball_coords[0][2]
    print('blob coordinates are: ' + str(xball) + ',' + str(yball) + ',' + str(zball))
    print('actual coordiantes are: 0.15, 0.15, 0.06')
    move_arm(pub_command, loop_rate, lab_invk(xball, yball, zball, 0), vel, accel) #use passed coords of centroid of the object
    #yellow_image_rc = blob_search(cv_image, 'yellow')
    gripper(pub_command, loop_rate, suction_on) #turn on suction
    # Delay to make sure suction cup has grasped the block
    time.sleep(1.5)
    move_arm(pub_command, loop_rate, go_away, vel, accel)
    gripper(pub_command, loop_rate, suction_off)
    time.sleep(1.5)
    '''

    ############## Your Code Start Here ##############
    # Hint: Remember to delay for your subscriber to update
    # Otherwise you might get None or old images (when running lab 5 multiple times)
    # a short rospy.sleep(time in seconds) after you publish should be sufficient

    # lab 2 move_block()

    ############## Your Code End Here ###############
    '''
    # Move arm to away position
    move_arm(pub_command, loop_rate, go_away, vel, accel)
    '''
    rospy.sleep(1)
