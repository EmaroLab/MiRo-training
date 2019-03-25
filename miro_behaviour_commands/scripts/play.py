#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import opencv_apps
from opencv_apps.msg import CircleArrayStamped

import math
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

## \file play.py 
## \brief The node play.py implements the action corresponding to the command "Play".
## @n The node subscribe to the circles detected in the Left and Right camera.
## @n The Robot moves in order to keep the ball in both camera frames.

## \brief The class BallDetection implements the detection of the circles in the cameras and the robot behavior to keep the circles in sight
class BallDetection():

        
    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',200)

        ## Initialization of the CircleArrayStamped variable in the Right callback
        self.detect_right = 0
        ## Initialization of the CircleArrayStamped variable in the Left callback
        self.detect_left = 0
        
        ## Flag for Red Ball detected in Right camera
        self.ball_right = False
        ## Flag for Red Ball detected in Leftcamera
        self.ball_left = False
        ## Flag for Red Ball detected in Both cameras
        self.ball = False

        ## Number of positive detection on Right camera
        self.count_right = 0
        ## Number of positive detection on Left camera
        self.count_left = 0
        ## Number of negative detection on Right camera
        self.count_no_right = 0
        ## Number of negative detection on Left camera
        self.count_no_left = 0
        ## Number of positive detection on Both cameras
        self.count_ball = 0

        ## Subscriber to the topic /right/hough_circles/circles a message of type CircleArrayStamped that cointains the information about the circles detected in the Right camera.
        self.sub_camera_right = rospy.Subscriber('right/hough_circles/circles', CircleArrayStamped, self.callback_camera_right,queue_size=1)
        ## Subscriber to the topic /right/hough_circles/circles a message of type CircleArrayStamped that cointains the information about the circles detected in the Left camera.
        self.sub_camera_left = rospy.Subscriber('left/hough_circles/circles', CircleArrayStamped, self.callback_camera_left,queue_size=1)
        ## Publisher to the topic /miro_play a message of type platform_control which corresponds to the "Play" action.
        self.pub_follow_ball = rospy.Publisher('miro_play',platform_control,queue_size=0)
        

    ## Callback that receive the message that contains the information of the circles detected in the Right camera.
    ## @n If the ball is detected for more than 3 times the flag self.ball_right is set to True.
    ## @n If the ball is not detected for more than 3 times the flag self.ball_right is set to False.
    def callback_camera_right(self, ball):

        self.detect_right = ball.circles

        if not self.detect_right: 
            self.count_no_right = self.count_no_right + 1 
            if self.count_no_right > 3:
                self.count_right = 0
                print "NO DETECTION IN RIGHT CAMERA"
                self.ball_right = False
            
        else:
            self.count_right = self.count_right + 1
            if self.count_right > 3:
                self.count_no_right = 0
                print "BALL DETECTED IN RIGHT CAMERA"
                self.ball_right = True
                
    ## Callback that receive the message that contains the information of the circles detected in the Left camera.
    ## @n If the ball is detected for more than 3 times the flag self.ball_right is set to True.
    ## @n If the ball is not detected for more than 3 times the flag self.ball_right is set to False.
    def callback_camera_left(self, ball):

        self.detect_left = ball.circles

        if not self.detect_left: 
            self.count_no_left = self.count_no_left + 1
            if self.count_no_left > 3:
                self.count_left = 0
                print "NO DETECTION IN LEFT CAMERA"
                self.ball_left = False
            
        else:
            self.count_left = self.count_left + 1
            if self.count_left > 3: 
                self.count_no_left = 0
                print "BALL DETECTED IN LEFT CAMERA"
                self.ball_left = True
               
    ## Function that evaluate the detection of the ball and publish a message of type platform_control in order to keep the ball always in sight.
    ## @n If the ball is detected in the Right camera, the robot will rotate towards right until the ball is detected also by the Left camera.
    ## @n If the ball is detected in the Left camera, the robot will rotate towards left until the ball is detected also by the Right camera.
    ## @n If the ball is detected in both cameras, the robot will go forward to reach the ball.
    def compared_detection(self):

        r = rospy.Rate(self.rate)
        q = platform_control()

        while not rospy.is_shutdown():
            if self.ball_right and self.ball_left:
                self.count_ball = self.count_ball + 1 
                # rospy.loginfo(self.count_ball)
                if self.count_ball > 3:
                    self.ball = True
                    print "DETECTION COMPLETE"
                    q.body_vel.linear.x = 100.0
                    q.body_vel.angular.z = 0.0
                    q.lights_raw = [255,150,0,255,150,0,255,150,0,255,150,0,255,150,0,255,150,0]
                self.pub_follow_ball.publish(q)

            else:
                self.count_ball = 0
                self.ball = False
                #q.body_vel.linear.x = 0.0
                #q.body_vel.angular.z = 0.0
                print "NO COMPLETE DETECTION"
                if self.ball_right:
                    q.body_vel.linear.x = 0.0
                    q.body_vel.angular.z = -0.2
                elif self.ball_left:
                    q.body_vel.linear.x = 0.0
                    q.body_vel.angular.z = 0.2
                else:
                    q.body_vel.linear.x = 0.0
                    q.body_vel.angular.z = 0.0

            self.pub_follow_ball.publish(q)
            r.sleep()
                     

if __name__== '__main__':
    rospy.init_node('play')
    ball = BallDetection()
    ball.compared_detection()


    
