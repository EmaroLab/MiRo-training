#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose
import random

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

## \file bad.py 
## \brief The node bad.py implements the action corresponding to the command "Bad".
## @n The Robot moves down its head and inclines it. It turns its back to the user and change its light pattern from Blue to Red.
##

## \brief The class BadMode implements a robot bad behaviour

class BadMode():
    NODE_RATE = 200

    ## Constructor
    def __init__(self):        
        ## Publisher to the topic /miro_bad a message of type platform_control which corresponds to the "Bad" action.
        self.pub_platform_control = rospy.Publisher('/miro_bad',platform_control,queue_size=0)
        rospy.Subscriber("//miro_state_controller/bad", Bool, state_controller_callback)
    
    def callback(state_controller_callback):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    ## Function that sets the parameters of the structure platform_control corresponding to action "Bad".
    def miro_bad(self):

        r = rospy.Rate(self.NODE_RATE)
        q = platform_control()

        while not rospy.is_shutdown():

            q.eyelid_closure = 0.3
            q.body_config = [0.0,1.0,0.2,0.1]
            q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
            q.lights_raw = [0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255]
            q.tail = -1.0
            q.ear_rotate = [1.0,1.0]
            q.body_vel.linear.x = 0.0
            q.body_vel.angular.z = 0.2
            self.pub_platform_control.publish(q)
            r.sleep()

if __name__== '__main__':
    rospy.init_node('bad')
    bad = BadMode()
    bad.miro_bad()
