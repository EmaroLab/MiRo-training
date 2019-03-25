#!/usr/bin/env python

################################################################

import rospy
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

## \file kill.py 
## \brief The node kill.py implements the action corresponding to the command "Kill".
## @n The Robot moves down its head and squint the eyes. It changes its light pattern to Red and emits an aggressive sound.

##\brief The class KillMode implements a angry behavior for the robot

class KillMode():

    def __init__(self):

        ## Node rate
        self.rate = rospy.get_param('rate',200)
        ## Publisher to the topic /miro_kill a message of type platform_control which corresponds to the "Kill" action.
        self.pub_platform_control = rospy.Publisher('/miro_kill',platform_control,queue_size=0)
    
    ## Function that sets the parameters of the structure platform_control corresponding to action "Kill".
    def miro_kill(self):

        r = rospy.Rate(self.rate)
        q = platform_control()

        while not rospy.is_shutdown():

            q.eyelid_closure = 0.4
            q.body_config = [0.0,0.87,0.0,0.75]
            q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
            q.lights_raw = [255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0]
            q.tail = 0.68
            q.ear_rotate = [1.0,1.0]
            q.sound_index_P2 = 15
            self.pub_platform_control.publish(q)
            r.sleep()

if __name__== '__main__':
    rospy.init_node('bad')
    sad = KillMode()
    sad.miro_kill()
