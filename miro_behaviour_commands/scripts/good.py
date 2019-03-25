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

## Function used for conversion from byteArray to String (The values that we get for the head touch sensors are byteArrays)
def fmt(x, f): 
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s

## \file good.py 
## \brief The node good.py implements the action corresponding to the command "Good".
## @n The Robot moves up its head and move its tail.
## @n The node subscribes to /platform/sensors and reads the values of the capacitive sensors on Miro's body and head.
## @n When the user touches Miro it changes behavior and lightening pattern.

## The class GoodMode implements a cheerful behavior and allows to interact with Miro's capacitive body

class GoodMode():

    def __init__(self):
        ## Node rate
        self.rate = rospy.get_param('rate',200)

        ## Initialization of head capacitive sensors
        self.h1 = 0
        self.h2 = 0
        self.h3 = 0
        self.h4 = 0
        ## Initialization of bodycapacitive sensors
        self.b1 = 0
        self.b2 = 0
        self.b3 = 0
        self.b4 = 0

        ## Subscriber to the topic /miro/rob01/platform/sensors a message of type platform_sensors that cointains the information about the capacitive sensors.
        self.sub_sensors_touch = rospy.Subscriber(topic_root + '/platform/sensors', platform_sensors, self.callback_touch,queue_size =1)
        ## Publisher to the topic /miro_good a message of type platform_control which corresponds to the "Good" action.
        self.pub_platform_control = rospy.Publisher('/miro_good',platform_control,queue_size=0)

    ## Callback function that saves in class' attributes the capacitive sensor readings converted    
    def callback_touch(self, datasensor):

        self.h1 = int(fmt(datasensor.touch_head, '{0:.0f}')[0]) 
        self.h2 = int(fmt(datasensor.touch_head, '{0:.0f}')[3]) 
        self.h3 = int(fmt(datasensor.touch_head, '{0:.0f}')[6])
        self.h4 = int(fmt(datasensor.touch_head, '{0:.0f}')[9])
        self.b1 = int(fmt(datasensor.touch_body, '{0:.0f}')[0])
        self.b2 = int(fmt(datasensor.touch_body, '{0:.0f}')[3]) 
        self.b3 = int(fmt(datasensor.touch_body, '{0:.0f}')[6]) 
        self.b4 = int(fmt(datasensor.touch_body, '{0:.0f}')[9])
            

    ## Function that implements different behavior and lightening pattern depending on where it is touched

    def miro_good(self):

        r = rospy.Rate(self.rate)
        q = platform_control()
        count = 0
        while not rospy.is_shutdown():
            
            q.sound_index_P1 = 1

            if self.h1 == 1 or self.h2 == 1 or self.h3 == 1 or self.h4 == 1:
                q.eyelid_closure = 0.4
                q.lights_raw = [255,64,64,255,64,64,255,64,64,255,64,64,255,64,64,255,64,64]
                q.tail = 0.0
                q.body_config = [0.2,0.5,0.2,-0.5]
                q.body_config_speed = [0.0,-1.0,-1.0,-1.0]

            elif self.b1 == 1 or self.b2 == 1 or self.b3 == 1 or self.b4 == 1:
                q.eyelid_closure = 0.1
                q.lights_raw = [255,129,0,255,129,0,255,129,0,255,129,0,255,129,0,255,129,0]
                q.tail = 0.0
                q.body_config = [0.0,0.29,-0.6,-0.26]
                q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
                q.ear_rotate = [0.5,0.5]
            else:
                q.eyelid_closure = 0.0
                q.body_config = [0.0,0.25,0.0,-0.25]
                q.body_config_speed = [0.0,-1.0,-1.0,-1.0]
                q.tail = 68
                q.lights_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                q.ear_rotate = [0.0,0.0]
            self.pub_platform_control.publish(q)

            r.sleep()


if __name__== '__main__':
    rospy.init_node('good')
    topic_root = "/miro/" + sys.argv[1]
    good = GoodMode()
    good.miro_good()
