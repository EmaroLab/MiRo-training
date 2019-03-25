#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import math
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

## \file gbb_miro.py
## \brief The node gbb_miro.py subscribes to the linear and angular velocity mapped in the imu_data_map node and publish a platform_control message
## @n A platform_control message contains linear and angular velocities, the lightening pattern of miro's body and other tipes of messages @n.
## @n More in details:
## @n Subscribe to the topic /imu_mapping
## @n Read from that topic the value from the smartwatch correctly mapped into miro body velocity
## @n Publish on /gbb a platform_control msg containing miro body velocity and a lightening pattern based on smartwatch commands
## @n For example: if the command of the smartwatch is a rotation towards right miro will turn right and the right part of its body will lit

##\brief The class GestureBased implements the Gesture based behavior

class GestureBased():
    ## Constructor
    def __init__(self):
        
        ## Linear and Angular velocities that will be part of the platform_control message
        self.body_vel=Twist()

        ## Subscriber to the topic /imu_mapping a message of type Twist
        self.sub_imu_mapping = rospy.Subscriber('/imu_mapping',Twist,self.callback_gbb,queue_size=1)

        ## Publisher on the topic /gbb a message of type platform_control which corresponds to the Gesture Based Behavior
        self.pub_platform_control = rospy.Publisher('/gbb', platform_control, queue_size=0)


    ## Callback function that receive the Twist message and set the miro's body lightening pattern to construct the platform_control message to publish

    def callback_gbb(self,vel_msg):

        q=platform_control()

       
        self.body_vel.linear.x = vel_msg.linear.x
        self.body_vel.angular.z = vel_msg.angular.z

        q.body_vel = self.body_vel

        #lightening pattern

        if -5< vel_msg.linear.x < 5 and -0.1 < vel_msg.angular.z<0.1 :
            q.lights_raw = [0,255,255,0,255,255,0,255,255,0,255,255,0,255,255,0,255,255]

        if vel_msg.angular.z > 0 and vel_msg.angular.z > 0.1 :
            q.lights_raw = [0,0,255,0,0,255,0,0,255,0,0,0,0,0,0,0,0,0]

        if vel_msg.angular.z < 0 and vel_msg.angular.z < - 0.1 :
            q.lights_raw = [0,0,0,0,0,0,0,0,0,255,0,255,0,0,255,0,0,255]

        self.pub_platform_control.publish(q)
            

        #rospy.loginfo(sonar_msg.sonar_range)

    def main (self):
        rospy.spin()

if __name__== '__main__':
    rospy.init_node('gbb_miro')
    gesture_based = GestureBased()
gesture_based.main()