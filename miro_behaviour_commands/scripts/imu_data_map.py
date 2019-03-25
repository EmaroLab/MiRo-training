#!/usr/bin/env python

################################################################ 

## \file imu_data_map.py
## \brief The node imu_data_map.py subscribes to the smartwatch's accelerometer data and publish the linear and angular velocities conveniently mapped.
## @n More in details:
## @n It subscribes to the topic /inertial
## @n It reads from that topic the values of linear accelerations along x and y
## @n It maps these accelerations into linear and angular velocities 
## @n It publish on /imu_mapping the mapped values
## @n Two modality are available: BASE and ADVANCED
## and they can be selected from launch file



import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose


import math
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

## \brief The Class SmartwatchData allows the subscription to the smartwatch's accelerometer data and publishing of the 
## linear and angular velocities conveniently mapped.


class SmartwatchData():

    ##Constructor
    def __init__(self):

        ##Last acelleration data received from smartwatch
        self.last_acc = [0,0,0]
        ##Control mode set from launch file
        self.mode = rospy.get_param('control_mode', 'advanced')
        ##Subscriber to the topic /inertial a message of type Imu
        self.sub_smartwatch = rospy.Subscriber('/inertial',Imu,self.callback_smartwatch_data,queue_size=1)
        ##Publisher on the topic /imu_mapping a message of type Twist
        self.pub_mapping = rospy.Publisher('/imu_mapping', Twist, queue_size=0)
    
    ## Callback function that when the data from the smartwatch are received maps them into linear and angular velocities.
    ## Based on the value of the attribute mode, two different modalities of control are available
    ## @n <b>mode: basic</b>
    ## <div style="margin-left:40px;">
    ## It allows the user to exert three different types of control. 
    ## They correspond to three gesture and therefore to specific ranges of linear acceleration values.
    ## @n The control gesture are:
    ## <ul><li><i>Stay Still</i></li> 
    ## <li><i>Turn Right/Left</i></li>
    ## <li><i>Go Forward/Backward</i></li></ul></div>
    ## @n <b>mode: advanced</b>
    ## <div style="margin-left:40px;">
    ## It allows the user to exert the basic types of control but also combination of them. 
    ## @n All the linear acceleration values beetween the basic ranges are mapped as combination of linear and angular velocities.
    ## @n<i> e.g The control gesture combination could be Go Forward and Turn Right.</i>  </div>
    def callback_smartwatch_data(self,imu_data):
        
        q=Twist()
        self.last_acc[0] = imu_data.linear_acceleration.x
        self.last_acc[1] = imu_data.linear_acceleration.y
        self.last_acc[2] = imu_data.linear_acceleration.z
        
        sw_vel=Twist()
        s_vel=Twist()

        #Basic Control Gesture: Stay Still
        if  -5 < self.last_acc[0] < 5 and -5 < self.last_acc[1] < 5:
            sw_vel.linear.x=0.0
            sw_vel.angular.z=0.0
            s_vel.linear.x=0.0
            s_vel.angular.z=0.0
            print ('miro stay still')


        #Basic Control Gesture: Go Forward/Backward
        elif -8 < self.last_acc[0] > 8:

            sw_vel.linear.x = self.last_acc[0]*50
            sw_vel.angular.z = 0.0
            s_vel.linear.x = self.last_acc[0]*50
            s_vel.angular.z = 0.0
        
        #Basic Control Gesture: Turn Right/Left
        elif -7 < self.last_acc[1] > 7:

            sw_vel.linear.x = 0.0
            sw_vel.angular.z = self.last_acc[1]*0.100
            s_vel.linear.x = 0.0
            s_vel.angular.z = self.last_acc[1]*0.100

        #Advanced Control Gesture: Combination of Basic Control Gesture
        else:

            sw_vel.linear.x=self.last_acc[0]*50
            sw_vel.angular.z=self.last_acc[1] #*0.50
            s_vel.linear.x = 0.0
            s_vel.angular.z = 0.0
            #print ('miro move')
        
        if self.mode == 'advanced':

            q = sw_vel

        if self.mode == 'basic':

            q = s_vel
        
        self.pub_mapping.publish(q)

    def main (self):
        rospy.spin()

if __name__=='__main__':
    rospy.init_node('imu_data_map')
    smartwatch = SmartwatchData()
    smartwatch.main()
