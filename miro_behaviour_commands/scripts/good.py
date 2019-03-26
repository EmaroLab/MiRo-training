#!/usr/bin/env python

import rospy
from miro_msgs.msg import platform_sensors
from ModeBase import StateMode


## \file good.py 
## \brief The node good.py implements the action corresponding to the command "Good".
## @n The Robot moves up its head and move its tail.
## @n The node subscribes to /platform/sensors and reads the values of the capacitive sensors on Miro's body and head.
## @n When the user touches Miro it changes behavior and lightening pattern.

## The class GoodMode implements a cheerful behavior and allows to interact with Miro's capacitive body
class GoodMode(StateMode):
    EDGE_NAME = 'good' # TODO duplicated in `command_State_manager.py`
    
    def __init__(self):
	StateMode.__init__(self)
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
        self.sub_sensors_touch = rospy.Subscriber(self.miro_root_topic_name + 'platform/sensors', platform_sensors, self.callback_touch,queue_size =1)

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
            

    def get_edge_name(self):
        return self.EDGE_NAME

    def set_miro_controll(self,msg):        
        msg.sound_index_P1 = 1
        if self.h1 == 1 or self.h2 == 1 or self.h3 == 1 or self.h4 == 1:
            msg.eyelid_closure = 0.4
            msg.lights_raw = [255,64,64,255,64,64,255,64,64,255,64,64,255,64,64,255,64,64]
            msg.tail = 0.0
            msg.body_config = [0.2,0.5,0.2,-0.5]
            msg.body_config_speed = [0.0,-1.0,-1.0,-1.0]
        elif self.b1 == 1 or self.b2 == 1 or self.b3 == 1 or self.b4 == 1:
            msg.eyelid_closure = 0.1
            msg.lights_raw = [255,129,0,255,129,0,255,129,0,255,129,0,255,129,0,255,129,0]
            msg.tail = 0.0
            msg.body_config = [0.0,0.29,-0.6,-0.26]
            msg.body_config_speed = [0.0,-1.0,-1.0,-1.0]
            msg.ear_rotate = [0.5,0.5]
        else:
            msg.eyelid_closure = 0.0
            msg.body_config = [0.0,0.25,0.0,-0.25]
            msg.body_config_speed = [0.0,-1.0,-1.0,-1.0]
            msg.tail = 68
            msg.lights_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            msg.ear_rotate = [0.0,0.0]
        return msg

if __name__== '__main__':
    rospy.init_node( GoodMode.EDGE_NAME)
    m = GoodMode()
    m.ros_node_loop()

## Function used for conversion from byteArray to String (The values that we get for the head touch sensors are byteArrays)
def fmt(x, f): 
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s


