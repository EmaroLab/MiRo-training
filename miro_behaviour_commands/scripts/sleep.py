#!/usr/bin/env python

import rospy
import miro_msgs

from ModeBase import StateMode

class SleepMode(StateMode):
    EDGE_NAME = 'sleep' # TODO duplicated in `command_State_manager.py`
    
    def get_edge_name(self):
        return self.EDGE_NAME

    def set_miro_controll(self,msg):        
        msg.eyelid_closure = 1.0
        msg.lights_raw = [0,255,255,0,0,0,0,0,0,0,0,0,0,0,0,0,255,255] #white color
        msg.tail = -1.0
        msg.ear_rotate = [0.0,0.0]
        msg.body_config = [0.0,0.8,0.3,0.02]
        msg.body_config_speed = [0.0,-1.0,-1.0,-1.0]
        return msg

if __name__== '__main__':
    rospy.init_node( SleepMode.EDGE_NAME)
    m = SleepMode()
    m.ros_node_loop()
