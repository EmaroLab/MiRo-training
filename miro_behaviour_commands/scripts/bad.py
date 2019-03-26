#!/usr/bin/env python

import rospy
import miro_msgs

from ModeBase import StateMode

class BadMode(StateMode):
    EDGE_NAME = 'bad' # TODO duplicated in `command_State_manager.py`
    
    def get_edge_name(self):
        return self.EDGE_NAME

    def set_miro_controll(self,msg):        
        msg.eyelid_closure = 0.3
        msg.body_config = [0.0,1.0,0.2,0.1]
        msg.body_config_speed = [0.0,-1.0,-1.0,-1.0]
        msg.lights_raw = [0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255]
        msg.tail = -1.0
        msg.ear_rotate = [1.0,1.0]
        msg.body_vel.linear.x = 0.0
        msg.body_vel.angular.z = 0.2
        return msg

if __name__== '__main__':
    rospy.init_node( BadMode.EDGE_NAME)
    m = BadMode()
    m.ros_node_loop()
