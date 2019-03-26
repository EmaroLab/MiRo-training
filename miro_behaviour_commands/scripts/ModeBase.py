#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Bool
import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

class StateMode():
    NODE_RATE = 200 # Hz
    ACTIVATION_DEFAULT = 'False' # active publisher at start up
    BASE_PUBLISHING_PATH = '/miro_state_controller/' # TODO duplicated in `command_State_manager.py`
    #EDGE_NAME = 'bad' should be implemented, TODO duplicated in `command_State_manager.py`

    def __init__(self):   
	# handle args
        self.miro_root_topic_name = ""
        for arg in sys.argv[1:]:
            f = arg.find(':=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f+2:]
            if key == "robot":
                self.miro_root_topic_name = '/miro/' + val + '/'
            else:
                rospy.loginfo("argument not recognised \"" + arg + "\"")
        if self.miro_root_topic_name == "":
            rospy.logerr("argument \'robot:=\' not given for mode " + self.EDGE_NAME)
        else:
            ## Publisher to the topic /miro_bad a message of type platform_control which corresponds to the "Bad" action.
            self.activation_topic_name = self.miro_root_topic_name + 'platform/control';
            self.controll_publisher = rospy.Publisher( self.activation_topic_name, platform_control, queue_size=0)
            # subscribe to activation topic from state machine
            rospy.Subscriber( self.BASE_PUBLISHING_PATH + str(self.get_edge_name()), Bool, self.state_controller_callback)
            # define shouldPublish flag
            self.state_activated = self.ACTIVATION_DEFAULT
    
    ## Function that sets the parameters of the structure platform_control corresponding to action "Bad".
    # Remember to call this function to activate the node
    def ros_node_loop(self):
        r = rospy.Rate(self.NODE_RATE)
        msg = platform_control()
        should_log = True
        while not rospy.is_shutdown():
            # if should publish (told by the state machine) 
            if self.state_activated == True:
                # set Miro behavior
                msg = self.set_miro_controll(msg)
                # publish Miro behavior
                self.controll_publisher.publish(msg)
                if should_log == True:
                    rospy.loginfo('sending \'' + str(self.EDGE_NAME) + '\' data to topic ' + self.activation_topic_name)
                    should_log = False
            else:
                self.reset_miro_controll(msg) # do not outomatically pusblish on reser
                if should_log == False:
                    rospy.loginfo('stop sending \'' + str(self.EDGE_NAME) + '\' data')
                    should_log = True
            r.sleep()

    # callback to state machine
    def state_controller_callback(self,msg):
        self.state_activated = msg.data

    # get the node name to defile listening topic to the state machine during class construction
    def get_edge_name(self):
        rospy.loginfo('should return a constant name used in the topic to the state machine. To be overwritten!') # TODO duplicated in `command_State_manager.py`
    
    # interface to be implemented
    def set_miro_controll(self,msg):        
        rospy.loginfo('should fill the msg parameter (continuously published when active) for topic ..Miro../platform/controll')

    # interface to be implemented
    def reset_miro_controll(self,msg):        
        return 0
#        rospy.loginfo('should fill the msg parameter (not automatically published; @see:self.controll_publisher) for topic ..Miro../platform/controll')

