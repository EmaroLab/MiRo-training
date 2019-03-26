#!/usr/bin/env python2.7

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool

import time
from threading import Thread, Lock

import signal
import sys

#import speech_recognition as sr # requires `pip install SpeechRecognition` and `PyAudio 0.2.11`

## \file command_state_manager.py
## \brief This script implements a finite state machine to manage the Miro's behaviour. 
##

# separate thread to listen at the comands coming from the users
class CommandListener:
    def __init__(self):
        self.command_thread = Thread(target=self.listening)
        self.command = ""
        self.kill = False
        self.mutex = Lock()
        self.command_thread.start()        

   # example for listening trhough keyboard
    def listening(self):
        while( not self.kill):
            user_text = raw_input("Enter something:")
            print("You entered:" + str(user_text))
            self.mutex.acquire()
            try:
                self.command = user_text
            finally:
                self.mutex.release()                

    # listing from microphone 
#    def listening(self):
#        while( not self.kill):
#            r = sr.Recognizer()                 # initialize recognizer
#            with sr.Microphone() as source:     # mention source it will be either Microphone or audio files.
#                rospy.loginfo("Miro is listening :")
#                audio = r.listen(source)        # listen to the source
#                rospy.loginfo("Miro is busy")
#                try:
#                    text = r.recognize_google(audio)    # use recognizer to convert our audio into text part.
#                    rospy.loginfo("You said: " + text)#format(text))
#                    sep=text.split()       
#                except:
#                    rospy.loginfo("Sorry could not recognize your voice")
#            #self.mutex.acquire()
#            #try:
#             #    self.command = user_text
#            #finally:
#            #    self.mutex.release()    
    
    def terminate(self):
        self.kill = True
        
    def join(self):
        self.command_thread.join()
        
    def get_command(self):
        self.mutex.acquire()
        try:
            out = self.command
            self.command = ""
            return out
        finally:
            self.mutex.release()                

# class extended by all the states below
class StateBase(smach.State):

    TIME_OUT = 5 # seconds
    CHECK_FREQ = 0.2 # seconds
    REACTIVE = True # if FALSE it waits before to send command
    DELAY_COMMAND = 2 # in seconds
    TRANSITION_TO_FROM = '2' # as string from meaning 'to' in the transition anme
    BASE_PUBLISHING_PATH = '/miro_state_controller/'
  
    # derived class must contain constants
    # STATE_NAME
    # EDGE_NAME (the name x (not capitalised))

    def __init__(self,listener):
        # set state machine
        self.listener = listener 
        smach.State.__init__(self, outcomes=self.get_outcomes())
        # set boolean publisher to activation ROS node
        self.activation_topic_name = self.BASE_PUBLISHING_PATH + self.EDGE_NAME
        self.publisher = rospy.Publisher( self.activation_topic_name, Bool, queue_size=0)

    # it must retyrb the transiction sub-name X for `self.transition_to + X`
    def wat_and_transit(self):      
        now = time.time()
        while( time.time() - now < self.TIME_OUT): # untill time out
            time.sleep(self.CHECK_FREQ) # wait for saving computation resources
            command = self.listener.get_command() # get the user command
            if not self.REACTIVE: # if set wait before to apply command
                time.sleep(self.DELAY_COMMAND)
            # set transition for all the connected states
            if command == 'good': 
                return Good.EDGE_NAME
            elif command == 'bad':
                return Bad.EDGE_NAME
            elif command == 'sleep':
                return Sleep.EDGE_NAME
            elif command == 'play':
                return Play.EDGE_NAME
            #else:
            #    print("Command not understood " + command)
        rospy.loginfo("TIME_OUT")
        return ""

    # smach interface
    def execute(self, userdata):
#        rospy.loginfo('Executing state ' + str(self.STATE_NAME))
        self.start_command() # start behaviour to implement
        user_command = self.wat_and_transit(); # check for user command or time out
        transition = self.transition_to(Demo.EDGE_NAME) # initialise time out transition
        if( user_command != ""):  # not time out
            transition = self.transition_to(user_command) # set user transition
        self.stop_command() # stop behaviour to implement
        return transition


    # the transition name fragment x2...
    def transition_to(self,to_edge_name):
        return  self.EDGE_NAME + self.TRANSITION_TO_FROM  + to_edge_name # e.g., x2edge_name

    # the transition name fragment ...2x [NOT USED !!!]
    def transition_from(self,from_edge_name): 
        return from_edge_name + TRANSITION_TO_FROM + self.EDGE_NAME # e.g., edge_name2x

    # automatically subscribe TRUE to related state ROS node in the topic `self.BASE_PUBLISHING_PATH + self.EDGE_NAME`
    def start_command(self):
	    rospy.loginfo('Start command implementation ' + self.STATE_NAME + ' implemented on topic ' + self.activation_topic_name)
	    self.publisher.publish(True)

    # automatically subscribe FALSE to related state ROS node in the topic `self.BASE_PUBLISHING_PATH + self.EDGE_NAME`
    def stop_command(self):
	    rospy.loginfo('Stop command implementation ' + self.STATE_NAME + ' implemented on topic ' + self.activation_topic_name)
	    self.publisher.publish(False)

    # required interfaces for the derived classes (called on constructor, those should return constants) (called during machine building)
    def get_outcomes(self):
        rospy.loginfo('Machine outcomes set-up, to be implemented in derived classes')

    def get_transitions(self):
        rospy.loginfo('Machine transition set-up, to be implemented in derived classes')
        
    def get_transitions(self):
        rospy.loginfo('return the name of the activation/disactivation topic')

    def get_activation_topic_name(self):
        rospy.loginfo('return the name of the topic for activating bheaviour, to be implemented')


# define state Demo
class Demo(StateBase):
    # const required from StateBase
    STATE_NAME = 'DEMO'
    EDGE_NAME = STATE_NAME.lower()

    # function required by the StateBase class interface (called during machine building)
    def get_outcomes(self):
        return [ self.transition_to( self.EDGE_NAME), # 'demo2demo'
                 self.transition_to( Good.EDGE_NAME), # 'demo2good'
                 self.transition_to( Bad.EDGE_NAME), # 'demo2bad'
                 self.transition_to( Sleep.EDGE_NAME), # 'demo2sleep'
                 self.transition_to( Play.EDGE_NAME)
               ]

    # function required by the StateBase class interface (called during machine building)
    def get_transitions(self):
        return { self.transition_to( self.EDGE_NAME) : self.STATE_NAME, # 'demo2demo':'DEMO',
                 self.transition_to( Good.EDGE_NAME) : Good.STATE_NAME, #'demo2good':'GOOD',
                 self.transition_to( Bad.EDGE_NAME) : Bad.STATE_NAME, # 'demo2bad':'BAD',
                 self.transition_to( Sleep.EDGE_NAME) : Sleep.STATE_NAME, #'demo2sleep':'SLEEP'
                 self.transition_to( Play.EDGE_NAME) : Play.STATE_NAME 
               }

# define state Good
class Good(StateBase):
    # const required from StateBase
    STATE_NAME = 'GOOD'
    EDGE_NAME = STATE_NAME.lower()

    # function required by the StateBase class interface (called during machine building)
    def get_outcomes(self):
        return [ self.transition_to( self.EDGE_NAME), # 'good2good'
                 self.transition_to( Demo.EDGE_NAME), # 'good2demo'
                 self.transition_to( Bad.EDGE_NAME), # 'good2bad'
                 self.transition_to( Sleep.EDGE_NAME), # 'good2sleep'
                 self.transition_to( Play.EDGE_NAME)
               ]

    # function required by the StateBase class interface (called during machine building)
    def get_transitions(self):
        return { self.transition_to( self.EDGE_NAME) : self.STATE_NAME, # 'good2good':'GOOD',
                 self.transition_to( Demo.EDGE_NAME) : Demo.STATE_NAME, #'good2demo':'DEMO',
                 self.transition_to( Bad.EDGE_NAME) : Bad.STATE_NAME, # 'good2bad':'BAD',
                 self.transition_to( Sleep.EDGE_NAME) : Sleep.STATE_NAME, #'good2bad':'SLEEP' 
                 self.transition_to( Play.EDGE_NAME) : Play.STATE_NAME
               }

# define state Bad
class Bad(StateBase):
    # const required from StateBase
    STATE_NAME = 'BAD'
    EDGE_NAME = STATE_NAME.lower()

    # function required by the StateBase class interface (called during machine building)
    def get_outcomes(self):
        return [ self.transition_to( self.EDGE_NAME), # 'bad2bad'
                 self.transition_to( Demo.EDGE_NAME), # 'bad2demo'
                 self.transition_to( Good.EDGE_NAME), # 'bad2good'
                 self.transition_to( Sleep.EDGE_NAME), # 'bad2sleep'
                 self.transition_to( Play.EDGE_NAME)
               ]

    # function required by the StateBase class interface (called during machine building)
    def get_transitions(self):
        return { self.transition_to( self.EDGE_NAME) : self.STATE_NAME, # 'bad2bad':'BAD',
                 self.transition_to( Demo.EDGE_NAME) : Demo.STATE_NAME, #'bad2demo':'DEMO',
                 self.transition_to( Good.EDGE_NAME) : Good.STATE_NAME, # 'bad2good':'GOOD',
                 self.transition_to( Sleep.EDGE_NAME) : Sleep.STATE_NAME, #'bad2sleep':'SLEEP' 
                 self.transition_to( Play.EDGE_NAME) : Play.STATE_NAME
               }

# define state Sleep
class Sleep(StateBase):
    # const required from StateBase
    STATE_NAME = 'SLEEP'
    EDGE_NAME = STATE_NAME.lower()


    # function required by the StateBase class interface (called during machine building)
    def get_outcomes(self):
        return [ self.transition_to( self.EDGE_NAME), # 'sleep2sleep'
                 self.transition_to( Demo.EDGE_NAME), # 'sleep2demo'
                 self.transition_to( Bad.EDGE_NAME), # 'sleep2bad'
                 self.transition_to( Good.EDGE_NAME), # 'sleep2good'
                 self.transition_to( Play.EDGE_NAME) 
               ]

    # function required by the StateBase class interface (called during machine building)
    def get_transitions(self):
        return { self.transition_to( self.EDGE_NAME) : self.STATE_NAME, # 'sleep2sleep':'SLEEP',
                 self.transition_to( Demo.EDGE_NAME) : Demo.STATE_NAME, #'sleep2demo':'DEMO',
                 self.transition_to( Good.EDGE_NAME) : Good.STATE_NAME, # 'sleep2good':'GOOD',
                 self.transition_to( Bad.EDGE_NAME) : Bad.STATE_NAME, #'sleep2bad':'BAD'
                 self.transition_to( Play.EDGE_NAME) : Play.STATE_NAME 
               }
            

# define state Sleep
class Play(StateBase):
    # const required from StateBase
    STATE_NAME = 'PLAY'
    EDGE_NAME = STATE_NAME.lower()


    # function required by the StateBase class interface (called during machine building)
    def get_outcomes(self):
        return [ self.transition_to( self.EDGE_NAME), # 'play2play'
                 self.transition_to( Demo.EDGE_NAME), # 'play2demo'
                 self.transition_to( Bad.EDGE_NAME), # 'play2bad'
                 self.transition_to( Good.EDGE_NAME), # 'play2good'
                 self.transition_to( Sleep.EDGE_NAME) 
               ]

    # function required by the StateBase class interface (called during machine building)
    def get_transitions(self):
        return { self.transition_to( self.EDGE_NAME) : self.STATE_NAME, # 'play2play':'PLAY',
                 self.transition_to( Demo.EDGE_NAME) : Demo.STATE_NAME, #'play2demo':'DEMO',
                 self.transition_to( Good.EDGE_NAME) : Good.STATE_NAME, # 'play2good':'GOOD',
                 self.transition_to( Bad.EDGE_NAME) : Bad.STATE_NAME, #'play2bad':'BAD' 
                 self.transition_to( Sleep.EDGE_NAME) : Sleep.STATE_NAME 
               }


# main
listener = CommandListener()
def state_machine_setup():
    rospy.init_node('miro_bheaviour_state_machine')
      
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # construct all states
    states = []
    states.append(Demo(listener))
    states.append(Good(listener))
    states.append(Bad(listener))
    states.append(Sleep(listener))
    states.append(Play(listener))

    # Open the container
    with sm:
        # Add states to the container
        for i in range(len(states)):
            st = states[i]
            smach.StateMachine.add(st.STATE_NAME, st, st.get_transitions())
    
    # Configuration to show the finite state machine with `rosrun smach_viewer smach_viewer.py `
    sis = smach_ros.IntrospectionServer('miro_state_command', sm, '/MIRO_STATES')
    sis.start()   
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()

def signal_handler(sig, frame):
    listener.terminate()
    listener.join()
    print('Bye Bye')
    try:
        sys.exit(0)
    except: 
        print("I am going to close the programm, please wait for a change of state!")
   

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    state_machine_setup()
