#!/usr/bin/env python2.7

import rospy
import smach
import smach_ros

import time
from threading import Thread, Lock

import signal
import sys

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

    def listening(self):
        while( not self.kill):
            user_text = raw_input("Enter something:")
            print("You entered:" + str(user_text))
            self.mutex.acquire()
            try:
                self.command = user_text
            finally:
                self.mutex.release()                
    
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

    TIME_OUT = 10 # in seconds
    
#    def __init__(self,listener):
#        self.listener = listener # defined in each derived class due to construction issues

    def switch_state(self):      
        now = time.time()
        while( time.time() - now < self.TIME_OUT):
            time.sleep(self.TIME_OUT/10) # checking frequency
            command = self.listener.get_command()
            if command == 'good':
                print("Triggering GOOD")
                return command
            elif command == 'bad':
                print("Triggering BAD")
                return command
            elif command == 'sleep':
                print("Triggering SLEEP")
                return command
            #else:
            #    print("Command not understood " + command)
        print("TIME_OUT")
        return ""
        
        

# define state Demo
class Demo(StateBase):
    def __init__(self,listener):
        self.listener = listener #super(StateBase,self).__init__(listener)
        smach.State.__init__(self, outcomes=['demo2demo','demo2good','demo2bad','demo2sleep'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state DEMO')
        transition = self.switch_state();
        if transition == "": # time out
            return 'demo2demo'
        else:
            return 'demo2' + transition

# define state Good
class Good(StateBase):
    def __init__(self,listener):
        self.listener = listener #super(StateBase,self).__init__(listener)
        smach.State.__init__(self, outcomes=['good2demo','good2good','good2bad','good2sleep'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GOOD')
        transition = self.switch_state();
        if transition == "": # time out
            return 'good2demo'
        else:
            return 'good2' + transition
        
# define state Bad
class Bad(StateBase):
    def __init__(self,listener):
        self.listener = listener #super(StateBase,self).__init__(listener)
        smach.State.__init__(self, outcomes=['bad2demo','bad2bad','bad2good','bad2sleep'])

    def execute(self, userdata):
        transition = self.switch_state();
        if transition == "": # time out
            return 'bad2demo'
        else:
            return 'bad2' + transition       

# define state Sleep
class Sleep(StateBase):
    def __init__(self,listener):
        self.listener = listener #super(StateBase,self).__init__(listener)
        smach.State.__init__(self, outcomes=['sleep2demo','sleep2sleep','sleep2good','sleep2bad'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SLEEP')
        transition = self.switch_state();
        if transition == "": # time out
            return 'sleep2demo'
        else:
            return 'sleep2' + transition
            

# main
listener = CommandListener()
def main():
    rospy.init_node('smach_example_state_machine')
      
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('DEMO', Demo(listener), transitions={'demo2demo':'DEMO','demo2good':'GOOD','demo2bad':'BAD','demo2sleep':'SLEEP'})
        smach.StateMachine.add('GOOD', Good(listener), transitions={'good2good':'GOOD','good2demo':'DEMO','good2bad':'BAD','good2sleep':'SLEEP'})
        smach.StateMachine.add('BAD', Bad(listener), transitions={'bad2bad':'BAD','bad2demo':'DEMO','bad2good':'GOOD','bad2sleep':'SLEEP'})        
	smach.StateMachine.add('SLEEP', Sleep(listener), transitions={'sleep2sleep':'SLEEP','sleep2demo':'DEMO','sleep2good':'GOOD','sleep2bad':'BAD'})        
    
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
    main()
