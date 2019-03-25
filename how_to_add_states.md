# how to add a state, for example: **sleep**, in (https://github.com/EmaroLab/MiRo-training/blob/arch/miro_behaviour_commands/scripts/command_state_manager.py)[command_state_manager.py]
- Copy paste a state class and name it as `Sleep` (for an example see below, assuming the `Good` and `Bad` states already implemented).
- Change `mach.State.__init__` to accept transitions as: `'sleep2demo','sleep2sleep','sleep2X¹','sleep2²', ...` where `X¹` and `Y²` goes for all the sates of the machine, excepts for `demo` and `sleep`.
- In the function `execute` of the `Sleep` class set:
  - `rospy.loginfo('Executing state SLEEP')`,
  - change return on time out condition to `'sleep2demo'`,
  - change else statement to `'sleep2' + transition`.
- In the `main()` function set:
  - add `smach.StateMachine.add('SLEEP', Sleep(listener), transitions={'sleep2sleep':'SLEEP','sleep2demo':'DEMO','sleep2X¹','sleep2²', ...)`,
  - add `'demo2sleep':'SLEEP'` in the `mach.StateMachine.add('DEMO'...` line
  - add `'X¹2sleep':'SLEEP'` and `'X²2sleep':'SLEEP'` and `...` for all the state already available (e.g., `Good` and `Bad`).
- Add `'demo2sleep'` on the constructor of the `Demo` class
- Add `'X¹2sleep'` and `X²2sleep` for each constructors of `X¹` and `X²` (e.g., `Good` and `Bad`).
- In the `switch_state` function add```
elif command == 'sleep':
	print("Triggering SLEEP")
	return command```


# Here an example of class for the new state `Sleep`
```
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
```

# Here a relative example of function `main()`
```
def main():
    ...
    with sm:
        # Add states to the container
        smach.StateMachine.add('DEMO', Demo(listener), transitions={'demo2demo':'DEMO','demo2good':'GOOD','demo2bad':'BAD','demo2sleep':'SLEEP'})
        smach.StateMachine.add('GOOD', Good(listener), transitions={'good2good':'GOOD','good2demo':'DEMO','good2bad':'BAD','good2sleep':'SLEEP'})
        smach.StateMachine.add('BAD', Bad(listener), transitions={'bad2bad':'BAD','bad2demo':'DEMO','bad2good':'GOOD','bad2sleep':'SLEEP'})
	smach.StateMachine.add('SLEEP', Sleep(listener), transitions={'sleep2sleep':'SLEEP','sleep2demo':'DEMO','sleep2good':'GOOD','sleep2bad':'BAD'})
    ...
```
