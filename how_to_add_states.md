# how to add a state, for example: PLAY
- add class `class Play(StateBase):` to `command_state_manager.py`
- change `STATE_NAME` (use a capitalized name)
- adjust the return value of `def get_outcomes(self):` to contain all combinations with other states
- add the returning field `self.transition_to( Play.EDGE_NAME)` to all `def get_outcomes(self):` functions of other states
- adjust the return value of `def get_transitions(self):` to contain all combinations with other states
- add the returning field `self.transition_to( Play.EDGE_NAME) : Play.STATE_NAME` to all `def get_transitions(self):` functions of other states
- in the `main()` function add the new state to the states array `states.append(Play(listener))`
- in the `def wat_and_transit (self)` function of `StateBase` add the relative case for user interface
```
elif command == 'play':
    return Play.EDGE_NAME
```
