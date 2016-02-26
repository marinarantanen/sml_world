import math

class SupervisorScenario3:
    # This file contains dummy/skeleton functions for the third scenario
    # random comment

    def __init__(self,data):
        self.dummy=0
        self.current_state=1 # set the state to the first one in the automaton when initializing
        self.vehicle_supervisory_module=data


    def execute_scenario(self):
        # This function is for executing the whole scenario and setting flags
        # process data function
        self.processData()
        # check safety
        self.check_transitions() # Check if the new received data leads to a transition or not
        self.update_flags_variables()# set flags according to state and update variables (like travelled distance) mazbe function called state update
        # return the important flags
        # random comment
        return True


    def processData(self):
        # function for processing the data received by the main supervisory function, which is extracting the necessary information
        # about the target vehicle its intention and so on to update the private variables of SupervisorScenario2
        # i.e. just extract messages with id for scenario 2
        # maybe needs a data input as well, but maybe not
        self.dummy=0
        return True


    def check_transitions(self):# this is maybe a more general function
        # according to the current state the transitions are checked, and the state is changed if a transition fires
        if self.current_state==1:
            return True
        elif self.current_state==2:
            return True
        elif self.current_state==3:
            return True
        elif self.current_state==4:
            return True
        elif self.current_state==5:
            return True
        elif self.current_state==6:
            return True
        elif self.current_state==7:
            return True


    def update_flags_variables(self):
        # this function updates the flags and variables according to the state we are in
        if self.current_state==1:
            return True
        elif self.current_state==2:
            return True
        elif self.current_state==3:
            return True
        elif self.current_state==4:
            return True
        elif self.current_state==5:
            return True
        elif self.current_state==6:
            return True
        elif self.current_state==7:
            return True
        else:
            return True