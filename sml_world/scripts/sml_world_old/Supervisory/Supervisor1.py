
import math
class SupervisorScenario1:
    # This file contains dummy/skeleton functions for the second scenario
    # attributes a vehicle needs for scenario 2: intention in {1,2,3}, lane in {1,2,3}, travelled_distance

    def __init__(self,data):
        self.dummy=0
        self.current_state=1 # set the state to the first one in the automaton when initializing
        self.vehicle_supervisory_module=data


    def execute_scenario(self):
        # This function is for executing the whole scenario and setting flags
        # process data function
        # check safety
        #print(self.vehicle_supervisory_module.position)
        self.check_transitions() # Check if the new received data leads to a transition or not
        # set flags according to state and update variables (like travelled distance) mazbe function called state update
        # return the important flags
        return True


    def processData(self,data):
        # function for processing the data received by the main supervisory function, which is extracting the necessary information
        # about the target vehicle its intention and so on to update the private variables of SupervisorScenario2
        # i.e. just extract messages with id for scenario 2
        return True


    def check_transitions(self):# this is maybe a more general function
        # according to the current state the transitions are checked, and the state is changed if a transition fires
        if self.current_state==2: # if you are in the waiting state...
            if self.start_flag: # ... and the start flag is set...
                self.current_state=3 #... fire transition to the third state
        elif self.current_state==5: # if you are in state 5 of scenario 2, which is the VCACC state
            cancel_flag=self.cancel_VCACC(self.target_state, self.target_intention, self.target_lane)
            if cancel_flag==1:
                 self.current_state=8 # cancel to CACC
            elif cancel_flag==2:
                 self.current_state=6 # cancel to CC
        return True

    def update_flags_variables(self):
        # this function updates the flags and variables according to the state we are in
        if self.current_state==1:
            return True
        elif self.current_state==2:
            return True
        else:
            return True


    def cancel_VCACC(self,target_state,target_intention,target_lane):
        # implement table 4.1 in D2.2 as if conditions to know which cancellation method should be used
        # after checking the right if condition, see if the cancel condition is fulfilled and if yes return true otherwise false
        # example for cancelling VCACC for vehicle 2, where we assume here that the coordinates are with respect to the IRF

        # maybe use two different flags, like cancel_to_cacc and cancel_to_cc for distinguishing where to go after VCACC in automton we use a cancel flag
        old_value=0.0 # should be a static variable
        if self.lane==2 and target_lane==1 and self.intention==1 and target_intention==2:
            new_value=-1*math.sin(self.state[2]*math.pi/180)*(target_state[1]-self.state[1])+math.cos(self.state[2]*math.pi/180)*(target_state[1]-self.state[1])
            if new_value*old_value<0:
                return True
            else:
                old_value=new_value
                return False


    def calculate_travelled_distance(self):
        # This functions calculates the travelled distance inside the competition zone, the calculations for that can be found in
        # section 4.1.3-4.1.5 in D2.2. Problems could be that we need to scale the coordinates back to the vehicle coordinate system
        if self.intention==1 and self.lane==2:
            self.travelled_distance=0
        elif self.intention==1 and self.lane==3:
            self.travelled_distance=0
        elif self.intention==2 and self.lane==1:
            self.travelled_distance=0
        elif self.intention==2 and self.lane==3:
            self.travelled_distance=0
        elif self.intention==3 and self.lane==1:
            self.travelled_distance=0
        elif self.intention==3 and self.lane==2:
            self.travelled_distance=0

            # random comment


    def scaling_coordinates(self):
        # this function scales the travelled distance, velocity and acceleration of a vehicle to project vehicle on a straight lane
        # question here is if we need to scale the distance of ourselves and/or the target as well, because the target might send the unscaled distance
        if self.intention==1:
            # just return the unscaled variables because we are already on a straight lane
            return True
        elif self.intention==2:
            # return the scaled variables according to section 4.2.2 in D2.2
            return True
        elif self.intention==3:
            # return the scaled variables according to section 4.2.2 in D2.2
            return True
        return True


    def calculate_VIVD(self):
        # calculates VIVD, which could also be done in scaling_coordinates
        return True


    def target_vehicle_assignment(self):
        # this function can be used for the target vehicle assignment, which is for scenario 2 quite easy I guess because the targets are just assigned according to the lanes
        if self.lane!=1:
            self.target_id=1
            self.platoon_id=2 # maybe not necessary
        else:
            self.target_id=None
            self.platoon_id=1 # maybe not necessary
        return True


    def reached_CZ(self):
        # Calculates if we reached the competition zone and returns true or false
        return True


    def get_reference_signal(self,intention):
        # used to get a reference signal for the vehicles in CC
        # it should be possible to connect it with the SML world in an intelligent way
        return True


    def get_target_data(self):
        # What is my purpose? Maybe the same as processData()
        return True




