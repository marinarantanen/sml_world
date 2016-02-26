
import math
class SupervisorScenario2:
    # This file contains dummy/skeleton functions for the second scenario
    # attributes a vehicle needs for scenario 2: intention in {1,2,3}, lane in {1,2,3}, travelled_distance, state [x,y,alhpa]

    def __init__(self,data):
        self.dummy=0
        # variables needed
        self.current_state=1 # set the state to the first one in the automaton when initializing
        # intersection variables
        self.r_cz=None # radius of competition zone
        self.w_p=None # width of primary road
        self.w_s=None # width of secondary road
        self.psi_cz=None # angle between primary and secondary road (for scenario 2 90 degrees)
        # host variables
        self.host_state=None
        self.host_intention=None
        self.host_lane=None
        self.host_travelled_distance=None
        # target variables
        self.target_state=None
        self.target_intention=None
        self.target_lane=None
        self.target_travelled_distance=None

        # auxiliary variables for cancellation
        self.cancel_help=0

        self.timeout=0.0

        self.vehicle_supervisory_module=data


    def execute_scenario(self):
        # This function is for executing the whole scenario and setting flags
        #self.processData(data)# process data function
        # check safety
        self.check_transitions() # Check if the new received data leads to a transition or not
        self.update_flags_variables()# set flags according to state and update variables (like travelled distance) mazbe function called state update
        # return the important flags, maybe they are already updated
        return True


    def processData(self,data):
        # function for processing the data received by the main supervisory function, which is extracting the necessary information
        # about the target vehicle its intention and so on to update the private variables of SupervisorScenario2
        # i.e. just extract messages with id for scenario 2
        self.dummy=0
        return True


    def check_transitions(self):# this is maybe a more general function
        # according to the current state the transitions are checked, and the state is changed if a transition fires
        if self.current_state==1:
            self.current_state==2
        elif self.current_state==2: # if you are in the waiting state...
            if self.start_flag: # ... and the start flag is set...
                self.current_state=3 #... fire transition to the third state
        elif self.current_state==3:
            return True
        elif self.current_state==4:
            return True
        elif self.current_state==5: # if you are in state 5 of scenario 2, which is the VCACC state
            cancel_flag=self.cancel_VCACC(self.target_state, self.target_intention, self.target_lane)
            if cancel_flag==1:
                 self.current_state=8 # cancel to CACC
            elif cancel_flag==2:
                 self.current_state=6 # cancel to CC
        elif self.current_state==6:
            # set CC controller and desired velocity
            return True
        elif self.current_state==7:
            # set CACC controller
            return True
        elif self.current_state==8:
            return True
        elif self.current_state==9:
            return True
        elif self.current_state==10:
            return True
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
        elif self.current_state==8:
            return True
        elif self.current_state==9:
            return True
        elif self.current_state==10:
            return True
        else:
            return True


    def cancel_VCACC(self,target_state,target_intention,target_lane):
        # implement table 4.1 in D2.2 as if conditions to know which cancellation method should be used
        # after checking the right if condition, see if the cancel condition is fulfilled and if yes return true otherwise false
        # example for cancelling VCACC for vehicle 2, where we assume here that the coordinates are with respect to the IRF

        # maybe use two different flags, like cancel_to_cacc and cancel_to_cc for distinguishing where to go after VCACC in automton we use a cancel flag
        if self.host_lane==2 and self.target_lane==1 and self.host_intention==1 and self.target_intention==2:
            new_value=-1*math.sin(self.state[2]*math.pi/180)*(target_state[1]-self.state[1])+math.cos(self.state[2]*math.pi/180)*(target_state[1]-self.state[1])
            if new_value*self.cancel_help<0:
                return 2
            else:
                self.cancel_help=new_value
                return 0
        #if self.
        # random comment



    def calculate_scaled_travelled_distance(self,x_k,y_k):
        # This functions calculates the travelled distance inside the competition zone, the calculations for that can be found in
        # section 4.1.3-4.1.5 in D2.2. Problems could be that we need to scale the coordinates back to the vehicle coordinate system
        # intention {1,2,3}={straight,left,right}
        if self.intention==1: # for straight intention the travelled distance is equal to x_k in the fixed coordinate system where the car entered the CZ
            if self.lane==2:
                travelled_distance=x_k
            elif self.lane==3:
                travelled_distance=x_k
            scaled_travelled_distance=travelled_distance # for the straight trajectory no scaling is needed

        elif self.intention==2: # left turn
            if self.lane==1:
                w_o=self.w_s
                w_f=self.w_p
                psi_l=math.radians(self.psi_cz) # should be in radians
            elif self.lane==3:
                w_o=self.w_p
                w_f=self.w_s
                psi_l=math.pi-math.radians(self.psi_cz) # should be in radians
            r_l=0.75*w_f
            a_l=0.5*w_f/math.sin(psi_l)-(r_l-0.25*w_o)*math.cos(psi_l)/math.sin(psi_l)
            b_l=-0.5*w_f*math.cos(psi_l)/math.sin(psi_l)+(r_l-0.25*w_o)/math.sin(psi_l)
            c_l=r_l*psi_l
            d_o=self.r_cz-a_l
            d_f=self.r_cz-b_l
            d_l=math.sqrt(math.pow(x_k-r_l*math.sin(psi_l)-d_o,2)+math.pow(y_k-r_l*(1-math.cos(psi_l)),2))
            psi_l_xy=math.atan2(x_k-d_o,r_l-y_k)
            if x_k<=d_o:
                travelled_distance=x_k
            elif x_k>d_o and y_k<=r_l*(1-math.cos(psi_l)):
                travelled_distance=d_o+r_l*psi_l_xy
            elif x_k>d_o and y_k>r_l*(1-math.cos(psi_l)):
                travelled_distance=d_o+r_l*psi_l+d_l
            # up to here the travelled distance is calculated and from now on we will scale it
            if travelled_distance<=d_o:
                scaled_travelled_distance=travelled_distance # until we reach the turn the travelled distance does not need to be scaled
            elif travelled_distance>d_o and travelled_distance<=d_o+c_l:
                scaled_travelled_distance=d_o+(a_l+b_l)/c_l*(travelled_distance-d_o)
            elif travelled_distance>d_o+c_l:
                scaled_travelled_distance=travelled_distance+a_l+b_l-c_l

        elif self.intention==3: # right turn
            if self.lane==1:
                w_o=self.w_s
                w_f=self.w_p
                psi_r=math.pi-math.radians(self.psi_cz)
            elif self.lane==2:
                w_o=self.w_p
                w_f=self.w_s
                psi_r=math.radians(self.psi_cz)
            r_r=0.25*w_f
            a_r=0.5*w_f/math.sin(psi_r)-(r_r+0.25*w_o)*math.cos(psi_r)/math.sin(psi_r)
            b_r=-0.5*w_f*math.cos(psi_r)/math.sin(psi_r)+(r_r+0.25+w_o)/math.sin(psi_r)
            c_r=r_r*psi_r
            d_o=self.r_cz-a_r
            d_f=self.r_cz-b_r
            psi_r_xy=math.atan2(x_k-d_o,r_r+y_k)
            d_r=math.sqrt(math.pow(x_k-r_r*math.sin(psi_r)-d_o,2)+math.pow(y_k+r_r*(1-math.cos(psi_r)),2))

            if x_k<=d_o:
                travelled_distance=x_k

            elif x_k>d_o and y_k>-1*r_r*(1-math.cos(psi_r)):
                travelled_distance=d_o+psi_r_xy*r_r

            elif x_k>d_o and y_k<+-1*r_r*(1-math.cos(psi_r)):
                travelled_distance=d_o+r_r*psi_r+d_r

            # up to here the travelled distance is calculated and from now on we will scale it
            if travelled_distance<=d_o:
                scaled_travelled_distance=travelled_distance # until we reach the turn the travelled distance does not need to be scaled

            elif travelled_distance>d_o and travelled_distance<=d_o+c_r:
                scaled_travelled_distance=d_o+(a_r+b_r)/c_r*(travelled_distance-d_o)

            elif travelled_distance>d_o+c_r:
                scaled_travelled_distance=travelled_distance+a_r+b_r-c_r

        return scaled_travelled_distance


    def transform_coordinates(self):
        # transforms coordinates to the fixed coordinate systems of each lane for checking the travelled distances according to D2.2
        if self.host_lane==1:
            x_transformed=self.host_state[1]+self.r_cz*math.sin(self.psi_cz)-0.25*self.w_s*math.cos(self.psi_cz)
            y_transformed=-1*self.host_state[0]+self.r_cz*math.cos(self.psi_cz)+0.25*self.w_s*math.sin(self.psi_cz)
            return x_transformed,y_transformed
        elif self.host_lane==2:
            x_transformed=self.host_state[0]+self.r_cz
            y_transformed=self.host_state[1]+self.w_p/4
            return x_transformed,y_transformed
        elif self.host_lane==3:
            x_transformed=self.r_cz-self.host_state[0]
            y_transformed=self.w_p/4-self.host_state[1]
            return x_transformed,y_transformed

    # def scaled_travelled_distance(self):
    #     # this function scales the travelled distance, velocity and acceleration of a vehicle to project vehicle on a straight lane
    #     # question here is if we need to scale the distance of ourselves and/or the target as well, because the target might send the unscaled distance
    #     x,y=self.transform_coordinates()
    #     travelled_distance=self.calculate_scaled_travelled_distance(x,y)
    #     if self.host_intention==1: # straight
    #         scaled_travelled_distance = travelled_distance # just return the unscaled variables because we are already on a straight lane
    #     elif self.host_intention==2: # left turn
    #
    #     elif self.host_intention==3: # right turn
    #         # return the scaled variables according to section 4.2.2 in D2.2
    #         scaled_travelled_distance = travelled_distance # just a dummy
    #     return scaled_travelled_distance


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
        self.dummy=0
        return True


    def get_reference_signal(self,intention):
        # used to get a reference signal for the vehicles in CC
        # it should be possible to connect it with the SML world in an intelligent way
        self.dummy=0
        return True


    def get_target_data(self):
        # What is my purpose? Maybe the same as processData()
        self.dummy=0
        return True




