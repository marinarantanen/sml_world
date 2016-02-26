# This file contains dummy/skeleton functions for the first scenario0
import math

#the following functions activates necessary controller agents for each maneuver
#need something to deactivate controllers as well
class Scenario1Maneuvers:
    def __init__(self):

    def platooning(self,agents,controller_inputs):
        activate_controller(agents,controller_inputs)

    def merging(self,agents,controller_inputs):
        activate_controller(agents,controller_inputs)

    def gap_making(self,agents,controller_inputs):
        activate_controller(agents,controller_inputs)


#the following functions perform the pair-up in order to do gap-making and merging
class Pairing:
    def __init__(self):

    def B2A(self):
        if fwd_MIOL != platoon_leader and fwd_MIOL != platoon_tail
            B2A_partner=fwd_MIOL

    def A2B(self):
        A2B_partner=vehicle_ID(B2A_partner-1) #I guess this is not how the code works, but the idea is just to take the predecessor of the B2A_partner as A2B_partner
