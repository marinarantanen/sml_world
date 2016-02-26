# This file contains dummy/skeleton functions for the second scenario
# attributes a vehicle needs for scenario 2: intention in {1,2,3}, lane in {1,2,3}, travelled_distance
import math
import time
import vehiclesupervisorymodule
import Supervisor1
import Supervisor2
import Supervisor3

class GCDC_supervisory_layer:

    def __init__(self,vehicle_supervisory_module,scenario):

        self.scenario = scenario
        self.vehicle_supervisory_module = vehicle_supervisory_module
        self.abort = Abort(self.vehicle_supervisory_module)

        #Initialize the scenario class
        if scenario==1:
            self.scenario_module=Supervisor1.SupervisorScenario1(self.vehicle_supervisory_module)
        elif self.scenario==2:
            self.scenario_module=Supervisor2.SupervisorScenario2(self.vehicle_supervisory_module)
        elif self.scenario==3:
            self.scenario_module=Supervisor3.SupervisorScenario3(self.vehicle_supervisory_module)

    def set_scenario(self,scenario):
        self.scenario = scenario
        if scenario==1:
            self.scenario_module=Supervisor1.SupervisorScenario1(self.vehicle_supervisory_module)
        elif self.scenario==2:
            self.scenario_module=Supervisor2.SupervisorScenario2(self.vehicle_supervisory_module)
        elif self.scenario==3:
            self.scenario_module=Supervisor3.SupervisorScenario3(self.vehicle_supervisory_module)

    def step(self):
        self.scenario_module.execute_scenario()


class Abort:
 
    def __init__(self,vehicle_supervisory_module):
 
        self.vehicle_supervisory_module = vehicle_supervisory_module
        self.time = time.time()

        self.state = 1
 
        self.safety = []
        self.d_safe = 10# what is it? Should it be in the

    def set_safety_distance(self, d_safe):
        self.d_safe = d_safe
 
    def check_safety(self, ID):
        # delivers true if there is a hazard
        if self.check_safety1() and ID(0): self.safety.append(1)
        if self.check_safety2() and ID(1): self.safety.append(2)
        if self.check_safety3() and ID(2): self.safety.append(3)
        if self.check_safety4() and ID(3): self.safety.append(4)
        if self.check_safety5() and ID(4): self.safety.append(5)
        if self.check_safety6() and ID(5): self.safety.append(6)
        if self.check_safety7() and ID(6): self.safety.append(7)
        if self.check_safety8() and ID(7): self.safety.append(8)
        if self.check_safety9() and ID(8): self.safety.append(9)
        if self.check_safety10() and ID(9): self.safety.append(10)
        if self.check_safety11() and ID(10): self.safety.append(11)
        if self.check_safety12() and ID(11): self.safety.append(12)
        if self.check_safety13() and ID(12): self.safety.append(13)
        if self.check_safety14() and ID(13): self.safety.append(14)
 
        if not self.safety: return False
        else:
            self.state = 2
            return True

    def check_transitions(self):
        if self.state==1:
            return True
        elif self.state==2:
            return True
        elif self.state==3:
            return True
        elif self.state==4:
            return True
        elif self.state==5:
            return True
        elif self.state==6:
            return True
        elif self.state==7:
            return True
        elif self.state==8:
            return True
        elif self.state==9:
            return True
        elif self.state==10:
            return True
        elif self.state==11:
            return True

 
    def check_safety1(self): #driver press emergency button: low level control disabled
        if self.vehicle_supervisory_module.manual_button_pressed: return True
        else: return False
 
    def check_safety2(self): #driver diables control from HMI: either longitudinal or lateral or both control disabled
        if self.vehicle_supervisory_module.manual_button_pressed: return True
        else: return False
 
    def check_safety3(self): # driver press brake pedal: overrides the controllers and go to manual
        if self.vehicle_supervisory_module.brake_pressed: return True
        else: return False
 
    def check_safety4(self): # driver press gas pedal: temporarelly overrides the controllers, back on automatic when unpressed
        if self.vehicle_supervisory_module.gas_pressed: return True
        else: return False
 
    def check_safety5(self):# driver steers car: overrides the lateral controllers temporarelly (not implemented in GCDC due to car without lateral control)
        return False
 
    def check_safety6(self): # system failure- logical failure: go to manual
        return False #not implemented yet
 
    def check_safety7(self):# sensor failure: go to manual
        if self.vehicle_supervisory_module.perception_error: return True
        else: return False
 
    def check_safety8(self): # control failure: go to manual
        if self.vehicle_supervisory_module.control_error: return True
        else: return False
 
    def check_safety9(self): # vehicle arrives to late or to early to the CZ in scenario 2: abort safely
        return False # not implemented yet
 
    def check_safety10(self):# vehcile 1 does not reach the intersection in calculated time in scenario 2 (will cause crash if not handled)
        return False # not implemented yet
 
    def check_safety11(self):# CAM indicates that another car is on manual driving: go to manual
        if self.vehicle_supervisory_module.manual_button_pressed: return True
        else: return False
         
    def check_safety12(self):# safety distance is not fulfilled
        if self.vehicle_supervisory_module.target_distance<self.d_safe: return True
        else: return False
 
    def check_safety13(self):# communication failure - loss of package/delay
        return False # not implemented yet
 
    def check_safety14(self): # time in state exceeds allowed time
        return False # not implemented yet



        """
    already done in upper class. but these functions need to be extended and
    it should be possible to mix the control inputs later on
    def activate_controller(self, ID):
        if ID>1 and ID<8:
            if ID==1:
                #CACCs
            elif ID==2:
                #VCACC
            elif ID==3:
                #CC
            elif ID==4:
                #OA
            elif ID==5:
                #CA
            elif ID==6:
                #approaching competition zone
            elif ID==7:
                #lateral platooning?!
            return True
        else
            return False

def deactivate_controller(self, ID):
    if ID>1 and ID<8:
        if ID==1:
            #CACC
        elif ID==2:
            #VCACC
        elif ID==3:
            #CC
        elif ID==4:
            #OA
        elif ID==5:
            #CA
        elif ID==6:
            #approaching competition zone
        elif ID==7:
            #lateral platooning?!
        return True
    else
        return False
        """

"""should be done in vehiclesupervisory module
def send_HMI(self, ID):
    if ID>1 and ID<8:
        if ID==1:
            #STOM
        elif ID==2:
            #Control mode (like controllers and automatic/manual)
        elif ID==3:
            #RSU scenario init information
        elif ID==4:
            #RSU start/end scenario information
        elif ID==5:
            #Emergency vehicle approaching -> information where to move
        elif ID==6:
            #Emergency vehicle left-> go back
        elif ID==7:
            #safety/abort
        return True
    else
        return False
        """

"""
def calculate_intervehicle_distance(self, car1, car2, ID):
    # Calc (virtual) intervehicle distance between two cars depending on ID,
    # where ID=1 calculates IVD and ID=2 calculates VIVD
    if ID>0 and ID<3
        if ID==1:
            #calc IVD
        elif ID==2:
            #calc VIVD
        return True
    else
        return False
        """