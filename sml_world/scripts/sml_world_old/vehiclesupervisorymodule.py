import time
import smartvehicle
import vehiclecontrolmodule
import MessageModule
import threading

import sys
import math
from Constants import *

sys.path.append('Supervisory')
import GCDC_supervisory_layer

# This is the new supervisory module, that will work as an interface to our own code.
# writen by Carolina Eriksson
#11-2015


# TIPS:

# Starting velocity and lane is set in smartvehicle (currently lines 120-125)
# To set velocity use the self.setVelocity(new_velocity) function
# self.getVelocty() returns your current velocity, self.vehicle.desired_velocity points to the velocity the control module is trying to keep
# To change lane use the self.addLaneChange(new_lane) function
# To change/set a new pair partner, use the self.setPairPartner(target_id, fwd = True/False, bwd = True/False, new = True/False) function

# Things from perception_module:
# 	- self.vehicle.perception_grid is a dictionary with keys 'fl','fm','fr','l','r','rl','rm','rr', with values lists of vehicle ids in that part of the grid
#	- self.vehicle.perceived_objects is a dictionary with vehicle ids as keys, and lists of relative x_coord, relative y_coord and velocity as values
#	- self.vehicle.closest_objects is a dictionary with the same keys as perception_grid, but only contains the id of the vehicle closest to you in that part of the grid
#	- self.vehicle.sensor_readings is a list with each element as a list of readings. Those readings contain vehicle id, x, y, speeds etc

# right lane is 2026 points, center lane is 1899 points, so to keep the vehicles approximately at the same distance from each other (over all) a center lane vehicle should have 0.937 of the right lane vehicle velocity



class VehicleSupervisoryModule:
	"This class is the Platooning Vehicle class used by the Platooning Manager Module"
	def __init__(self, vehicle):

		## gains access to the vehicle variables
		self.vehicle = vehicle
		#time for the initiating of supervisory
		self.module_start_time = time.time()

		######################################################new stuff
		#initialize all attributes GCDC needs
		self.target_distance=None
		self.ACCC_target_id=None

		# Most important object in the front, currently necessary for messageModule
		self.MIO_id = None
		self.MIO_range = None
		self.MIO_bearing = None
		self.MIO_range_rate = None

		# states of the vehicle
		self.velocity=None
		self.position=None
		self.acceleration=None
		self.orientation=None

		# errors
		self.perception_error=False
		self.control_error=False
		self.communication_error=False

		self.time_headway = None	# time to vehicle in front, messages
		self.cruise_speed = None	# nominal speed of platoon, messages
		self.travelled_CZ = None	# Distance travelled in Competition zone

		self.current_lane=None
		self.distance_to_intersection=None
		self.intersection_position=None #should be the coordinates of the intersection
		self.type_controller =None # keeping track of which controller we use
		self.platoon_id = None
		self.intention	= None # where are we heading
		self.lane_entering_CZ = None
		self.intersection_vehicle_counter = None
		#HMI variables
		self.gas_pressed=False
		self.brake_pressed=False
		self.manual_button_pressed=False

		#message attributes
		self.STOM=False
		self.intersection_vehicle_counter=None

		self.dummy_time = time.time()
		self.reference_time = time.time()
		
		self.merge_flag_request	= False
		self.merging_flag = False

		self.reference_time	= None					#id 37
		self.event_type = None

		#initialize GCDC
		self.GCDC_module=GCDC_supervisory_layer.GCDC_supervisory_layer(self,self.vehicle.sml_world.scenario)

		#initialize messagemodule
		self.message_module=MessageModule.MessageModule(self.vehicle, self)
		self.message_incoming=None

		#initialize threads for communication
		# self.vehicle.start_thread(self.message_frequency,args=([10]))
		# self.vehicle.start_thread(self.message_frequency,args=([1]))
		# self.vehicle.start_thread(self.message_handler, args=([]))




		self.state = StateScen1.INIT
		self.vehicle.ready = False


		print "VehicleSupervisoryModule started"


	def step(self):
		#run this call in thread at certain rate.
		self.update_values()
		#call GCDC step

		if time.time()-self.dummy_time>2.0:
			self.GCDC_module.step()
		## This is the step function that is running at each cycle of the thread in SmartVehicle

		#self.scen1()

	def scen1(self):
		nextState = self.state

		if time.time()-self.dummy_time>2.0:
			if self.is_car_ready():
				if self.vehicle.perception_grid!=[]:
					if 'fm' in self.vehicle.closest_objects:
						id = self.vehicle.closest_objects['fm']
						if isinstance(id,int):
							if id!=self.vehicle.control_module.ACCC_target_id:
								self.setACCCTarget(id, 20.0)
								self.vehicle.ready = True
					else:
						self.vehicle.ready = True


		if self.state == StateScen1.WAIT_PAIR:
			self.receive_pair_from_B()

		## -- Transitions --
		if self.state == StateScen1.INIT and (self.vehicle.ready) :
			nextState = StateScen1.WAIT_RSU;

		# A vehicle
		if self.state == StateScen1.WAIT_RSU and self.get_lane()==Lane.A:
			nextState = StateScen1.WAIT_FWD_PAIR

		if self.state == StateScen1.WAIT_FWD_PAIR and (self.do_i_have_a_miol_fwd() and self.is_miol_fwd_not_paired()):
			nextState = StateScen1.B_PAIRUP

		if self.state == StateScen1.B_PAIRUP: # need to add the merging zone
			self.pairupB()
			nextState = StateScen1.GAP_MAKING

		if self.state == StateScen1.GAP_MAKING and self.is_gap_made():
			self.set_STOM(True)
			nextState = StateScen1.A_IS_MERGING

		if self.state == StateScen1.A_IS_MERGING and self.is_fwd_pair_merged():
			self.reset_pairing()
			self.reset_messages()
			nextState = StateScen1.WAIT_FWD_PAIR

		# B vehicle
		if self.state == StateScen1.WAIT_RSU and self.get_lane()==Lane.B:
			nextState = StateScen1.WAIT_PAIR

		if self.state == StateScen1.WAIT_PAIR and self.vehicle.bwd_pair_partner != False:
			nextState = StateScen1.WAIT_MERGE

		if self.state == StateScen1.WAIT_MERGE and self.vehicle.platoon_leader:
			self.pairupA()
			nextState = StateScen1.WAIT_STOM

		if self.state == StateScen1.WAIT_STOM and self.bwd_STOM():
			self.merge()
			nextState = StateScen1.MERGING

		if self.state == StateScen1.MERGING and self.am_i_merged():
			self.reset_pairing()
			nextState = StateScen1.WAIT_FWD_PAIR

		self.state = nextState;



	def update_values(self):
		##help-function that gather all values needed by the supervisory layer. Writen by Carolina Eriksson 11-2015
		#do the GCDC need anything else?
		
		#state of vehicle
		self.velocity=self.get_velocity()
		self.position=[self.vehicle.x, self.vehicle.y]
		self.acceleration=None
		self.orientation=self.vehicle.yaw
		self.current_lane=self.vehicle.getCurrentLaneTrajectoryString()#current lane

		####----values from the perception layer----
		if hasattr(self.vehicle,'perception_module'):
			#self.target_distance=self.vehicle.perception_module.getLongitudinalDistanceToTarget(self.ACCC_target_id) #distance to target
			#self.distance_to_intersection=self.vehicle.perception_module.distance(self.position, self.intersection_position)#distance to intersection center if scenario 2 is run -needed?
			if not ('fm' in self.vehicle.closest_objects):
				self.vehicle.platoon_leader = True
			else:
				self.vehicle.platoon_leader = False

		#####---values from control layer----
		#if hasattr(self.vehicle, 'control_module'):
		#	self.ACCC_target_id=self.vehicle.control_module.ACCC_target_id	#target ID

		####----- Error Messages----
		self.perception_error=self.vehicle.getError()[0]#error message state of perception (true/false)
		self.control_error=self.vehicle.getError()[1]#error message state of control (true/false)
		#self.communication_error is updated in communication below if changed. (true/false)

		####----HMI----
		#self.gas_pressed=self.vehicle.getHMI(gas)
		#self.brake_pressed=self.vehicle.getHMI(brake)
		#self.manual_button_pressed=self.vehicle.getHMI(manual)


	def is_car_ready(self):
		return hasattr(self.vehicle,'perception_module') and \
			   hasattr(self.vehicle,'control_module')
###########################################################################functions for GCDC to call for setting/sending messages and giving control commands

	def set_leader_flag(self, bool_value): #message
		self.vehicle.platoon_leader=bool_value

	def set_target_id(self, ID):#message (MIO)
		self.vehicle.control_module.ACCC_target_id=ID

	def set_fwd_pair_partner(self, ID): #message
		self.vehicle.fwd_pair_partner=ID

	def set_bwd_pair_partner(self, ID): #message
		self.vehicle.bwd_pair_partner=ID

	#def set_merge_request_flag(): #message (unclear what it should be)

	#def set_merging_flag(): #message (unclear what it should be)

	def set_STOM(self, bool_value): #message (indicates that distance ahead is large enough for vehicle to enter)
		self.STOM=bool_value

	def set_tail_vehicle_flag(self, bool_value): #message
		self.vehicle.tail_vehicle=bool_value

	def set_intersection_vehicle_counter(self, counter): #message
		self.intersection_vehicle_counter=counter

	#def set_HMI_message(self, message):
		#print message to GUI

	def change_lane(self, new_lane):
		self.set_desired_lane(new_lane)
		self.vehicle.control_module.update_lane()

	def get_MIO_id(self):
		if 'fm' in self.vehicle.closest_objects:
			return self.vehicle.closest_objects['fm']
		else:
			return False
	####################################################################################functions to set some attributes (can be used if only one is to be chnaged instead of updating all as in functions below) Written by Sofie/Carolina

	def set_desired_velocity(self, desired_velocity):
		self.vehicle.desired_velocity = desired_velocity

	def set_desired_distance(self, desired_distance):
		self.vehicle.control_module.ACCC_desired_distance = desired_distance

	def set_desired_virtual_distance(self, desired_virtual_distance):
		self.vehicle.control_module.VACCC_desired_distance=desired_virtual_distance

	def set_type_controller(self, controller_type):
		self.type_controller = controller_type
		if controller_type == 'ACCC':
			self.vehicle.control_module.ACCC=True
			self.vehicle.control_module.VACC=False
			self.vehicle.control_module.OA=False
		elif controller_type== 'VACCC':
			self.vehicle.control_module.ACCC=False
			self.vehicle.control_module.VACC=True
			self.vehicle.control_module.OA=False
		elif controller_type=='CC':
			self.vehicle.control_module.ACCC=False
			self.vehicle.control_module.VACC=False    
			self.vehicle.control_module.OA=False
		elif controller_type=='OA':
			self.vehicle.control_module.ACCC=False
			self.vehicle.control_module.VACCC=False
			self.vehicle.control_module.OA=True
		
	def set_target_id(self, ID):
		self.vehicle.control_module.ACCC_target_id=ID


	def set_desired_OA_distance(self, distance):
		self.vehicle.control_module.OA_desired_distance = distance

	def set_desired_OA_target_ID(self, ID):
		self.vehicle.control_module.OA_target_id = ID

########################################################################################some functions gathering the functions above for easier calls. Written by Sofie

	def set_ACCC_control(self, ID, distance):
		self.set_target_id = ID
		self.set_type_controller = 'ACCC'
		self.set_desired_distance = distance

	def set_VACCC_control(self, ID, distance):
		self.set_target_id(ID)
		self.set_type_controller('VACCC')
		self.set_desired_virtual_distance(distance)

	def set_CC_control(self, speed):
		self.set_type_controller('CC')
		self.set_desired_velocity(speed)

	def set_OA_control(self, ID, distance):
		self.set_desired_OA_target_ID(ID)
		self.set_type_controller('OA')
		self.set_desired_OA_distance(distance)
######################################################################################## getter functions
	def get_lane(self):
		return self.vehicle.desired_lane

	def get_moil_fwd(self):
		if self.do_i_have_a_miol_fwd():
			return self.vehicle.closest_objects['fl']
		else:
			return False
	def bwd_STOM(self):
		id = self.vehicle.bwd_pair_partner
		return self.vehicle.bodies_dict[id].supervisory_module.STOM
######################################################################################## Scenario 1 functions

## return true if the perception module is available and that the forward left object is visible
	def do_i_have_a_miol_fwd(self):
		return self.vehicle.closest_objects!=[] and ('fl' in self.vehicle.closest_objects)

	def is_miol_fwd_not_paired(self):
		id = self.get_moil_fwd()
		if id!=False:
			if self.vehicle.bodies_dict[id].bwd_pair_partner == False:
				return True
		return False

	def pairupB(self):
		id = self.get_moil_fwd()
		self.set_fwd_pair_partner(id)
		self.setOATarget(id,10.0)

	def is_gap_made(self):
		d_fwd_p = self.vehicle.perception_module.getLongitudinalDistanceToTarget(self.vehicle.fwd_pair_partner)
		d_fwd = self.vehicle.perception_module.getLongitudinalDistanceToTarget(self.vehicle.closest_objects['fm'])
		if d_fwd_p>5.0 and d_fwd > 20.0:
			return True
		return False

	def pairupA(self):
		bwd_pair_id = self.vehicle.bwd_pair_partner
		fwd_pair_id = self.vehicle.bodies_dict[bwd_pair_id].closest_objects['fm']
		self.set_fwd_pair_partner(fwd_pair_id)
		self.setOATarget(fwd_pair_id,10.0)


	def receive_pair_from_B(self):
		self.vehicle.bwd_pair_partner = False
		for vehicle in self.vehicle.bodies_dict.values():
			if self.vehicle.id == vehicle.fwd_pair_partner:
				self.vehicle.bwd_pair_partner = vehicle.id

	def merge(self):
		self.set_desired_lane(Lane.A)

	def reset_pairing(self):
		self.resetOA()
		self.set_fwd_pair_partner(None)
		self.set_bwd_pair_partner(None)

	def reset_messages(self):
		self.set_STOM(False)

	def is_fwd_pair_merged(self):
		return self.vehicle.bodies_dict[self.vehicle.fwd_pair_partner].supervisory_module.am_i_merged()

	def am_i_merged(self):
		if self.vehicle.control_module.getTraj() == Lane.RIGHT:
			return True
		return False
########################################################################################internal functions
	def get_velocity(self):
		''' Returns the current velocity '''
		velocity = math.hypot(self.vehicle.x_speed, self.vehicle.y_speed)
		return velocity

	def set_desired_lane(self, desired_lane): #desired_lane is a string ("right_lane" osv)
		self.vehicle.desired_lane=desired_lane

	def send_message(self,freq): #freq=1,25 or 10
		'''send message '''
		if freq==10:
			message_list=self.message_module.generateMessage10()
		else:
			message_list=self.message_module.generateMessage(freq)
		self.vehicle.v2v_network_input.extend(message_list)

	def message_frequency(self,freq):
		counter=1
		while True:
			start_time=time.time()
			if freq==10:
				self.send_message(10)
				time_end=time.time()
				elapsed_time=time_end-start_time
				sleep_time=0.1-elapsed_time
				if sleep_time>0:
					time.sleep(sleep_time)
					self.communication_error=False
				else:
					self.communication_error=True
			else:
				if counter==25:
					self.send_message(1)
					counter=1
				else:
					self.send_message(25)
					counter+=1
				time_end=time.time()
				elapsed_time=time_end-start_time
				sleep_time=0.04-elapsed_time
				if sleep_time>0:
					time.sleep(sleep_time)
					self.communication_error=False
				else:
					self.communication_error=True

	def message_handler(self):
		while True:
			start_time=time.time()
			self.message_incoming=self.vehicle.get_network_output_messages()
			#Handle the message: filter for importance
			end_time=time.time()
			elapsed_time=end_time-start_time
			sleep_time=0.04-elapsed_time
			if sleep_time>0:
				time.sleep(sleep_time)
				self.communication_error=False
			else:
				self.communication_error=True

	def setACCCTarget(self, ID, desired_distance): ## ID is the id of the vehicle you want to follow, desired_distance is the distance the controller will try to keep
		''' Sets ACCC target and necessary variables, clears ACCC_ready and returns True if it succeeded. Decomment lateral_following if you want, though it doesn't work well '''

		if self.vehicle.control_module:
			print str(self.vehicle.id)+" setting accc on "+str(ID)
			control_module = self.vehicle.control_module
			control_module.ACCC_target_id = ID
			control_module.ACCC_desired_distance = desired_distance
			control_module.ACCC = True

			# control_module.lateral_following = True
			self.vehicle.ACCC_ready.clear()
			return True
		else:
			print "No control module"
			return False

	def setOATarget(self, ID, desired_distance): ## ID is the id of the vehicle you want to follow, desired_distance is the distance the controller will try to keep
		''' Sets ACCC target and necessary variables, clears ACCC_ready and returns True if it succeeded. Decomment lateral_following if you want, though it doesn't work well '''

		if self.vehicle.control_module:
			print str(self.vehicle.id)+" setting OA on "+str(ID)
			control_module = self.vehicle.control_module
			control_module.OA_target_id = ID
			control_module.OA_desired_distance = desired_distance
			control_module.OA = True

			# control_module.lateral_following = True
			self.vehicle.OA_ready.clear()
			return True
		else:
			print "No control module"
			return False

	def resetACCC(self):
		''' Turns of ACCC and resets all the necessary variables '''

		if self.vehicle.control_module:
			self.vehicle.control_module.ACCC = False
			self.vehicle.control_module.ACCC_target_velocity = None
			self.vehicle.control_module.lateral_following = False
			self.vehicle.control_module.ACCC_desired_distance = None
			self.vehicle.control_module.ACCC_distance = None
			self.vehicle.control_module.ACCC_target_id = None
			self.vehicle.ACCC_ready.clear()

	def resetOA(self):
		''' Turns of ACCC and resets all the necessary variables '''

		if self.vehicle.control_module:
			self.vehicle.control_module.OA = False
			self.vehicle.control_module.OA_desired_distance = None
			self.vehicle.control_module.OA_distance = None
			self.vehicle.control_module.OA_target_id = None
			self.vehicle.OA_ready.clear()


	def waitForEvent(self, event, function_to_run, abort_parameter, function_params = (), time_out = None):
		"""
		This function starts a thread that waits on event and then runs function_to_run with arguments function_params.

		:event: Thread waits for this event
		:function_to_run: Function to run when event is triggered
		:abort_parameter: Thread checks this to be False before running function_to_run
		:function_params: tuple or list of the parameters passed to function_to_run
		"""

		temp_thread = threading.Thread(target= self.waitForEventThread, args = (function_to_run, event, abort_parameter, function_params, time_out))
		temp_thread.daemon = True
		temp_thread.start()

	def waitForEventThread(self, function_to_run, event, abort_parameter, function_params, time_out):

		event.wait(time_out)

		if not abort_parameter:# and event.isSet():
			function_to_run(*function_params)

