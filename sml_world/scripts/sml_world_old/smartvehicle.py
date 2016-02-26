import socket, sys, time, math, random, threading

import vehiclesupervisorymodule_old
import vehiclecontrolmodule
import vehicleperceptionmodule
import vehiclesupervisorymodule
import bodyclasses

import numpy


# TODO
# add more to handle message function

class SmartVehicle(bodyclasses.WheeledVehicle):
	"This class is supposed to encompass three other classes which are running in threads"
	"The three other classes are the SupervisoryModule, the PerceptionModule and the "
	"ControlModule"
	"This class manages all the three Modules at the same time, making sure they are able to"
	"communicate with each other"
	

	length = 4.12 # Very carefully calculated through complex scientific observations and experiments
	width = 1.7 # Pure guesswork

	# Very caluculation


	def __init__(self, sml_world, vehicle_id, lane_start_point_index = 0, real_vehicle = False, emergency_vehicle = False, intersection_vehicle = False, init_velocity=50./3.6/2.0, init_lane="right_lane"):

		# For implementation with Unity's simulator
		# If set to True, the control module will 
		# not be able to set the steering angle,
		# instead this steering angle will be provided
		# by the Driver in the Loop user.
		self.unity_steering_override = False

		if not isinstance(vehicle_id, int):
			raise NameError('In class SmartVehicle: constructor: id must be an integer')

		bodyclasses.WheeledVehicle.__init__(self)

		self.sml_world = sml_world

		self.intersection_vehicle = intersection_vehicle

		# Gives vehicles access to all other vehicles for convenience
		self.bodies_dict = sml_world.bodies_dict

		self.road_module = sml_world.road_module
		
		# Lengths used somewhere instead of SmartVehicle.length and widht.
		self.length = 4.12
		self.width = 1.7

		self.air_drag_coeff = 50.
		self.vehicle_mass = 1000.

		# If this is true the vehicle is externally controlled via MyRIO
		if real_vehicle:

			self.simulated = False

		self.real_vehicle = real_vehicle


		# Defines if the car is equiped with a radar sensor
		# If True the car will be (additionaly) equipped
		# with a radar sensor 
		self.radar_sensor = False
		self.radar_angle = math.radians(60)
		self.radar_range = 100.

		# Defines if the car is equiped with a velodyne sensor
		# If True the car will be (additionaly) equipped
		# with a velodyne sensor 
		self.velodyne_sensor = True
		self.velodyne_range = 50.

		# Defines if the car is equiped with a perfect sensor
		# that has readings from every car in the SML World.
		self.omniscient_sensor = False


		# It tells the control module if we want to cycle through the trajectory
		# Used when we have closed trajectories
		self.cycle_traj = True

		# Smart Vehicle can use the wifi/network, to achieve communications and agreement with other vehicles

		# Check this to True if you want the platoon merge scenario, and change number_platooning_vehicles aswell as creation of broken vehicles in world_class
		self.platoon_merge_scenario = False
		self.overtaking_toggled = False


		# All input and output to and from the wifi and network broadcast system. As implemented now v2v_wifi_output and v2v_network_output is the outgoing messages from this vehicle.
		# They are also lists of at most one element (v2v_wifi_output and v2v_network_output).
		self.v2v_wifi_input = []
		self.v2v_wifi_output = []
		self.v2v_network_input = []
		self.v2v_network_output = []

		
		# COmmands fetched by the commands_module
		self.commands = dict()
		self.commands['throttle'] = 0.0
		self.commands['steering'] = 0.0
		self.current_steering_angle = 0.0

		# ID of this vehicle
		self.id = vehicle_id

		# Variables to store the lane trajectories

		# Definition of string constants
		self.RIGHT_LANE_STRING = "right_lane"
		self.CENTER_LANE_STRING = "center_lane"
		self.LEFT_LANE_STRING = "left_lane"
		
		# Initialization of the lane trajectorys
		self.traj = []
		self.right_lane_traj = []
		self.center_lane_traj = []
		self.left_lane_traj = []

		self.traj_points_per_meter = 5.
		# This fills self.<something>_lane_traj variables with information
		self.getLaneTrajectorys(self.road_module)

		# Old lane trajectory dict
		self.lane_traj_dict = {self.RIGHT_LANE_STRING: self.right_lane_traj, self.CENTER_LANE_STRING: self.center_lane_traj, self.LEFT_LANE_STRING: self.left_lane_traj}

		# numpy is used to find closest point
		self.np_lane_traj_dict = {self.RIGHT_LANE_STRING: self.np_right_lane_traj, self.CENTER_LANE_STRING: self.np_center_lane_traj, self.LEFT_LANE_STRING: self.np_left_lane_traj}


		# Later filled with readings from the wifi sensors
		self.current_body_readings = None

		
		
		# current variables #####################################

		# These are filled by perception module
		self.closest_objects = {}
		self.perception_grid = {}
		self.perceived_objects = {}
		
		## Choosing start lane, depends on whether you are in the platoon_merge_scenario or not
		self.desired_lane = init_lane
		if self.id%2 == 0 and self.platoon_merge_scenario:
			self.desired_lane = self.CENTER_LANE_STRING

		## Setting the starting velocity (which is in m/s)
		self.desired_velocity = 50./3.6/2.


		self.traj = self.getLaneTrajectory(self.desired_lane)


		#########################################################


		# safety variables
		self.saved_obstacle_messages = {} # key will be obstacle id, value is the message
		self.emergency_velocity_flag = False # True if you have an emergency velocity set
		self.temporary_velocity_flag = None # will point to the id of the vehicle who sent the temp vel
		self.old_velocity = None # saves velocity you had before setting temporary/emergency velocity

		# Overtaking variables
		self.overtaking_vehicle_flag = False # True if you are currently overtaking one or more vehicles
		self.overtaking_original_lanes = [] # needs to be list for recursive overtaking
		self.overtaken_vehicles = [] # needs to be list for recursive overtaking
		self.overtaking_original_velocities = [] # will be list
		self.new_desired_lanes = [] # for when a new lane is desired, but not currently available
		self.fwd_partner_lane = None # the lane fwd partner is in, set in handleFwdPartnerInfo in supervisory module
		self.new_fwd_partner_lane = None # the lane new fwd partner is in, set in handleFwdPartnerInfo in supervisory module


		self.saved_smartvehicle_ids = set() # will contain ids of all smartvehicles, updated through network broadcasts
		self.saved_wifi_messages = {} # saves most recent wifi message. key is vehicle id
		self.saved_network_messages = {} # saves most recent network message. key is vehicle id

		# Emergency vehicle variables
		self.emergency_vehicle = emergency_vehicle
		self.saved_emergency_vehicle_messages = {}
		self.closeby_emergency_vehicles = set()
		self.emergency_vehicle_offset_pointer = None #will be vehicle id
		self.new_offset = 0.
		self.lane_offset = 0.

		# Merging flags etc
		self.merging = False # True if you are merging (at the moment meaning you have a new fwd pair partner)
		self.platooned_vehicle = False # True if vehicle belongs to a platoon
		self.counted_platoon_vehicles = [] # contains ID of every vehicle in platoon, checked by plat leader
		self.emergency_merge_initiation = False # True if you are initiating a merge because an obstacle is in the way, used for GCDC scenario 1


		# some platooning variabels
		self.catching_up = False # True if you are catching up with fwd/new fwd pair partner because you are too far away
		self.catchup_factor = 1.3 # multiplier for the velocity used to catch up with fwd/new fwd pair partner
		self.merge_distance = 12. # The distance the ACCC will try to keep for a merged vehicle to its target
		self.send_desired_merge_distance = 120. # a vehicle will not send a desired merge through broadcast search to vehicles outside of this distance
		self.new_fwd_pair_partner = None # Future fwd partner
		self.new_bwd_pair_partner = None # Future bwd partner
		self.fwd_pair_partner = None # fwd partner
		self.bwd_pair_partner = None # bwd partner
		self.platoon_leader = False# is set in supervisory module if you have no fwd/new fwd partner but you have a bwd or new bwd partner
		self.tail_vehicle = False #last vehicle in platoon

		self.forward_partner_info = {} # is the most recent message pinged from your fwd partner, resets when you call setPairPartner with fwd = True
		self.new_forward_partner_info = {} # is the most recent message pinged from your new fwd partner, resets when you call setPairPartner with fwd = True and new = True
		self.platoon_lane = None # the lane the platoon is trying to keep, dictated by the leader and sent to everyone in the platoon through the fwd partner ping

		#Error variables, added by Sofie Andersson 11-2015
		self.error_perception=False
		self.error_control=False
		self.error_supervisory=False


		# ACCC parameters, rest of them are in control module
		self.ACCC_ready = threading.Event() # Event fired when ACCC is ready, that is the target vehicle is at the right distance
		self.ACCC_ready_margin = 3.0 # Parameter for firing the ACCC_ready event

		# ACCC parameters, rest of them are in control module
		self.OA_ready = threading.Event() # Event fired when ACCC is ready, that is the target vehicle is at the right distance
		self.OA_ready_margin = 3.0 # Parameter for firing the ACCC_ready event

		# other events, internal flag initially false, unless expicitly set() (hahaha)
		self.lanekeeping_ready = threading.Event() # is set if you are withing the lanekeeping_margin of your desired trajectory (y-direction)
		self.merge_distance_reached = threading.Event() # is set when the merge distance is reached for the emergency merge (GCDC scenario)
		self.platoon_length_count_completed = threading.Event() # is set when platoon length count is completed for the emergency merge (GCDC scenario)

		# lanekeeping margin
		if self.real_vehicle:
			self.lanekeeping_margin = 1. # margin used to set lanekeeping_ready
		else:
			self.lanekeeping_margin = 0.3

		# event waiter aborters
		self.fix_lane_change_abort = False
		self.abort_merge_start = False
		self.abort_fwd_pp_search = False

		# Creating the Supervisory Module, setting its rate and starting its thread
		if self.emergency_vehicle:
			self.supervisory_module = vehiclesupervisorymodule_old.EmergencyVehicleSupervisoryModule(self);
		else:
			self.supervisory_module = vehiclesupervisorymodule.VehicleSupervisoryModule( self )
		self.supervisory_module_rate = 30.
		self.start_thread( self.supervisory_module_thread, args=([]) )

		if self.real_vehicle:

			# Creating the Control Module, setting its rate and starting its thread
			self.control_module = vehiclecontrolmodule.VehicleControlModule(self, self.id, intersection_vehicle = self.intersection_vehicle)
			self.control_module_rate = 20.
			time.sleep(0.02) # This sleep is just to try to make the prints more separate
			# self.set_vehicle_on_trajectory_state_based_on_id(lane_start_point_index)
			self.start_thread( self.control_module_thread, args=([]) )

		else:

			# Creating the Control Module, setting its rate and starting its thread
			self.control_module = vehiclecontrolmodule.VehicleControlModule(self, self.id, intersection_vehicle = self.intersection_vehicle)
			self.control_module_rate = 20.
			time.sleep(0.02) # This sleep is just to try to make the prints more separate
			# self.set_vehicle_on_trajectory_state_based_on_id(lane_start_point_index)
			self.start_thread( self.control_module_thread, args=([]) )

		# Creating the Perception Module, setting its rate and starting its thread:

		self.perception_module = vehicleperceptionmodule.VehiclePerceptionModule(self, intersection_vehicle = self.intersection_vehicle)
		self.perception_module_rate = 20.
		time.sleep(0.02) # This sleep is just to try to make the prints more separate
		self.start_thread( self.perception_module_thread, args=([]) )


		self.assignPerceptionModule()

		self.setInitialVelocity(init_velocity)

	def setInitialVelocity(self,velocity):
		self.x_speed = velocity

	def assignPerceptionModule(self):
		self.supervisory_module.perception_module = self.perception_module
		self.control_module.perception_module = self.perception_module

	def getLaneTrajectory(self, lane_traj_string):
		if self.lane_traj_dict:
			return self.lane_traj_dict[lane_traj_string]
		else:
			print "ERROR: no lane traj dict created"
			return None

	def getNumpyLaneTrajectory(self, lane_traj_string):
		if self.np_lane_traj_dict:
			return self.np_lane_traj_dict[lane_traj_string]
		else:
			print "ERROR: no lane traj dict created"
			return None

	def getCurrentLaneTrajectory(self):
		if self.lane_traj_dict:
			return self.lane_traj_dict[self.desired_lane]

	def getCurrentLaneTrajectoryString(self):
		if self.desired_lane:
			return self.desired_lane
		else:
			return None

	def getLaneTrajectorys(self, road_module):
		"""
		sets the lane trajectorys of the vehicle

		:osm_info: some kind of info
		:returns: boolean True if it was successful, has never happened that it failed without crashing though

		"""
		start_id = -1
		node_start = -1

		# right_lane_node_id = -1543
		# center_lane_node_id = -1424
		# left_lane_node_id = -1297

		# node_id_list = [ osm_info.osm_node_list[idx].id for idx in xrange( len( osm_info.osm_node_list ) ) ]

		# right_lane_node_start_idx = node_id_list.index(right_lane_node_id)		

		right_lane_node_ids_list = road_module.osm_node_tag_dict['right_lane_node']
		right_lane_node_id = right_lane_node_ids_list[0]

		center_lane_node_ids_list = road_module.osm_node_tag_dict['center_lane_node']
		center_lane_node_id = center_lane_node_ids_list[0]

		left_lane_node_ids_list = road_module.osm_node_tag_dict['left_lane_node']
		left_lane_node_id = left_lane_node_ids_list[0]


		[right_lane_traj_x, right_lane_traj_y] = road_module.get_closed_path_from_node_id(right_lane_node_id, self.traj_points_per_meter)
		[center_lane_traj_x, center_lane_traj_y] = road_module.get_closed_path_from_node_id(center_lane_node_id, self.traj_points_per_meter)
		[left_lane_traj_x, left_lane_traj_y] = road_module.get_closed_path_from_node_id(left_lane_node_id, self.traj_points_per_meter)

		if len(right_lane_traj_x) == 0 or len(center_lane_traj_x) == 0 or len(left_lane_traj_x) == 0 :
			#Error, could not find a suitable trajectory
			# Can happen if a vehicle is in a node for trucks only
			print "PlatooningManager Error: could not find a suitable trajectory"
			return False

		desired_velocity = 10

		# vehicle.set_state_trajectory(traj_x, traj_y, desired_velocity)
		self.set_right_lane_trajectory( right_lane_traj_x, right_lane_traj_y, desired_velocity )
		self.set_center_lane_trajectory( center_lane_traj_x, center_lane_traj_y, desired_velocity )
		self.set_left_lane_trajectory( left_lane_traj_x, left_lane_traj_y, desired_velocity )

		return True



	def set_vehicle_on_trajectory_state(self, idx = 0):
		
		if len( self.traj ) == 0:

			raise NameError('In class SmartVehicle: trying to use set_vehicle_on_trajectory_state when trajectory is not yet defined')

		idx = idx%len(self.traj[0])

		self.x = self.traj[0][idx]
		self.y = self.traj[1][idx]
		self.yaw = math.degrees(self.traj[2][idx])

		return

	def set_vehicle_on_trajectory_state_based_on_id(self):

		# This function is used to place the vehicle in the initial position
		# of the trajectory

		if self.id > 0:

			id_to_use = self.id

		else:

			id_to_use = -self.id

		## SPAWNS FOR OVERTAKING
		if not self.platoon_merge_scenario:

			idx = int(-self.id*100)
			idx += -2500

		## SPAWNS FOR PLATOON MERGE SCENARIO
		if self.platoon_merge_scenario:
			idx = int(-self.id*40)
			if self.id%2 != 0:
				idx += 90
			idx += -500


		idx = idx % len(self.traj[0])
		self.control_module.current_trajectory_id = idx

		self.set_vehicle_on_trajectory_state(idx)

		if self.emergency_vehicle:
			self.x = 33.
			self.y = -37.5
			self.yaw = 0.
		self.z = 0.0

		self.roll = 0.0

		return


	def set_right_lane_trajectory(self, traj_x, traj_y, desired_velocity):
		# Function used to define the right lane trajectory
		self.set_lane_trajectory( traj_x, traj_y, desired_velocity, self.RIGHT_LANE_STRING)

	def set_center_lane_trajectory(self, traj_x, traj_y, desired_velocity):
		# Function used to define the center lane trajectory
		self.set_lane_trajectory( traj_x, traj_y, desired_velocity, self.CENTER_LANE_STRING)

	def set_left_lane_trajectory(self, traj_x, traj_y, desired_velocity):
		# Function used to define the left lane trajectory
		self.set_lane_trajectory( traj_x, traj_y, desired_velocity, self.LEFT_LANE_STRING)

	def set_lane_trajectory(self, traj_x, traj_y, desired_velocity, lane_position):
		# Function used to set a lane trajectory, used for getting all the lane trajectorys
		
		traj_theta = []
		temp_theta = 0
		traj_time = []
		current_time = 0

		for idx in range( len ( traj_x ) - 1 ):

			delta_x = traj_x[idx+1] - traj_x[idx]
			delta_y = traj_y[idx+1] - traj_y[idx]

			temp_theta = math.atan2(delta_y, delta_x)
			traj_theta.append(temp_theta)

			traj_time.append(current_time)

			distance_moved = math.hypot(delta_x, delta_y) # Euclidean norm
			time_passed = distance_moved/desired_velocity

			current_time = current_time + time_passed

		traj_theta.append(temp_theta)
		traj_time.append(current_time)

		if len(traj_y) != len(traj_x) or len(traj_time) != len(traj_x) or len(traj_theta) != len(traj_x):

			raise NameError("SmartVehicle Trajectory creation resulted in a mistake!")

		if lane_position == self.RIGHT_LANE_STRING:

			self.right_lane_traj = [traj_x, traj_y, traj_theta, traj_time]
			self.np_right_lane_traj = numpy.asarray([ traj_x , traj_y , traj_theta])

		else:
		
			if lane_position == self.CENTER_LANE_STRING:

				self.center_lane_traj = [traj_x, traj_y, traj_theta, traj_time]
				self.np_center_lane_traj = numpy.asarray([ traj_x , traj_y , traj_theta])

			else:

				if lane_position == self.LEFT_LANE_STRING:

					self.left_lane_traj = [traj_x, traj_y, traj_theta, traj_time]
					self.np_left_lane_traj = numpy.asarray([ traj_x , traj_y , traj_theta])

				else:

					print "ERROR SmartVehicle: No suitable lane found!"

		return

	def get_current_state(self, radians = True):
		#Function to simply return the current state
		# returns [x, y, yaw]

		state_vector = []

		state_vector.append( self.x )
		state_vector.append( self.y )

		if radians:

			state_vector.append( self.yaw )

		else:

			state_vector.append( math.degrees(self.yaw) )

		return state_vector

	def supervisory_module_thread(self):

		# This is the function that will be running continuously on a thread
		# This function is supposed to manage/interface with the Supervisory Layer

		while True:

			start_time = time.time()

			# Run a step of the Supervisory Module
			self.supervisory_module.step()

			
			# Make the thread sleep and run with adequate cycle time, set error_supervisory as true if it is to slow (added by Sofie)
			self.error_supervisory=self.make_thread_sleep(start_time, self.supervisory_module_rate,  "In SmartVehicle: supervisory_module_step thread is taking too long!", verbose = True)


	def control_module_thread(self):

		# This is the function that will be running continuously on a thread
		# This function is supposed to manage/interface with the Control Layer

		while True:

			start_time = time.time()

			# Tell the control module the desired lane

			self.control_module.step()

			# Make the thread sleep and run with adequate cycle time, set error_supervisory as true if it is to slow (added by Sofie)
			self.error_control=self.make_thread_sleep(start_time, self.control_module_rate,  "In SmartVehicle: control_module_step thread is taking too long!", verbose = False)

	def perception_module_thread(self):

		# This is the function that will be running continuously on a thread
		# This function is supposed to manage/interface with the Perception Layer

		while True:

			start_time = time.time()

			self.perception_module.step()

			# Make the thread sleep and run with adequate cycle time, set error_supervisory as true if it is to slow (added by Sofie)
			self.error_perception=self.make_thread_sleep(start_time, self.perception_module_rate,  "In SmartVehicle: perception_module_step thread is taking too long!", verbose = True)


	def make_thread_sleep(self, start_time, desired_rate,  warning_string, verbose = True):

		# A simple function that will make the thread sleep (or the cycle wait)
		# in order to respect the desired rate.
		# It will also issue a warning in case the desired rate cannot be performed
		# (due to taking too much time)
		end_time = time.time()
		elapsed_time = end_time - start_time
		desired_cycle_time = 1./desired_rate
		sleep_time = desired_cycle_time - elapsed_time
		if sleep_time <= 0:
			if verbose:
				print warning_string
				return True #added by sofie as error message
		else:
			time.sleep(sleep_time)
			return False #added by sofie as error message

	def start_thread(self, handler, args=()):
		# Function that simplifies thread creation
		t = threading.Thread(target=handler, args=args)
		t.daemon = True
		# self.threads.append(t)
		t.start()

	def getPerceivedObject(self, identity):
		# Getter for perceived object

		if identity in self.perceived_objects:
			return self.perceived_objects[identity]
		else:
			return None

	def getState(self, params):
		# This is used for the state ping. This protocol is not used atm but should work fine. The function that handles the ping is in supervisory module
		state = []
		for param in params:
			if param in self.stateDict:
				tup = []
				tup.append(param)
				tup.append(self.stateDict[param]())
				state.append(tup)

		return state

	def getError(self):
		##get errors from perception, control and supervisory due to slow steps. Added by Sofie, unsure if it works :P
		errors=[self.error_perception, self.error_control, self.error_supervisory]
		return errors
			






	'''
	V2V Interface Methods
	Author: Rui
	These methods allow the V2VModule to interact
	with the SmartVehicle in a cleaner / more
	Object Oriented way
	'''
	def set_network_input_messages(self, messages_list):

		# Prints for Debugging	
		# print_str =  "Vehicle " + str(self.id) + " received the following network messages"
		# for message in messages_list:
		# 	print_str += "\n" + str(message)  
		# print print_str

		self.v2v_network_input.extend( messages_list )

		return

	def get_network_output_messages(self):

		# If no messages to output, return none
		if not self.v2v_network_output:

			return None

		# Prints for Debugging	
		# print_str =  "Vehicle " + str(self.id) + " will output the following network messages"
		# for message in self.v2v_network_output:
		# 	print_str += "\n" + str(message)  
		# print print_str

		messages_to_output = self.v2v_network_output[:]

		del self.v2v_network_output[:]

		return messages_to_output

	def set_wifi_input_messages(self, messages_list):

		# Prints for Debugging	
		# print_str =  "Vehicle " + str(self.id) + " received the following Wi-Fi messages"
		# for message in messages_list:
		# 	print_str += "\n" + str(message)  
		# print print_str

		self.v2v_wifi_input.extend( messages_list )

		return


	def get_wifi_output_messages(self):

		# If no messages to output, return none
		if not self.v2v_wifi_output:

			return None

		# Prints for Debugging	
		# print_str =  "Vehicle " + str(self.id) + " will output the following Wi-Fi messages"
		# for message in self.v2v_wifi_output:
		# 	print_str += "\n" + str(message)  
		# print print_str

		messages_to_output = self.v2v_wifi_output[:]

		del self.v2v_wifi_output[:]

		return messages_to_output

	'''
	Vehicle Sensor Readings Interface
	Author: Rui
	This method allow the SimulatorModule to interact
	with the SmartVehicle in a cleaner / more
	Object Oriented way
	'''

	def set_sensor_readings(self, sensor_readings_list):
		'''
		Given a list of sensor readings, I will store them
		in my attribute sensor_readings
		'''

		self.sensor_readings = sensor_readings_list

		return


	'''
	Vehicle State Update Interface
	Author: Rui
	This method allows the SimulatorModule to interact
	with the SmartVehicle in a cleaner / more
	Object Oriented way
	'''
	def vehicle_state_update(self, time_to_simulate):
		'''
		Defines the state update of the vehicle, given 
		the time_to_simulate in seconds.

		This vehicle state update assumes the vehicle 
		model is an acceleration based unicycle model.

		This method is called by the SimulatorModule, for 
		at every simulation loop.
		'''

		# Compute my old linear speed and my current acceleration
		# in order to know my new linear speed
		linear_speed = math.hypot(self.x_speed, self.y_speed)

		acceleration = ( self.commands['throttle'] - self.air_drag_coeff*linear_speed ) / self.vehicle_mass
	
		linear_speed += acceleration*time_to_simulate

		# Here I am assuming that negative throttles correspond to braking, and as such cannot make the vehicle go backwards
		if self.commands['throttle'] < 0. and linear_speed < 0.:

			linear_speed = 0.

		# Knowing the current linear speed, I can know the speed components in x and y
		self.x_speed = math.cos( math.radians( self.yaw ) )*linear_speed
		self.y_speed = math.sin( math.radians( self.yaw ) )*linear_speed
		
		# Update my position
		self.x = self.x + self.x_speed*time_to_simulate
		self.y = self.y + self.y_speed*time_to_simulate

		'''
		Extension to include steering wheel dynamics
		That is, there is a delay between the desired
		steering angle and the current steering angle
		due to a maximum steering angle change rate

		Not added because, controller is not yet able 
		to deal with it

		desired_steering_rate = self.commands['steering'] - self.current_steering_angle

		max_steering_rate = math.radians(90)
		max_steering_change = max_steering_rate*time_to_simulate

		if desired_steering_rate > max_steering_change:

			desired_steering_rate = max_steering_change

		elif desired_steering_rate < -max_steering_change:

			desired_steering_rate = -max_steering_change

		self.current_steering_angle += desired_steering_rate

		self.yaw = self.yaw + math.degrees( (linear_speed/self.axles_distance)*math.tan(self.current_steering_angle)*time_to_simulate )

		'''

		# Update my yaw, assuming a simple kinematic car model
		self.yaw = self.yaw + math.degrees( (linear_speed/self.axles_distance)*math.tan(self.commands['steering'])*time_to_simulate )
		

		# Limit the yaw to the range between 0 and 360
		while self.yaw > 360:

			self.yaw -= 360

		while self.yaw < 0:

			self.yaw += 360
