import socket, sys, time, math, random, threading, datetime


import numpy


class VehicleControlModule:
	"This class is the Platooning Vehicle class used by the Platooning Manager Module"
	def __init__(self, vehicle, vehicle_id, intersection_vehicle = False):

		self.vehicle = vehicle

		self.intersection_vehicle = intersection_vehicle

		self.bodies_dict = vehicle.bodies_dict
		self.vehicle_id = vehicle_id


		self.current_body_readings = []
		self.desired_velocity = 0.
		self.state = []

		# Variables to store the lane trajectories
		self.traj_points_per_meter = self.vehicle.traj_points_per_meter

		self.current_lane = self.vehicle.desired_lane

		self.traj = self.vehicle.getLaneTrajectory(self.vehicle.desired_lane)
		traj_x = self.traj[0]
		traj_y = self.traj[1]
		traj_theta = self.traj[2]
		self.np_traj = numpy.asarray([ traj_x , traj_y , traj_theta])

		self.right_lane_traj = self.vehicle.getLaneTrajectory("right_lane")
		traj_x = self.right_lane_traj[0]
		traj_y = self.right_lane_traj[1]
		traj_theta = self.right_lane_traj[2]
		self.np_right_lane_traj = numpy.asarray([ traj_x , traj_y , traj_theta])

		self.center_lane_traj = self.vehicle.getLaneTrajectory("center_lane")
		traj_x = self.center_lane_traj[0]
		traj_y = self.center_lane_traj[1]
		traj_theta = self.center_lane_traj[2]
		self.np_center_lane_traj = numpy.asarray([ traj_x , traj_y , traj_theta])

		self.left_lane_traj = self.vehicle.getLaneTrajectory("left_lane")
		traj_x = self.left_lane_traj[0]
		traj_y = self.left_lane_traj[1]
		traj_theta = self.left_lane_traj[2]
		self.np_left_lane_traj = numpy.asarray([ traj_x , traj_y , traj_theta])

		# Definition of string constants
		self.RIGHT_LANE_STRING = "right_lane"
		self.CENTER_LANE_STRING = "center_lane"
		self.LEFT_LANE_STRING = "left_lane"

		self.current_trajectory_id = 0

		# Variables to store the low level commands that will be
		# read by the SmartVehicle module and sent to the Simulator
		# self.current_velocity_command = 0.
		# self.current_steering_command = 0.

		self.anchor_idx = None
		self.distance_to_anchor = None

		self.VACCC = False
		self.ACCC = False
		self.CC = True
		self.OA = False
		self.lateral_following = False

		# variables for the accc to work
		self.ACCC_target_id = None
		self.ACCC_desired_distance = None
		self.ACCC_distance = None
		self.ACCC_target_velocity = None

		# variables for the OA to work
		self.OA_target_id = None
		self.OA_desired_distance = None
		self.OA_distance = None
		self.OA_small_distance = 5.0
		self.OA_deccelerate_throttle = -100.0
		self.OA_action = True

		# variables for the VACCC to work
		self.VACCC_desired_distance = None
		self.VACCC_distance = None
		self.VACCC_target_velocity = None

		# variables for lateral_following
		self.lateral_following_displacement = 0

		# The last time the ACCC controller was executed
		# Useful for computations of the Integral and Derivative part of the PID controller
		self.last_ACCC_time = []
		# The integrated error of the ACCC used for the Integral part of the PID controller
		self.ACCC_I_e = []
		# The last error of the ACCC, used for the Derivative part of the PID controller
		self.last_ACCC_e = []
	
		# ACCC distance PID gain
		self.ACCC_k_p = 300.
		self.ACCC_k_i = 0.
		self.ACCC_k_d = 0.


		# The last time the CC controller was executed
		# Useful for computations of the Integral and Derivative part of the PID controller
		self.last_CC_time = []
		# The integrated error of the CC used for the Integral part of the PID controller
		self.CC_I_e = []
		# The last error of the CC, used for the Derivative part of the PID controller
		self.last_CC_e = []
	
		# CC PID gain
		self.CC_k_p = 500. # 250
		self.CC_k_i = 50. # 50
		self.CC_k_d = 150.
		self.reset_CC()

		# The last time the lateral following controller was executed
		# Useful for computations of the Integral and Derivative part of the PID controller
		self.last_latfoll_time = []
		# The integrated error of the latfoll used for the Integral part of the PID controller
		self.latfoll_I_e = []
		# The last error of the latfoll, used for the Derivative part of the PID controller
		self.last_latfoll_e = []
	
		# latfoll PID gain
		self.latfoll_k_p = 0.04
		self.latfoll_k_i = 0.02
		self.latfoll_k_d = 0.05

		#Tips
		# self.state = [real_world_meters, radians]
		# print "self.state = " + str(self.state)
		# self.traj[0] in real_world_meters
		# self.traj[2] in radians
		# print "self.traj[2] = " + str(self.traj[2])

		# brake 
		self.brake = False

		if not self.vehicle.simulated:

			# Will create the socket to communicate with the real truck LabVIEW
			self.start_tcp_port()

		print "VehicleControlModule started"

	def start_tcp_port(self):
		'''
		This function creates the socket to send information to the truck in LabVIEW
		'''

		# Create a TCP/IP socket
		self.labview_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		# Bind the socket to the port
		server_port = 16000 + self.vehicle_id
		server_address = ('', server_port)
		print 'Starting LabVIEW socket on ' + str(server_address[0]) + ' port ' + str(server_address[1])
		self.labview_socket.bind(server_address)

		# Listen for incoming connections
		self.labview_socket.listen(1)

		self.labview_socket.settimeout(0.05)

		self.labview_connected = False

		self.tcp_publish_rate = 20.
		self.tcp_last_publish_time = time.time()

	def step(self):
		'''
		This is the step function that is running at each cycle of the thread
		It updates the current perceived ego state, and computes the control inputs 
		'''

		# Update my state perception
		if self.vehicle_id in self.bodies_dict:

			self.state = [self.bodies_dict[self.vehicle_id].x, self.bodies_dict[self.vehicle_id].y, math.radians(self.bodies_dict[self.vehicle_id].yaw) ]

		# Updates my perception of the distance to the car to follow, in case I am in ACCC mode
		if self.ACCC:
			self.updateACCCTarget()

		if self.OA:
			self.updateOATarget()


		# Update my perception of the lateral displacement to the car to follow, in case I am in lateral following mode
		# Currently not being used!
		if self.lateral_following:
			self.update_lateral_following_target()

		# Will if a lane change was requested, and perform it in case it has
		self.update_lane()

		# Compute the control actions and apply them to the vehicle instance
		if self.vehicle_id in self.bodies_dict:

			[speed, steering] = self.compute_low_level_inputs()

			self.vehicle.commands['throttle'] = speed

			if not self.vehicle.unity_steering_override:

				self.vehicle.commands['steering'] = steering

		# In case we are using a real mini truck we will send information to LabVIEW
		if self.vehicle.simulated == False:

			self.tcp_step()


	def tcp_step(self):
		'''
		This function simply manages the connection to the LabVIEW truck controller
		'''

		# If LabVIEW has not connected yet, we need to try to accept it's connection
		if not self.labview_connected:

			try:

				self.labview_connection, client_address = self.labview_socket.accept()
				# LabVIEW just connected
				self.labview_connected = True

			except socket.timeout:

				#In case LabVIEW did not connect, simply try again in the next control loop
				pass
				
		else:
		# The connection is established

			try:

				# Send information to LabVIEW
				self.send_data_to_labview()

			except socket.error:

				# Will probably happen if the connection died
				print "Socket error in vehicle send_data_to_labview()"


	def send_data_to_labview(self):
		'''
		This function simply sends relevant information to the LabVIEW control loop that is 
		running on the real mini trucks.
		'''

		# Only send the information at the rate self.tcp_publish_rate
		# Do not send it more often than that
		if time.time() - self.tcp_last_publish_time > 1./self.tcp_publish_rate:

			self.tcp_last_publish_time = time.time()

		else:

			return

		# Get the relevant information string and send it to LabVIEW
		data = self.get_tcp_string()
		self.labview_connection.sendall(data)

	def get_tcp_string(self):
		'''
		Returns a string that contains all the relevant information that needs to be sent
		to the LabVIEW truck controller
		'''

		time_stamp_str = " " + str(time.time())
		terminator_str = '\r\n'
		separator_token = 'XcRt'

		data = "time=" + str(self.bodies_dict[self.vehicle_id].timestamp)

		id_string = "id=" + str(self.vehicle_id) + ","

		if self.ACCC:

			if self.ACCC_target_velocity:

				desired_velocity = self.ACCC_target_velocity/32.

			else:

				desired_velocity = self.vehicle.desired_velocity/32.

			if self.ACCC_distance != None:				

				longitudinal_string = "current_distance=" + str(self.ACCC_distance/32.) + ",desired_distance="  + str(self.ACCC_desired_distance/32.) + ",velocity=" + str(desired_velocity)

			else:

				longitudinal_string = "current_distance=" + str(self.ACCC_desired_distance/32.) + ",desired_distance="  + str(self.ACCC_desired_distance/32.) + ",velocity=" + str(desired_velocity)
		else:

			desired_velocity = self.vehicle.desired_velocity/32.

			dummy_number = 32.
			longitudinal_string = "current_distance=" + str(dummy_number/32.) + ",desired_distance="  + str(dummy_number/32.) + ",velocity=" + str(desired_velocity)

		
		if self.current_lane == self.LEFT_LANE_STRING:

			lane_string = "lane=left,"

		elif self.current_lane == self.CENTER_LANE_STRING:

			lane_string = "lane=center,"

		else:

			lane_string = "lane=right,"


		pose_string = ",x=" + str(self.bodies_dict[self.vehicle_id].x/32.) + ",y=" + str(self.bodies_dict[self.vehicle_id].y/32.) + ",yaw=" + str(self.bodies_dict[self.vehicle_id].yaw)

		lane_offset_string = ",lane_offset=" + str(self.vehicle.lane_offset/32.)

		data +=  separator_token + id_string + lane_string + longitudinal_string + pose_string + lane_offset_string + terminator_str

		return data

	def setLaneCommand(self,lane):
		self.vehicle.desired_lane = lane

	def updateACCCTarget(self):
		'''
		Updates ACCC related information. Mainly the perceived distance to the car to follow
		'''
		target_coords = self.vehicle.getPerceivedObject(self.ACCC_target_id)

		if target_coords:

			self.ACCC_distance = target_coords[0]

			if abs(self.ACCC_desired_distance - self.ACCC_distance) < self.vehicle.ACCC_ready_margin:

				self.vehicle.waiting_for_ACCC = False
				self.vehicle.ACCC_ready.set()

			else:

				self.vehicle.waiting_for_ACCC = True
				self.vehicle.ACCC_ready.clear()

	def updateOATarget(self):
		'''
		Updates OA related information. Mainly the perceived distance to the car to follow
		'''
		target_coords = self.vehicle.getPerceivedObject(self.OA_target_id)

		if target_coords:

			self.OA_distance = target_coords[0]

			if abs(self.OA_desired_distance - self.OA_distance) < 3.0:

				self.vehicle.waiting_for_OA = False
				self.vehicle.OA_ready.set()

			else:

				self.vehicle.waiting_for_OA = True
				self.vehicle.OA_ready.clear()

	def update_lateral_following_target(self):
		'''
		CURRENTLY NOT BEING USED
		Updates my lateral perception of the car being followed
		'''

		target_coords = self.vehicle.getPerceivedObject(self.ACCC_target_id)

		if target_coords:

			self.lateral_following_displacement = target_coords[1]
		
	def update_lane(self):
		'''
		Function that changes the current trajectory to be followed, if 
		a new desired_lane is requested
		'''

		print_str = "A new lane was requested, changing now to "

		if self.vehicle.desired_lane != self.current_lane:

			if self.vehicle.desired_lane == self.CENTER_LANE_STRING:

				print_str += "center lane"
				self.smooth_lane_change(self.center_lane_traj, self.np_center_lane_traj)
				self.current_lane = self.CENTER_LANE_STRING

			elif self.vehicle.desired_lane == self.LEFT_LANE_STRING:

				print_str += "left lane"
				self.smooth_lane_change(self.left_lane_traj, self.np_left_lane_traj)
				self.current_lane = self.LEFT_LANE_STRING

			elif self.vehicle.desired_lane == self.RIGHT_LANE_STRING:

				print_str += "right lane"
				self.smooth_lane_change(self.right_lane_traj, self.np_right_lane_traj)
				self.current_lane = self.RIGHT_LANE_STRING

			else:

				print_str = "ERROR: Lane change request to an invalid lane"

			print print_str
	
	def smooth_lane_change(self, desired_traj, np_desired_traj):
		'''
		This function creates a new trajectory for the car upon request
		The trajectory is composed of a smooth transition from the current
		lane into the desired lane, and after the transition the trajectory
		simply becomes the desired lane trajectory
		'''

		best_idx = self.find_closest_trajectory_point(self.state, self.np_traj)

		current_traj_start_idx = best_idx

		merging_indexes = 150

		current_traj_end_idx = current_traj_start_idx + merging_indexes
		current_traj_end_idx = current_traj_end_idx%len(self.traj[0])

		best_idx = self.find_closest_trajectory_point(self.state, np_desired_traj)

		current_lane_merge_range = []
		# print "---"
		if current_traj_start_idx < current_traj_end_idx:
			current_lane_merge_range = xrange(current_traj_start_idx, current_traj_end_idx)
		else:
			current_lane_merge_range.extend(xrange(current_traj_start_idx, len(self.traj[0])))
			current_lane_merge_range.extend(xrange(current_traj_end_idx))
			# print "current_lane_merge_range = " + str(current_lane_merge_range)
		

		desired_traj_start_idx = best_idx

		current_traj_end_state = [ self.traj[0][current_traj_end_idx], self.traj[1][current_traj_end_idx], self.traj[2][current_traj_end_idx] ]
		best_idx = self.find_closest_trajectory_point(self.state, np_desired_traj)

		
		desired_traj_end_idx = desired_traj_start_idx + len(current_lane_merge_range)
		desired_traj_end_idx = desired_traj_end_idx%len(desired_traj[0])
		
		desired_lane_merge_range = []

		if desired_traj_start_idx < desired_traj_end_idx:
			desired_lane_merge_range = xrange(desired_traj_start_idx, desired_traj_end_idx)
		else:
			desired_lane_merge_range.extend(xrange(desired_traj_start_idx, len(desired_traj[0])))
			desired_lane_merge_range.extend(xrange(desired_traj_end_idx))
			# print "desired_lane_merge_range = " + str(desired_lane_merge_range)

		# sanity check
		if len( current_lane_merge_range ) != len( desired_lane_merge_range ):

			print "ERROR IN VEHICLE CONTROL MODULE, WRONG MERGE LANE SIZES"

		merging_traj = [[],[],[]]

		for i in range( len( current_lane_merge_range ) ):

			current_ratio = float( i )/float( len(current_lane_merge_range) )
			current_x = (1-current_ratio)*self.traj[0][ current_lane_merge_range[i] ] + current_ratio*desired_traj[0][ desired_lane_merge_range[i] ]
			current_y = (1-current_ratio)*self.traj[1][ current_lane_merge_range[i] ] + current_ratio*desired_traj[1][ desired_lane_merge_range[i] ]
			current_theta = (1-current_ratio)*self.traj[2][ current_lane_merge_range[i] ] + current_ratio*desired_traj[2][ desired_lane_merge_range[i] ]

			merging_traj[0].append(current_x)
			merging_traj[1].append(current_y)
			merging_traj[2].append(current_theta)

		remaining_traj = [[],[],[]]

		remaining_traj_indexes = []
		if desired_traj_start_idx < desired_traj_end_idx:
			remaining_traj_indexes.extend( xrange(desired_traj_end_idx, len(desired_traj[0]) ) )
			remaining_traj_indexes.extend( xrange( desired_traj_start_idx ) )
			# desired_lane_merge_range = xrange(desired_traj_start_idx, desired_traj_end_idx)
		else:
			remaining_traj_indexes = xrange(desired_traj_end_idx, desired_traj_start_idx)


		for idx in remaining_traj_indexes:

			merging_traj[0].append( desired_traj[0][idx] )
			merging_traj[1].append( desired_traj[1][idx] )
			merging_traj[2].append( desired_traj[2][idx] )

		self.traj = merging_traj

		traj_x = merging_traj[0]
		traj_y = merging_traj[1]
		traj_theta = merging_traj[2]
		self.np_traj = numpy.asarray([ traj_x , traj_y , traj_theta])


		self.vehicle.supervisory_module.waitForEvent(self.vehicle.lanekeeping_ready, self.fixTrajAfterLaneChange, self.vehicle.fix_lane_change_abort)


	def fixTrajAfterLaneChange(self):

		self.traj = self.vehicle.getCurrentLaneTrajectory()
		if self.traj is self.left_lane_traj:

			# print "Again on the left traj!"
			self.np_traj = self.np_left_lane_traj

		elif self.traj is self.center_lane_traj:

			# print "Again on the center traj!"
			self.np_traj = self.np_center_lane_traj

		elif self.traj is self.right_lane_traj:

			# print "Again on the right traj!"
			self.np_traj = self.np_right_lane_traj

		else:

			print "ERROR: Unknown trajecory in vehiclecontrolmodule.py: fixTrajAfterLaneChange() "

	def getTraj(self):

		self.traj = self.vehicle.getCurrentLaneTrajectory()
		if self.traj is self.left_lane_traj:
			return self.LEFT_LANE_STRING
		elif self.traj is self.center_lane_traj:
			return self.CENTER_LANE_STRING
		elif self.traj is self.right_lane_traj:
			return self.RIGHT_LANE_STRING
		else:
			return None

	def find_closest_trajectory_point(self, state, trajectory_to_search):
		'''
		This function will look for the closest point (in the trajectory) to state.
		The points in the trajectory that are searched range from current_idx to current_idx + number_points_ahead 
		(trajectory indexes wrap around)
		'''
		# TODO: Can be further improved by using the search_rangese

		np_state = numpy.array([[state[0]],[state[1]]])

		temp_distance = numpy.sum((trajectory_to_search[0:2, :] - np_state)**2, axis = 0)

		# Find the closest trajectory point that matches my desired speed and current heading
		best_idx = numpy.argmin(temp_distance)

		return best_idx

	def set_anchor_idx(self):
		'''
		Finds and sets the index of the closest trajectory point
		to the anchor node.
		Called by update_distance_to_anchor method.
		'''

		self.anchor_idx = self.find_closest_trajectory_point([self.vehicle.anchor_node.x, self.vehicle.anchor_node.y], self.np_traj)
		
		return

	def update_distance_to_anchor(self, current_idx):
		'''
		Functionality specific to the Intersection Scenario
		This will compute the distance along the trajectory
		to the anchor node.
		It is used for VACCC purposes, by the perception module.
		'''

		# If the anchor index is not yet defined, do it
		if not self.anchor_idx:

			self.set_anchor_idx()

		# The distance to the anchor along the trajectory is simply
		# the difference in indexes between the closest trajectory point
		# to the anchor, to our current trajectory points, multiplied of
		# course by the resolution of the trajectory.
		self.distance_to_anchor = (float(self.anchor_idx - current_idx))*(1./self.traj_points_per_meter)


	def compute_low_level_inputs(self):
		'''
		This function simply returns the current low level inputs
		'''
		# self.state = [-39.44159273512578, -38.36738366701975, -2.3967515204479533]
		# self.state comes in real world metres and radians

		# Get the index of the closest trajectory point
		best_idx = self.find_closest_trajectory_point(self.state, self.np_traj)

		if self.intersection_vehicle:

			self.update_distance_to_anchor(best_idx)

		current_closest_trajectory_point = best_idx

		# Add some look ahead to improve controller performance		
		best_idx += 5

		# Get the reference state corresponding to the trajectory point 
		# given by the closest_index + look_ahead
		traj_len = len( self.traj[0] )



		if best_idx > traj_len and not self.vehicle.cycle_traj:
		
			# If we are supposed to not cycle the trajectory, let's just stop at the end
			steering_command = 0.
			throttle_command = self.get_emergency_braking_throttle()

		else:

			reference_state = [self.traj[0][ (best_idx)%traj_len ] , self.traj[1][ (best_idx)%traj_len ] , self.traj[2][ (best_idx)%traj_len ] ]

			# Compute the controller action based on the current state and the reference state
			[throttle_command, steering_command] = self.get_controller_action(self.state, reference_state)

		

		# # Apply the computed commands 
		# self.current_velocity_command = velocity_command
		# self.current_steering_command = steering_command # Steering command should be outputteed in DEGREES		

		# Store the current closest index
		self.current_trajectory_id = current_closest_trajectory_point

		# Return the computed control commands
		return [throttle_command, steering_command]

	def reset_latfoll(self):
		# To make sure that the latfoll is reseted when it is not used
		self.last_latfoll_time = []
		self.latfoll_I_e = []
		self.last_latfoll_e = []

	def reset_CC(self):
		# To make sure that the CC is reseted when it is not used
		self.last_CC_time = []
		self.CC_I_e = []
		self.last_CC_e = []
	
	def reset_ACCC(self):
		# To make sure that the ACCC is reseted when it is not used
		self.last_ACCC_time = []
		self.ACCC_I_e = []
		self.last_ACCC_e = []

	def get_velocity_throttle(self, desired_velocity):
		'''
		This function runs a PID that outputs a throttle
		The error source of this PID is the error in the current velocity 
		and the desired velocity
		'''

		# Get the current time for PID computations
		current_time = time.time()

		# The current linear velocity is fetched
		linear_velocity = math.hypot(self.bodies_dict[self.vehicle_id].x_speed, self.bodies_dict[self.vehicle_id].y_speed)

		# Computing the error between desired velocity and current velocity
		current_error = desired_velocity - linear_velocity

		if self.last_CC_time:

			# Compute last sampling time
			time_passed = current_time - self.last_CC_time
			self.last_CC_time = current_time

			# Update integral error
			self.CC_I_e += time_passed*current_error

			# To avoid integration windup (i.e. big steady errors will result in big overshoots)
			max_integral_action = 1000.
			integral_action = self.CC_k_i*self.CC_I_e

			if math.fabs(integral_action) > max_integral_action:

				# We limit the integral action
				integral_action = math.copysign(max_integral_action, integral_action)

				# We limit the integrator memory
				self.CC_I_e = integral_action/self.CC_k_i

			# Compute the derivative of the error
			if time_passed >0:

				self.derivative_error = (current_error - self.last_CC_e)/time_passed

			self.last_CC_e = current_error

			# Compute PID output
			throttle_output = 0 + self.CC_k_p*current_error + integral_action + self.CC_k_d*self.derivative_error

		else:

			# Initialization of the ACCC controller (first iteration of this controller)
			self.last_CC_time = current_time
			self.last_CC_e = current_error
			self.CC_I_e = 0.0
			self.derivative_error = 0.0

			throttle_output = 0

		return throttle_output

	def get_controller_action(self, current_state, reference_state):
		'''
		Based on the current state and reference state, compute the control inputs to be applied to the car
		'''

		# Compute the steering input
		steering_command = self.get_steering_command(current_state, reference_state)


		if self.brake == True:
			self.vehicle.desired_velocity = 0.0

		# Compute the throttle input
		
		if self.vehicle.desired_velocity < 5./3.6:
			# If the desired velocity is set to a very small value (<5 km/h), we will
			# apply a very large negative throttle (corresponding to hitting the brakes hard)

			throttle_command = self.get_emergency_braking_throttle()
			
		elif self.ACCC or self.VACCC:
			# If in ACCC mode, we need to use the ACCC controller			

			throttle_command = self.get_ACCC_throttle()
			
		else:
			# Otherwise we are in CC mode, so we use the CC controller

			throttle_command = self.get_CC_throttle()

		throttle_OA = self.get_OA_throttle() # need to be computed before to get the self.OA_action value
		if self.OA and self.OA_action:
			throttle_command = throttle_OA

		# Return the command inputs
		command_inputs = [throttle_command, steering_command]

		return command_inputs

	def get_ACCC_throttle(self):
		'''
		Compute the throttle that will make the car follow a target car with a certain distance
		It also allows for the implementation of VACCC, since both controllers work
		on the same principle.
		'''

		# Proportional gain used to compensate for errors in distance to car in front
		K_distance_error = 0.3

		if self.VACCC:
		
			print "self.VACCC = " + str(self.VACCC)

			distance_to_other_car = self.VACCC_distance
			desired_distance = self.VACCC_desired_distance

		else:
			distance_to_other_car = self.ACCC_distance
			desired_distance = self.ACCC_desired_distance

		if distance_to_other_car != None:

			# The current distance error is simply the reference distance minus the desired distance
			current_error = distance_to_other_car - desired_distance

		else:

			# In case the ACCC is not yet set properly (no reference distance, simply avoid compensating for
			# the distance error)
			print "self.ACCC_distance = " + str(self.ACCC_distance)
			print "self.ACCC = " + str(self.ACCC)
			print "self.vehicle.id = " + str(self.vehicle.id)

			print "WARNING: Trying to run controller action without an ACCC_distance"
			current_error = 0

		# Simply compute the compensation to try to fix distance errors
		distance_velocity_compensation = K_distance_error * current_error

		max_distance_velocity_compensation_ratio = 5.

		if not self.ACCC_target_velocity:

			if distance_velocity_compensation*max_distance_velocity_compensation_ratio > self.vehicle.desired_velocity:

				distance_velocity_compensation = self.vehicle.desired_velocity/max_distance_velocity_compensation_ratio

			throttle_command = self.get_velocity_throttle(self.vehicle.desired_velocity + distance_velocity_compensation)

		else:


			if distance_velocity_compensation*max_distance_velocity_compensation_ratio > self.ACCC_target_velocity:

				distance_velocity_compensation = self.ACCC_target_velocity/max_distance_velocity_compensation_ratio

			throttle_command = self.get_velocity_throttle(self.ACCC_target_velocity + distance_velocity_compensation)

		return throttle_command

	def get_OA_throttle(self):
		'''
		Compute the throttle that will make the car avoid a target car with a certain distance
		'''

		# Proportional gain used to compensate for errors in distance to car in front
		K_distance_error = 0.3

		distance_to_other_car = self.OA_distance
		desired_distance = self.OA_desired_distance

		if distance_to_other_car != None:

			# The current distance error is simply the reference distance minus the desired distance
			current_error = distance_to_other_car - desired_distance
			if current_error < 0:
				self.OA_action = True
				return self.OA_deccelerate_throttle
		self.OA_action = False

		return 0

	def get_CC_throttle(self):
		'''


		Compute the throttle that will make the car have the velocity given by desired_velocity
		'''

		# Reseting the ACCC controller
		self.reset_ACCC()

		desired_velocity = self.vehicle.desired_velocity # Desired velocity comes in km/h, must convert to m/s

		# We simply tell the velocity controller to try to have a velocity corresponding to the desired velocity
		throttle_command = self.get_velocity_throttle(desired_velocity)

		return throttle_command


	def get_emergency_braking_throttle(self):
		'''
		This function simply returns the throttle corresponding to 
		a hard braking
		'''

		throttle_command = -10000.

		return throttle_command

	def get_tracking_error(self, current_state, reference_state):
		'''
		Computes the tracking error from the current state and a reference state
		The tracking error is composed of a longitudinal, lateral and orientation error
		'''

		error_x = math.cos( current_state[2] )*(reference_state[0] - current_state[0]) + math.sin( current_state[2] )*(reference_state[1] - current_state[1])
		error_y = -math.sin( current_state[2] )*(reference_state[0] - current_state[0]) + math.cos( current_state[2] )*(reference_state[1] - current_state[1])
		error_theta = reference_state[2] - current_state[2]

		# Orientation error must always be between [-Pi,Pi]
		while error_theta > math.pi:
			error_theta -= 2*math.pi
		while error_theta < -math.pi:
			error_theta += 2*math.pi

		return [error_x, error_y, error_theta]

	def get_steering_command(self, current_state, reference_state, max_steering_radians = 30):
		'''
		Computes the steering command based on the current state and reference state
		'''

		tracking_error = self.get_tracking_error(current_state, reference_state)


		# tracking_error[1] is the lateral error
		# tracking_error[2] is the orientation error

		# steering_command = 8.0* (10.0*tracking_error[1] + 15.0*tracking_error[2]) # Stable for unicycle model
		steering_command = 1.0* (1.0*(tracking_error[1] + self.vehicle.lane_offset) + 1.5*tracking_error[2]) 

		# Need to limit the steering command to 30 degrees
		if steering_command > math.radians(max_steering_radians):
			steering_command = math.radians(max_steering_radians)

		if steering_command < -math.radians(max_steering_radians):
			steering_command = -math.radians(max_steering_radians)

		return steering_command
	
