import socket, sys, time, math, random

sys.path.append('Lanelets')
import laneletmodule
import laneletlibrary





class PlatooningVehicle:
	"This class is the Platooning Vehicle class used by the Platooning Manager Module"
	def __init__(self, simulation_period, vehicle_id):

		if not isinstance(vehicle_id, int):
			raise NameError('In class PlatooningVehicle: constructor: id must be an integer')

		self.id = vehicle_id
		self.simulation_period = 1./simulation_period
		self.state = []
		self.input = [0, 0]
		self.time = -1
		self.current_time_delay = 0.0
		self.current_trajectory_id = 0
		# self.velocity = 50.0 + 45.0*random.random()
		self.velocity = 30.0 + 10.0*random.random()

		self.vehicle_iteration = 0

		self.traj = []
		self.right_lane_traj = []
		self.center_lane_traj = []
		self.left_lane_traj = []

		self.axis_length = 1.2

		self.changing_lane = False
		self.desired_lane = "right_lane"
		self.boolean_desired_lane_traj = []

		self.debug_lane_changes_time = time.time()



		self.RIGHT_LANE_STRING = "right_lane"
		self.CENTER_LANE_STRING = "center_lane"
		self.LEFT_LANE_STRING = "left_lane"

		self.current_lane = self.RIGHT_LANE_STRING

		self.start_time_debugger_variable = time.time()

		#Tips
		# self.state = [real_world_meters, radians]
		# print "self.state = " + str(self.state)
		# closeby_body_readings = [real_world_meters, radians]
		# print "closeby_body_readings = " + str(closeby_body_readings)
		# self.traj[0] in real_world_meters
		# self.traj[2] in radians
		# print "self.traj[2] = " + str(self.traj[2])

	def set_initial_state(self, state_vector):

		for dimension in range( len( state_vector ) ):

			self.state.append( state_vector[dimension] )

	def set_vehicle_on_trajectory_state(self, idx = 0):

		# print "!!!!!!!!START!!!!!!!!!!!!!!!!!!!!!set_vehicle_on_trajectory_state"

		self.current_trajectory_id = idx

		if len( self.traj ) == 0:

			raise NameError('In class PlatooningVehicle: trying to use set_vehicle_on_trajectory_state when trajectory is not yet defined')

		if len( self.state ) == 0:
			#Uninitialized state, must create it

			for dimension in range( len( self.traj ) - 1 ):

				self.state.append( self.traj[dimension][idx] )

		else:
			#Must overwrite the state

			for dimension in range( len( self.state ) ):

				self.state[dimension] = self.traj[dimension][idx]

		self.state[2] = math.degrees(self.state[2])


		# print "self.traj[dimension][idx]  = " + str(self.traj[dimension][idx] )
		# print "self.state = " + str(self.state)

		# print "!!!!!!!!END!!!!!!!!!!!!!!!!!!!!!set_vehicle_on_trajectory_state"

	def set_initial_input(self, input_vector):	

		for dimension in range( len( input_vector ) ):

			self.input.append( input_vector[dimension] )

	def set_current_input(self, input_vector):	

		for dimension in range( len( input_vector ) ):

			self.input[dimension] = input_vector[dimension]

	def set_current_time(self, current_time):	

		self.time = current_time

	def set_lane_trajectory(self, traj_x, traj_y, desired_velocity, lane_position):

		# print "DO ME"
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

			raise NameError("PlatooningVehicle Trajectory creation resulted in a mistake!")

		if lane_position == "right_lane":

			self.right_lane_traj = [traj_x, traj_y, traj_theta, traj_time]

		else:
		
			if lane_position == "center_lane":

				self.center_lane_traj = [traj_x, traj_y, traj_theta, traj_time]

			else:

				if lane_position == "left_lane":

					self.left_lane_traj = [traj_x, traj_y, traj_theta, traj_time]

				else:

					print "ERROR PlatooningVehicle: No suitable lane found!"


		self.traj = self.left_lane_traj

		# print traj_time

		return

	def get_current_state(self, radians = True):

		state_vector = []

		state_vector.append( self.state[0] )
		state_vector.append( self.state[1] )

		if radians:

			state_vector.append( self.state[2] )

		else:

			state_vector.append( math.degrees(self.state[2]) )

		return state_vector

	def get_current_input(self):

		input_vector = []

		for dimension in range( len( self.input ) ):

			input_vector.append( self.input[dimension] )

		return input_vector

	def get_current_time(self):

		return self.time

	def get_id(self):

		return self.id

	def simulate(self, current_time):

		if self.time == -1:
			# print str(self.id) + "first_simulate() "
			self.time = current_time
			return



		time_passed = current_time - self.time
		self.time = current_time

		# print str(self.id) + " time_passed = " + str(time_passed)
		# print str(self.id) + " self.state = " + str(self.state)

		self.state[0] = self.state[0] + time_passed*self.input[0]*math.cos( self.state[2] )
		self.state[1] = self.state[1] + time_passed*self.input[0]*math.sin( self.state[2] )
		self.state[2] = self.state[2] + time_passed*(self.input[0]/self.axis_length)*math.tan( self.input[1] )

		while ( self.state[2] > math.pi ):
			self.state[2] = self.state[2] - 2*math.pi

		while ( self.state[2] < -math.pi ):
			self.state[2] = self.state[2] + 2*math.pi

		# print str(self.id) + " self.state = " + str(self.state)

	def get_current_reference(self, current_time):

		# print "get_current_reference"

		last_trajectory_time = self.traj[3][-1]

		while current_time > last_trajectory_time:

			current_time = current_time - last_trajectory_time

		best_time_distance = 10e10
		best_idx = -1

		for idx in range( len( self.traj[3] ) ):

			temp_distance = math.fabs( current_time - self.traj[3][idx] )

			if temp_distance < best_time_distance:

				best_time_distance = temp_distance
				best_idx = idx

		current_reference = []

		# print "best_idx" + str( best_idx )

		for dimension in range( 3 ):

			current_reference.append( self.traj[dimension][best_idx] )

		# print "current_reference" + str(current_reference)

		# print "end get_current_reference"

		return current_reference

	def get_ahead_reference(self):

		# print "get_current_reference"

		look_ahead_distance = 0.5

		ahead_point_x = self.state[0] + (look_ahead_distance + self.axis_length )*math.cos( self.state[2] )
		ahead_point_y = self.state[1] + (look_ahead_distance + self.axis_length )*math.sin( self.state[2] )

		best_distance = 10e10
		best_idx = -1

		for idx in range( len( self.traj[3] ) ):

			current_distance = math.hypot( ahead_point_x - self.traj[0][idx], ahead_point_y - self.traj[1][idx] )

			if current_distance < best_distance:

				best_distance = current_distance
				best_idx = idx

		current_reference = []

		for dimension in range( 3 ):

			current_reference.append( self.traj[dimension][best_idx] )

		return current_reference

	def get_closest_reference(self):

		best_distance = 10e10
		best_idx = -1

		for idx in range( len( self.traj[3] ) ):

			current_distance = math.hypot( self.state[0] - self.traj[0][idx], self.state[1] - self.traj[1][idx] )

			if current_distance < best_distance:

				best_distance = current_distance
				best_idx = idx

		current_reference = []

		for dimension in range( 3 ):

			current_reference.append( self.traj[dimension][best_idx] )

		return current_reference

	def controller_action(self, current_state, current_reference):

		tracking_error = self.tracking_error(current_state, current_reference)
		# print "tracking_error = " + str(tracking_error) 
		while tracking_error[2] > math.pi:
			tracking_error[2] = tracking_error[2] - math.pi
		while tracking_error[2] < -math.pi:
			tracking_error[2] = tracking_error[2] + math.pi
		# current_state[2] comes in radians
		# current_reference[2] comes in radians
		# current_state[2] = math.radians( current_state[2] )
		# print "current_state = " + str(current_state) 
		# print "current_reference = " + str(current_reference)
		# current_reference[2] comes in radians
		# print "tracking_error = " + str(tracking_error)
		# velocity_command = tracking_error[0]*0.01
		velocity_command = 0.35

		steering_command = 8.0* (10.0*tracking_error[1] + 15.0*tracking_error[2])

		# print "\n\n\n\nsteering_command = " + str(steering_command) 

		# steering_command = -steering_command
		# max_steering_amplitude = 100
		# steering_command = max( min(max_steering_amplitude, steering_command) , -max_steering_amplitude)
		# print "steering_command = " + str(steering_command) 


		# Working well with the normal Scania city
		# steering_command = 8.0* (10.0*tracking_error[1] + 10.0*tracking_error[2])

		# steering_command_max = math.radians(20)

		# if steering_command > steering_command_max:
		# 	# print "Max steering reached"
		# 	steering_command = steering_command_max

		# if steering_command < -steering_command_max:
		# 	# print "Max steering reached"
		# 	steering_command = -steering_command_max

		command_inputs = [velocity_command, steering_command]
		# print "command_inputs = " + str(command_inputs)
		return command_inputs

	def tracking_error(self, current_state, current_reference):

		error_x = math.cos( current_state[2] )*(current_reference[0] - current_state[0]) + math.sin( current_state[2] )*(current_reference[1] - current_state[1])
		error_y = -math.sin( current_state[2] )*(current_reference[0] - current_state[0]) + math.cos( current_state[2] )*(current_reference[1] - current_state[1])
		error_theta = current_reference[2] - current_state[2]

		# print "current_reference[2] = " + str(current_reference[2])
		# print "current_state[2] = " + str(current_state[2])

		# print "error_theta = " + str(error_theta)

		while error_theta > math.pi:
			error_theta -= 2*math.pi
		while error_theta < -math.pi:
			error_theta += 2*math.pi

		# print "error_theta = " + str(error_theta)


		return [error_x, error_y, error_theta]
		# error_x = math.cos( current_state[2] )*(current_reference[0] - current_state[0]) + math.sin( current_state[2] )*(current_reference[1] - current_state[1])

	def fake_trajectory_tracking(self, iteration):

		number_trajectory_points = len( self.traj[0] )

		while iteration >= number_trajectory_points:

			iteration = iteration - number_trajectory_points

		self.set_vehicle_on_trajectory_state(iteration)

		return

	def is_colliding_soon(self, current_time, body_readings):

		# last_trajectory_time = self.traj[3][-1]

		# while current_time > last_trajectory_time:

		# 	current_time = current_time - last_trajectory_time

		# desired_velocity = 30.0 / 3.6

		# next_state = [ self.state[0] + desired_velocity*0.03*math.cos( self.state[2] ) , self.state[1] + desired_velocity*0.03*math.sin( self.state[2] ) ]  

		best_distance = 10e10
		best_idx = 10e10

		best_idx = self.current_trajectory_id
		number_points_ahead = 10
		cnt = 0

		meters_ahead = 10.0
		meters_ahead_truck = 20.0

		future_traj = []
		future_traj_truck = []

		cnt = 1
		distance_moved = 0

		while True:

			if best_idx + cnt > len(self.traj[0]) - 1:

				cnt = cnt - len(self.traj[0]) - 1

			distance_moved = distance_moved + math.hypot( self.traj[0][best_idx+cnt] - self.traj[0][best_idx+cnt-1] , self.traj[1][best_idx+cnt] - self.traj[1][best_idx+cnt-1] )

			# print " len(self.traj[0]) " + str ( len(self.traj[0]) )
			# print " best_idx+cnt " + str ( best_idx+cnt )
			# print " self.traj[:][best_idx+cnt] " + str( self.traj[:][best_idx+cnt] )

			# future_traj.append( self.traj[:][best_idx+cnt] ) 
			future_traj_truck.append( [ self.traj[0][best_idx+cnt], self.traj[1][best_idx+cnt], self.traj[2][best_idx+cnt] ] ) 

			if distance_moved < meters_ahead:

				future_traj.append( [ self.traj[0][best_idx+cnt], self.traj[1][best_idx+cnt], self.traj[2][best_idx+cnt] ] ) 

			cnt = cnt + 1

			if distance_moved > meters_ahead_truck:

				break

		# print "len( future_traj ) = " + str( len( future_traj ) )

		colliding_distance = 1.5

		mega_safe_distance = 2.0*meters_ahead

	 	# print "body_readings['readings'] = " + str(body_readings['readings'])



		for idx, body_reading in enumerate( body_readings['readings'] ):

			# print "Here"
			# print "body_reading[1] = " + str(body_reading[1])
			temp_body_reading = []
			temp_body_reading.append( body_reading[0]*32 )
			temp_body_reading.append( body_reading[1]*32 )

			temp_body_reading.append( math.radians( body_reading[2] ) )

			is_trailer = False

			axis_length_obstacle_avoidance = self.axis_length

			# print "body_readings = " + str(body_readings )

			if len( body_reading ) == 4:

				if body_reading[3] == 'trailer':

				# if body_reading[idx]['trailer']:

					# Putting the axis length in such a way that the front point is in the middle of the trailer
					axis_length_obstacle_avoidance = 0.08*32


			if math.hypot(future_traj[0][0] - temp_body_reading[0] , future_traj[0][1] - temp_body_reading[1]) > mega_safe_distance:

				# Vehicle is super far away, I do not need to check it
				continue

			# print "temp_body_reading[2] = " + str(temp_body_reading[2])

			distance_angle = math.atan2( future_traj[0][1] - temp_body_reading[1], future_traj[0][0] - temp_body_reading[0] )

			angle_difference = distance_angle - future_traj[0][2] 

			while angle_difference < 0:
				angle_difference = angle_difference + 2.0*math.pi

			while angle_difference > 2.0*math.pi:
				angle_difference = angle_difference - 2.0*math.pi

			if angle_difference < math.pi/2.0 or angle_difference > math.pi*(3.0/2.0):

				# print "Vehicle is behind me, I do not need to check it"
				continue


			for future_state in future_traj:

				other_vehicle_front_axle = []
				# SELF.AXIS_LENGTH SHOULD BE OTHER VEHICLE'S AXIS LENGTH
				other_vehicle_front_axle.append( temp_body_reading[0] + axis_length_obstacle_avoidance*math.cos( temp_body_reading[2] ) )
				other_vehicle_front_axle.append( temp_body_reading[1] + axis_length_obstacle_avoidance*math.sin( temp_body_reading[2] ) )

				future_state_front_axle = []
				future_state_front_axle.append( future_state[0] + axis_length_obstacle_avoidance*math.cos(future_state[2]) )
				future_state_front_axle.append( future_state[1] + axis_length_obstacle_avoidance*math.sin(future_state[2]) )

				# # Check for collision with rear axle
				# if math.hypot(future_state[0] - other_vehicle_state[0] , future_state[1] - other_vehicle_state[1]) < colliding_distance:

				# 	return True

				# # Check for collision with front axle
				# if math.hypot(future_state[0] - other_vehicle_front_axle[0] , future_state[1] - other_vehicle_front_axle[1]) < colliding_distance:

				# 	return True

				# Check my front axle for collision with front axle
				if math.hypot(future_state_front_axle[0] - other_vehicle_front_axle[0] , future_state_front_axle[1] - other_vehicle_front_axle[1]) < colliding_distance:

					# print "my front axle in collision with their front axle. my id =" + str(self.id)

					return True

				# Check my front axle for collision with rear axle
				if math.hypot(future_state_front_axle[0] - temp_body_reading[0] , future_state_front_axle[1] - temp_body_reading[1]) < colliding_distance:

					# print "my front axle in collision with their rear axle. my id =" + str(self.id)

					return True

				# Check my front axle for collision with front axle
				if math.hypot(future_state[0] - other_vehicle_front_axle[0] , future_state[1] - other_vehicle_front_axle[1]) < colliding_distance:

					# print "my rear axle in collision with their front axle. my id =" + str(self.id)

					return True

				# Check my front axle for collision with rear axle
				if math.hypot(future_state[0] - temp_body_reading[0] , future_state[1] - temp_body_reading[1]) < colliding_distance:

					# print "my rear axle in collision with their rear axle. my id =" + str(self.id)

					return True

				

		return False


	def new_distance_to_car_in_future_trajectory(self, current_closest_trajectory_point, body_readings, safety_distance):
		# I will compute the distance to a car that is overlapping with my close future trajectory
		# Returns 100.0 if no car is detected

		closest_distance = 10e10
		closest_distance_idx = None

		# meters_ahead_search = safety_distance*1.5
		meters_ahead_search = 10.
		colliding_distance = 1.5
		mega_safe_distance = 2.0*meters_ahead_search

		future_traj = [[],[],[]]
		future_traj_distance_traveled = []

		temp_cnt = 0
		temp_distance_traveled = 0
		traj_length = len( self.traj[0] )

		while True:

			current_idx = current_closest_trajectory_point + temp_cnt
			next_idx = current_idx + 1

			current_idx = current_idx % traj_length
			next_idx = next_idx % traj_length
			
			current_distance_travelled = math.hypot( self.traj[0][next_idx] - self.traj[0][current_idx] ,
													self.traj[1][next_idx] - self.traj[1][current_idx] )

			temp_distance_traveled += current_distance_travelled

			if temp_distance_traveled > meters_ahead_search:

				break

			future_traj_distance_traveled.append( temp_distance_traveled )
			future_traj[0].append( self.traj[0][next_idx] )
			future_traj[1].append( self.traj[1][next_idx] )
			future_traj[2].append( self.traj[2][next_idx] )

			temp_cnt += 1

		# print "len(future_traj_distance_traveled) = " + str(len(future_traj_distance_traveled))
		# print "future_traj[0] = " + str(future_traj[0])
		# print "future_traj[1] = " + str(future_traj[1])
		# future_traj[2]

		#Tips
		# self.state = [real_world_meters, radians]
		# print "self.state = " + str(self.state)
		# closeby_body_readings = [real_world_meters, radians]
		# print "closeby_body_readings = " + str(closeby_body_readings)
		# self.traj[0] in real_world_meters
		# self.traj[2] in radians
		# print "self.traj[2] = " + str(self.traj[2])


		# Remove the body readings that are farther away than mega_safe_distance
		relevant_body_readings = []

		# print "mega_safe_distance = " + str(mega_safe_distance)

		for temp_body_reading in body_readings:

			distance = math.hypot( temp_body_reading[0] - self.state[0] , temp_body_reading[1] - self.state[1] )

			# print "distance = " + str(distance)

			if distance < mega_safe_distance:

				relevant_body_readings.append(temp_body_reading)

		# print "len( body_readings ) = " + str( len( body_readings ) )
		# print "len( relevant_body_readings ) = " + str( len( relevant_body_readings ) )
		# print "body_readings = " + str(body_readings)
		# print "[self.state[0], self.state[1]] = " + str([self.state[0], self.state[1]])

		for temp_body_reading in relevant_body_readings:

			for traj_point_idx in xrange( len( future_traj ) ):

				distance_to_body_reading = math.hypot( temp_body_reading[0] - future_traj[0][traj_point_idx] , temp_body_reading[1] - future_traj[1][traj_point_idx] )
				ahead_distance = future_traj_distance_traveled[traj_point_idx]

				if distance_to_body_reading < colliding_distance and ahead_distance < closest_distance:
 
					closest_distance = ahead_distance
					# closest_distance_idx = traj_point_idx

					break

		print "closest_distance = " + str(closest_distance)

	def distance_to_car_in_future_trajectory(self, current_time, body_readings, safety_distance):
		# I will compute the distance to a car that is overlapping with my close future trajectory
		# Returns 100.0 if no car is detected

		best_distance = 10e10
		best_idx = 10e10

		best_idx = self.current_trajectory_id
		number_points_ahead = 10
		cnt = 0

		# meters_ahead = 10.0
		meters_ahead = 1.5*safety_distance
		colliding_distance = 1.5
		mega_safe_distance = 2.0*meters_ahead

		future_traj = []
		future_traj_distance_traveled = []

		future_traj.append( [ self.state[0], self.state[1], self.state[2] ] ) 
		future_traj_distance_traveled.append( 0.1 )

		# print str(self.id) + " self.state = " + str(self.state)

		# self.traj[0] in real_world_meters
		# self.traj[2] in radians
		# print "self.traj[2] = " + str(self.traj[2])

		future_traj.append( [ self.traj[0][best_idx], self.traj[1][best_idx], self.traj[2][best_idx] ] ) 
		future_traj_distance_traveled.append( 0.2 )

		# print str(self.id) + " [ self.traj[0][best_idx], self.traj[1][best_idx], self.traj[2][best_idx] ] = " + str([ self.traj[0][best_idx], self.traj[1][best_idx], self.traj[2][best_idx] ])
		
		cnt = 1
		distance_moved = 0

		while True:

			if best_idx + cnt > len(self.traj[0]) - 1:

				cnt = cnt - len(self.traj[0]) - 1

			distance_moved = distance_moved + math.hypot( self.traj[0][best_idx+cnt] - self.traj[0][best_idx+cnt-1] , self.traj[1][best_idx+cnt] - self.traj[1][best_idx+cnt-1] )

			# print " len(self.traj[0]) " + str ( len(self.traj[0]) )
			# print " best_idx+cnt " + str ( best_idx+cnt )
			# print " self.traj[:][best_idx+cnt] " + str( self.traj[:][best_idx+cnt] )

			# future_traj.append( self.traj[:][best_idx+cnt] ) 

			if distance_moved < meters_ahead:

				future_traj.append( [ self.traj[0][best_idx+cnt], self.traj[1][best_idx+cnt], self.traj[2][best_idx+cnt] ] ) 
				future_traj_distance_traveled.append( distance_moved )


			else:

				break

			cnt = cnt + 1
			

		# print "len( future_traj ) = " + str( len( future_traj ) )


	 	# print "body_readings['readings'] = " + str(body_readings['readings'])

	 	closest_car_distance = 100.

		for idx, body_reading in enumerate( body_readings ):

			# print "Here"
			# print "body_reading[1] = " + str(body_reading[1])
			temp_body_reading = []
			temp_body_reading.append( body_reading[0] )
			temp_body_reading.append( body_reading[1] )

			temp_body_reading.append( body_reading[2] )

			is_trailer = False

			axis_length_obstacle_avoidance = self.axis_length

			# print "body_readings = " + str(body_readings )

			if len( body_reading ) == 4:

				if body_reading[3] == 'trailer':

				# if body_reading[idx]['trailer']:

					# Putting the axis length in such a way that the front point is in the middle of the trailer
					axis_length_obstacle_avoidance = 0.08*32


			if math.hypot(future_traj[0][0] - temp_body_reading[0] , future_traj[0][1] - temp_body_reading[1]) > mega_safe_distance:

				# Vehicle is super far away, I do not need to check it
				continue

			# print "temp_body_reading[2] = " + str(temp_body_reading[2])

			distance_angle = math.atan2( future_traj[0][1] - temp_body_reading[1], future_traj[0][0] - temp_body_reading[0] )

			angle_difference = distance_angle - future_traj[0][2] 

			while angle_difference < 0:
				angle_difference = angle_difference + 2.0*math.pi

			while angle_difference > 2.0*math.pi:
				angle_difference = angle_difference - 2.0*math.pi

			if angle_difference < math.pi/2.0 or angle_difference > math.pi*(3.0/2.0):

				# print "Vehicle is behind me, I do not need to check it"
				# continue
				pass

			closest_car_distance = 100.

			# print "future_traj = " + str(future_traj)

			for future_state_idx, future_state in enumerate(future_traj):

				other_vehicle_front_axle = []
				# SELF.AXIS_LENGTH SHOULD BE OTHER VEHICLE'S AXIS LENGTH
				other_vehicle_front_axle.append( temp_body_reading[0] + axis_length_obstacle_avoidance*math.cos( temp_body_reading[2] ) )
				other_vehicle_front_axle.append( temp_body_reading[1] + axis_length_obstacle_avoidance*math.sin( temp_body_reading[2] ) )

				future_state_front_axle = []
				future_state_front_axle.append( future_state[0] + axis_length_obstacle_avoidance*math.cos(future_state[2]) )
				future_state_front_axle.append( future_state[1] + axis_length_obstacle_avoidance*math.sin(future_state[2]) )

				# # Check for collision with rear axle
				# if math.hypot(future_state[0] - other_vehicle_state[0] , future_state[1] - other_vehicle_state[1]) < colliding_distance:

				# 	return True

				# # Check for collision with front axle
				# if math.hypot(future_state[0] - other_vehicle_front_axle[0] , future_state[1] - other_vehicle_front_axle[1]) < colliding_distance:

				# 	return True

				# print "future_traj_distance_traveled[ future_state_idx ] = " + str(future_traj_distance_traveled[ future_state_idx ]) 

				# Check my front axle for collision with front axle
				current_distance = math.hypot(future_state_front_axle[0] - other_vehicle_front_axle[0] , future_state_front_axle[1] - other_vehicle_front_axle[1])
				if current_distance < colliding_distance:
					if future_traj_distance_traveled[ future_state_idx ] < closest_car_distance:
						closest_car_distance = future_traj_distance_traveled[ future_state_idx ]
					print "my front axle in collision with their front axle. my id =" + str(self.id)
					continue
					# return future_traj_distance_traveled[ future_state_idx ]
					# return True

				# Check my front axle for collision with rear axle
				current_distance = math.hypot(future_state_front_axle[0] - temp_body_reading[0] , future_state_front_axle[1] - temp_body_reading[1])
				if current_distance < colliding_distance:
					if future_traj_distance_traveled[ future_state_idx ] < closest_car_distance:
						closest_car_distance = future_traj_distance_traveled[ future_state_idx ]
					print "my front axle in collision with their rear axle. my id =" + str(self.id)
					continue
					# return future_traj_distance_traveled[ future_state_idx ]
					# return True

				# Check my front axle for collision with front axle
				current_distance = math.hypot(future_state[0] - other_vehicle_front_axle[0] , future_state[1] - other_vehicle_front_axle[1])
				if current_distance < colliding_distance:
					if future_traj_distance_traveled[ future_state_idx ] < closest_car_distance:
						closest_car_distance = future_traj_distance_traveled[ future_state_idx ]
					print "my rear axle in collision with their front axle. my id =" + str(self.id)
					continue
					# return future_traj_distance_traveled[ future_state_idx ]
					# return True

				# Check my front axle for collision with rear axle
				current_distance = math.hypot(future_state[0] - temp_body_reading[0] , future_state[1] - temp_body_reading[1])
				if current_distance < colliding_distance:
					if future_traj_distance_traveled[ future_state_idx ] < closest_car_distance:
						closest_car_distance = future_traj_distance_traveled[ future_state_idx ]
					print "my rear axle in collision with their rear axle. my id =" + str(self.id)
					continue
					# return future_traj_distance_traveled[ future_state_idx ]
					# return True


		# if closest_car_distance < safety_distance:
				
		# 	print "future_traj_distance_traveled = " + str(future_traj_distance_traveled)

		return closest_car_distance

	def fake_controller_trajectory_tracking(self, current_time, body_readings):

		desired_velocity = self.velocity / 3.6

		next_state = [ self.state[0] + desired_velocity*self.simulation_period*math.cos( self.state[2] ) , self.state[1] + desired_velocity*self.simulation_period*math.sin( self.state[2] ) ]  

		best_distance = 10e10
		best_idx = 10e10

		current_idx = self.current_trajectory_id
		# number_points_ahead = 10
		cnt = 0


		while True:

			temp_distance = math.hypot( next_state[0] - self.traj[0][current_idx] , next_state[1] - self.traj[1][current_idx] )

			if temp_distance < best_distance:

				best_distance = temp_distance
				best_idx = current_idx


			current_idx = current_idx + 1

			if current_idx > len( self.traj[0] ) - 1:

				current_idx = 0

			cnt = cnt + 1

			if cnt > 50:

				break

		# for idx in range( self.current_trajectory_id, len( self.traj[3] ) ):

		# 	temp_distance = math.hypot( next_state[0] - self.traj[0][idx] , next_state[1] - self.traj[1][idx] )

		# 	if temp_distance < best_distance:

		# 		best_distance = temp_distance
		# 		best_idx = idx


		

		if not self.is_colliding_soon(current_time, body_readings):

			if best_idx == self.current_trajectory_id:

				print "best_idx = " + str(best_idx)
				print "self.current_trajectory_id = " + str(self.current_trajectory_id)

				print "Must manually increase"
				best_idx = best_idx + 1
				print "best_idx = " + str(best_idx)
				print "len( self.traj[0] ) = " + str(len( self.traj[0] ))

				print "self.current_trajectory_id = " + str(self.current_trajectory_id)
				print "self.traj[0][self.current_trajectory_id] = " + str(self.traj[0][self.current_trajectory_id])
				print "self.traj[1][self.current_trajectory_id] = " + str(self.traj[1][self.current_trajectory_id])

			if best_idx > len( self.traj[0] ) - 1:

				print "TELEPORTAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAL"
				
				best_idx = 0
				# print "self.traj[best_idx] = " + str(self.traj[best_idx])
				print "best_idx = " + str(best_idx)
				print "self.traj[0][best_idx] = " + str(self.traj[0][best_idx])
				print "self.traj[1][best_idx] = " + str(self.traj[1][best_idx])
				print "best_idx+1 = " + str(best_idx+1)
				print "self.traj[0][best_idx+1] = " + str(self.traj[0][best_idx+1])
				print "self.traj[1][best_idx+1] = " + str(self.traj[1][best_idx+1])
			self.set_vehicle_on_trajectory_state(best_idx)
			return True

		else:

			# print "Colliding SOON " + str( time.time() )
			# warning_bool = bool( random.getrandbits(1) )
			# # 

			# if warning_bool:
			# 	print "WARNING"
			# else:
			# 	print "CRASH"


			# self.current_time_delay = self.current_time_delay - 0.03
			# print "Did not move the trajectory"
			return False

	def find_closest_trajectory_point(self, state, current_idx, number_points_ahead):
		# This function will look for the closest point (in the trajectory) to state.
		# The points in the trajectory that are searched range from current_idx to current_idx + number_points_ahead 
		# (trajectory indexes wrap around)

		# print "find_closest_trajectory_point()"

		whole_search_range = range(len(self.traj[0]))
		search_range = range(0, number_points_ahead)
		traj_len = len( self.traj[0] )


		# print "state = " + str(state)

		# print "self.traj[1] = " + str(self.traj[1])
		# self.traj[1] comes in real world metres

		# print "self.traj[2] = " + str(self.traj[2])
		# self.traj[2] comes in radians



		# temp_distance = [math.hypot( state[0]  - self.traj[0][ (current_idx+i)%traj_len ] , state[1] - self.traj[1][ (current_idx+i)%traj_len ] ) for i in search_range]
		temp_distance = [math.hypot( state[0]  - self.traj[0][ (i)%traj_len ] , state[1] - self.traj[1][ (i)%traj_len ] ) for i in whole_search_range]

		# Find the closest trajectory point that matches my desired speed and current heading
		best_distance = min(temp_distance)
		best_idx = temp_distance.index(best_distance)

		return [best_distance, best_idx]

	def change_lane(self, current_lane, next_lane, current_index, meters_of_merging):

		current_lane_traj = []

		if current_lane == self.RIGHT_LANE_STRING:

			current_lane_traj = self.right_lane_traj

		else:

			if current_lane == self.CENTER_LANE_STRING:

				current_lane_traj = self.center_lane_traj

			else:


				if current_lane == self.LEFT_LANE_STRING:

					current_lane_traj = self.left_lane_traj

				else:
				
					print "In PlatooningVehicle change_lane(): current_lane not found!"

		next_lane_traj = []

		if next_lane == self.RIGHT_LANE_STRING:

			next_lane_traj = self.right_lane_traj

		else:

			if next_lane == self.CENTER_LANE_STRING:

				next_lane_traj = self.center_lane_traj

			else:

				if next_lane == self.LEFT_LANE_STRING:

					next_lane_traj = self.left_lane_traj

				else:
				
					print "In PlatooningVehicle change_lane(): current_lane not found!"

		print "len( current_lane_traj ) = " + str(len( current_lane_traj ))
		print "len( current_lane_traj[0] ) = " + str(len( current_lane_traj[0] ))

		print "len( next_lane_traj ) = " + str(len( next_lane_traj ))
		print "len( next_lane_traj[0] ) = " + str(len( next_lane_traj[0] ))


		current_lane_traj_len = len( current_lane_traj[0] )

		travelled_distance = 0
		final_merge_id_offset = 0

		while travelled_distance < meters_of_merging:

			temp_id = current_index + final_merge_id_offset
			temp_id = temp_id%current_lane_traj_len
			temp_next_id = temp_id + 1
			temp_next_id = temp_next_id%current_lane_traj_len

			travelled_distance += math.hypot( current_lane_traj[0][temp_next_id] - current_lane_traj[0][temp_id] , current_lane_traj[1][temp_next_id] - current_lane_traj[1][temp_id] )

			final_merge_id_offset += 1

		final_merge_id = current_index + final_merge_id_offset
		final_merge_id = final_merge_id%current_lane_traj_len
		final_merge_point = [current_lane_traj[0][final_merge_id], current_lane_traj[1][final_merge_id]]

		current_lane_final_merge_id = final_merge_id

		whole_search_range = range(len(self.traj[0]))
		next_lane_traj_len = len( next_lane_traj[0] )

		temp_distance = [math.hypot( final_merge_point[0]  - next_lane_traj[0][ (i)%next_lane_traj_len ] , final_merge_point[1] - next_lane_traj[1][ (i)%next_lane_traj_len ] ) for i in whole_search_range]

		# Find the closest trajectory point that matches my desired speed and current heading
		best_distance = min(temp_distance)
		best_idx = temp_distance.index(best_distance)

		
		next_lane_final_merge_id = best_idx

		

		initial_merge_point = [current_lane_traj[0][current_index], current_lane_traj[1][current_index]]

		temp_distance = [math.hypot( initial_merge_point[0]  - self.traj[0][ (i)%next_lane_traj_len ] , initial_merge_point[1] - self.traj[1][ (i)%next_lane_traj_len ] ) for i in whole_search_range]

		# Find the closest trajectory point that matches my desired speed and current heading
		best_distance = min(temp_distance)
		best_idx = temp_distance.index(best_distance)


		current_lane_start_merge_id = current_index
		next_lane_start_merge_id = best_idx

		print "current_lane_start_merge_id = " + str(current_lane_start_merge_id)
		print "current_lane_final_merge_id = " + str(current_lane_final_merge_id)

		print "next_lane_start_merge_id = " + str(next_lane_start_merge_id)
		print "next_lane_final_merge_id = " + str(next_lane_final_merge_id)

		num_points_next_traj_merge = next_lane_final_merge_id - next_lane_start_merge_id
		if num_points_next_traj_merge < 0:
			num_points_next_traj_merge += next_lane_traj_len

		num_points_current_traj_merge = current_lane_final_merge_id - current_lane_start_merge_id
		if num_points_current_traj_merge < 0:
			num_points_current_traj_merge += current_lane_traj_len

		current_lane_merging_traj = []
		next_lane_merging_traj = []

		if next_lane_final_merge_id > next_lane_start_merge_id:
			next_lane_merging_traj = [ next_lane_traj[0][next_lane_start_merge_id:next_lane_final_merge_id],
										next_lane_traj[1][next_lane_start_merge_id:next_lane_final_merge_id],
										next_lane_traj[2][next_lane_start_merge_id:next_lane_final_merge_id],
										next_lane_traj[3][next_lane_start_merge_id:next_lane_final_merge_id] ]
		else:
			next_lane_merging_traj = [ next_lane_traj[0][next_lane_start_merge_id:], next_lane_traj[0][:next_lane_final_merge_id],
										next_lane_traj[1][next_lane_start_merge_id:], next_lane_traj[1][:next_lane_final_merge_id],
										next_lane_traj[2][next_lane_start_merge_id:], next_lane_traj[2][:next_lane_final_merge_id],
										next_lane_traj[3][next_lane_start_merge_id:], next_lane_traj[3][:next_lane_final_merge_id] ]

		if current_lane_final_merge_id > current_lane_start_merge_id:
			# print "HERE"
			# print "current_lane_start_merge_id = " + str(current_lane_start_merge_id)
			# print "current_lane_final_merge_id = " + str(current_lane_final_merge_id)
			# print "len(current_lane_traj) = " + str(len(current_lane_traj))
			# print "len(current_lane_traj[0]) = " + str(len(current_lane_traj[0]))
			# print "[current_lane_start_merge_id:current_lane_final_merge_id] = " + str([current_lane_start_merge_id:current_lane_final_merge_id])
			current_lane_merging_traj = [current_lane_traj[0][current_lane_start_merge_id:current_lane_final_merge_id],
										current_lane_traj[1][current_lane_start_merge_id:current_lane_final_merge_id],
										current_lane_traj[2][current_lane_start_merge_id:current_lane_final_merge_id],
										current_lane_traj[3][current_lane_start_merge_id:current_lane_final_merge_id]]

			# print "current_lane_merging_traj = " + str(current_lane_merging_traj)
		else:
			current_lane_merging_traj = [current_lane_traj[0][current_lane_start_merge_id:], current_lane_traj[0][:current_lane_final_merge_id],
										current_lane_traj[1][current_lane_start_merge_id:], current_lane_traj[1][:current_lane_final_merge_id],
										current_lane_traj[2][current_lane_start_merge_id:], current_lane_traj[2][:current_lane_final_merge_id],
										current_lane_traj[3][current_lane_start_merge_id:], current_lane_traj[3][:current_lane_final_merge_id]]
			# current_lane_merging_traj = [ current_lane_traj[:][current_lane_start_merge_id:], current_lane_traj[:][:current_lane_final_merge_id] ]

		merging_traj = []

		# print "current_lane_final_merge_id = " + str(current_lane_final_merge_id)
		# print "current_lane_start_merge_id = " + str(current_lane_start_merge_id)
		# print "current_lane_merging_traj = " + str(current_lane_merging_traj)

		print "num_points_next_traj_merge = " + str(num_points_next_traj_merge)
		print "num_points_current_traj_merge = " + str(num_points_current_traj_merge)
		print "num_points_next_traj_merge > num_points_current_traj_merge = " + str(num_points_next_traj_merge > num_points_current_traj_merge)

		if num_points_next_traj_merge > num_points_current_traj_merge:

			current_traj_merge_dist = [0]

			for temp_idx in xrange(1, len( current_lane_merging_traj[0] ) ):

				current_traj_merge_dist.append( current_traj_merge_dist[-1] + math.hypot( current_lane_merging_traj[0][temp_idx] - current_lane_merging_traj[0][temp_idx-1] ,
															current_lane_merging_traj[1][temp_idx] - current_lane_merging_traj[1][temp_idx-1] ) )

			next_traj_merge_dist = [0]

			for temp_idx in xrange(1, len( next_lane_merging_traj[0] ) ):

				next_traj_merge_dist.append( next_traj_merge_dist[-1] + math.hypot( next_lane_merging_traj[0][temp_idx] - next_lane_merging_traj[0][temp_idx-1] ,
															next_lane_merging_traj[1][temp_idx] - next_lane_merging_traj[1][temp_idx-1] ) )


			for temp_idx in xrange( len( next_lane_merging_traj[0] ) ):

				current_next_lane_point = [next_lane_merging_traj[0][temp_idx],
											next_lane_merging_traj[1][temp_idx],
											next_lane_merging_traj[2][temp_idx],
											next_lane_merging_traj[3][temp_idx]]
				# current_next_lane_point = next_lane_merging_traj[:][temp_idx]

			# print "next_traj_merge_dist = " + str(next_traj_merge_dist)

			normalized_next_traj_dist = [ next_traj_merge_dist_value/next_traj_merge_dist[-1] for next_traj_merge_dist_value in next_traj_merge_dist ]

			# print "normalized_next_traj_dist = " + str(normalized_next_traj_dist)

			normalized_current_traj_dist = [ current_traj_merge_dist_value/current_traj_merge_dist[-1] for current_traj_merge_dist_value in current_traj_merge_dist ]

			for temp_idx in xrange( len( next_lane_merging_traj[0] ) ):

				point_a = [ normalized_next_traj_dist[temp_idx]*next_lane_merging_traj[0][temp_idx], normalized_next_traj_dist[temp_idx]*next_lane_merging_traj[1][temp_idx],
							normalized_next_traj_dist[temp_idx]*next_lane_merging_traj[2][temp_idx], normalized_next_traj_dist[temp_idx]*next_lane_merging_traj[3][temp_idx] ]

				point_b = []

				if temp_idx == 0:

					point_b = [ current_lane_merging_traj[0][temp_idx], current_lane_merging_traj[1][temp_idx], current_lane_merging_traj[2][temp_idx], current_lane_merging_traj[3][temp_idx] ]

				else:

					if temp_idx == len( next_lane_merging_traj[0] ) - 1:

						point_b = [ 0, 0 , 0, 0]

					else:

						desired_dist = normalized_next_traj_dist[temp_idx]
						best_below_index = -1
						best_above_index = + 1000000

						for temp_j in xrange( len( normalized_current_traj_dist ) ):

							if normalized_current_traj_dist[temp_j] < desired_dist:

								best_below_index = temp_j

							if normalized_current_traj_dist[-temp_j-1] > desired_dist:

								best_above_index = temp_j
						
						# print "normalized_current_traj_dist = " + str(normalized_current_traj_dist)

						dist_to_below = desired_dist - normalized_current_traj_dist[best_below_index] 
						dist_to_above = normalized_current_traj_dist[best_above_index] - desired_dist

						# print "temp_idx = " + str(temp_idx)

						# print "dist_to_below = " + str(dist_to_below)
						# print "dist_to_above = " + str(dist_to_above)

						below_weight = dist_to_above/(dist_to_below + dist_to_above)
						above_weight = dist_to_below/(dist_to_below + dist_to_above)

						point_b = [ current_lane_merging_traj[0][best_below_index]*below_weight + current_lane_merging_traj[0][best_above_index]*above_weight , 
									current_lane_merging_traj[1][best_below_index]*below_weight + current_lane_merging_traj[1][best_above_index]*above_weight , 
									current_lane_merging_traj[2][best_below_index]*below_weight + current_lane_merging_traj[2][best_above_index]*above_weight , 
									current_lane_merging_traj[3][best_below_index]*below_weight + current_lane_merging_traj[3][best_above_index]*above_weight ]

						point_b = [ point_b[0]*(1.-normalized_next_traj_dist[temp_idx]), 
									point_b[1]*(1.-normalized_next_traj_dist[temp_idx]),
									point_b[2]*(1.-normalized_next_traj_dist[temp_idx]),
									point_b[3]*(1.-normalized_next_traj_dist[temp_idx]) ]

				merging_traj.append( [ point_a[0] + point_b[0] , point_a[1] + point_b[1] ,  point_a[2] + point_b[2] , point_a[3] + point_b[3] ] )

		else:

			print "NOT IMPLEMENTED YET"


		# merging_traj is the section of the trajectory in which the merge occurs
		# now we must append the remainder of the next_lane_traj

		next_lane_traj_to_append = []

		print "next_lane_final_merge_id = " + str(next_lane_final_merge_id)
		print "next_lane_start_merge_id = " + str(next_lane_start_merge_id)

		if next_lane_final_merge_id > next_lane_start_merge_id:

			x = []
			print "len(x) = " + str(len(x))
			x.extend(next_lane_traj[0][next_lane_final_merge_id:])
			print "len(x) = " + str(len(x))
			x.extend(next_lane_traj[0][:next_lane_start_merge_id])
			print "len(x) = " + str(len(x))

			y = []
			y.extend(next_lane_traj[1][next_lane_final_merge_id:])
			y.extend(next_lane_traj[1][:next_lane_start_merge_id])

			theta = []
			theta.extend(next_lane_traj[2][next_lane_final_merge_id:])
			theta.extend(next_lane_traj[2][:next_lane_start_merge_id])

			time = []
			time.extend(next_lane_traj[3][next_lane_final_merge_id:])
			time.extend(next_lane_traj[3][:next_lane_start_merge_id])
		
			next_lane_traj_to_append = [x, y, theta, time]
			# next_lane_traj_to_append = [ [next_lane_traj[0][next_lane_final_merge_id:], next_lane_traj[0][:next_lane_start_merge_id] ],
			# 							 [next_lane_traj[1][next_lane_final_merge_id:], next_lane_traj[1][:next_lane_start_merge_id] ],
			# 							 [next_lane_traj[2][next_lane_final_merge_id:], next_lane_traj[2][:next_lane_start_merge_id] ],
			# 							 [next_lane_traj[3][next_lane_final_merge_id:], next_lane_traj[3][:next_lane_start_merge_id] ] ]

		else:

			next_lane_traj_to_append = [ next_lane_traj[0][next_lane_final_merge_id:next_lane_start_merge_id], 
										next_lane_traj[1][next_lane_final_merge_id:next_lane_start_merge_id],
										next_lane_traj[2][next_lane_final_merge_id:next_lane_start_merge_id],
										next_lane_traj[3][next_lane_final_merge_id:next_lane_start_merge_id] ]





		print "len( next_lane_traj_to_append ) = " + str(len( next_lane_traj_to_append ))
		print "len( next_lane_traj_to_append[0] ) = " + str(len( next_lane_traj_to_append[0] ))

		print "merging_traj = " + str(merging_traj)

		transposed_merging_traj = []

		for i in len( merging_traj[0] ):

			temp_list = []

			for j in len( merging_traj ):

				temp_list.extend( transposed_merging_traj[j][i] )

			print "len(temp_list) = " + str(len(temp_list))

			transposed_merging_traj.append(temp_list)


		new_traj = [merging_traj, next_lane_traj_to_append]

		print "len( merging_traj ) = " + str(len( merging_traj ))
		print "len( merging_traj[0] ) = " + str(len( merging_traj[0] ))
		print "len( next_lane_traj_to_append ) = " + str(len( next_lane_traj_to_append ))
		print "len( next_lane_traj_to_append[0] ) = " + str(len( next_lane_traj_to_append[0] ))

		self.traj = new_traj

		# num_points_current_traj_merge = current_lane_final_merge_id - current_lane_start_merge_id

	def check_lane_change_completion(self):

		pass


	def lame_lane_change(self):

		current_passed_time = time.time() - self.debug_lane_changes_time

		current_passed_time = current_passed_time%40

		# print "current_passed_time = " + str(current_passed_time)

		# Lame lane change
		if current_passed_time > 30:
			if self.current_lane != self.CENTER_LANE_STRING:
				print "Changing to center lane"
				self.traj = self.center_lane_traj
				self.current_lane = self.CENTER_LANE_STRING
		else:
			if current_passed_time > 20:
				if self.current_lane != self.LEFT_LANE_STRING:
					print "Changing to left lane"
					self.traj = self.left_lane_traj
					self.current_lane = self.LEFT_LANE_STRING
				
			else:
				if current_passed_time > 10:
					if self.current_lane != self.CENTER_LANE_STRING:
						print "Changing to center lane"
						self.traj = self.center_lane_traj
						self.current_lane = self.CENTER_LANE_STRING
				else:
					if self.current_lane != self.RIGHT_LANE_STRING:
						print "Changing to right lane"
						self.traj = self.right_lane_traj
						self.current_lane = self.RIGHT_LANE_STRING


	
	def get_low_level_inputs(self, current_time, body_readings):
		# This is the one being used now!

	
		closeby_body_readings = []

		for temp_body_reading in body_readings['readings']:

			temp_closeby_body_reading = [ temp_body_reading[0]*32. , temp_body_reading[1]*32. , math.radians( temp_body_reading[2] ) ]

			distance_to_body = math.hypot( self.state[0] - temp_closeby_body_reading[0] , self.state[1] - temp_closeby_body_reading[1] )

			# print "distance_to_body = " + str(distance_to_body)

			if distance_to_body < 30.0:

				closeby_body_readings.append(temp_closeby_body_reading)

		#Tips
		# self.state = [real_world_meters, radians]
		# print "self.state = " + str(self.state)
		# closeby_body_readings = [real_world_meters, radians]
		# print "closeby_body_readings = " + str(closeby_body_readings)
		# self.traj[0] in real_world_meters
		# self.traj[2] in radians
		# print "self.traj[2] = " + str(self.traj[2])




		# print "closeby_body_readings = " + str(closeby_body_readings)
		# print "len( closeby_body_readings ) = " + str(len( closeby_body_readings ))

		desired_velocity = self.velocity / 3.6

		current_idx = self.current_trajectory_id

		number_points_ahead = 25

		# print "self.state = " + str(self.state)
		# self.state = [-39.44159273512578, -38.36738366701975, -2.3967515204479533]
		# self.state comes in real world metres and radians

		[best_distance, best_idx] = self.find_closest_trajectory_point( self.state, current_idx, number_points_ahead)

		current_closest_trajectory_point = best_idx

		self.lame_lane_change()

		traj_len = len( self.traj[0] )

		best_idx += 5

		reference_state = [self.traj[0][ (current_idx+best_idx)%traj_len ] , self.traj[1][ (current_idx+best_idx)%traj_len ] , self.traj[2][ (current_idx+best_idx)%traj_len ] ]

		# print "get_trajectory_tracking_inputs reference_state = " + str(reference_state)

		# converted_state = [self.state[0]/32., self.state[1]/32., self.state[2]]
		[velocity_command, steering_command] = self.controller_action(self.state, reference_state)
		# Will ignore velocity_command
		# steering_command = math.degrees(steering_command)

		safety_distance = 30.
		
		# Will check distance to cars in front
		# distance_to_car = self.distance_to_car_in_future_trajectory(current_time, closeby_body_readings, safety_distance)
		distance_to_car = self.new_distance_to_car_in_future_trajectory(current_closest_trajectory_point, closeby_body_readings, safety_distance)

		# distance_to_car = self.is_colliding_soon(current_time, body_readings)

		print "distance_to_car = " + str(distance_to_car) + " len( closeby_body_readings ) = " + str(len( closeby_body_readings ))

		adjusted_velocity = []


		# if distance_to_car == 100. or distance_to_car > safety_distance:

		# 	adjusted_velocity = desired_velocity

		# else:

		# 	adjusted_velocity = desired_velocity*(distance_to_car/safety_distance)

			# print "distance_to_car = " + str(distance_to_car)
			# print "safety_distance = " + str(safety_distance)
			# print "distance_to_car/safety_distance = " + str(distance_to_car/safety_distance)

			# print "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"

			# adjusted_velocity = 0.
		adjusted_velocity = velocity_command
		# if distance_to_car:

		# 	adjusted_velocity = 0.0

		# else:

		# 	adjusted_velocity = desired_velocity
			

		# self.input[0] = adjusted_velocity
		# self.input[1] = steering_command

		# print "return [adjusted_velocity, steering_command] = " + str([adjusted_velocity, steering_command])
		# print "Controller with throttle " + str( adjusted_velocity ) + \
		# 		" and steering " + str( steering_command ) + " at " + "(" + str(self.state[0]/32.) + \
		# 			"," + str(self.state[1]/32.) + "," + str( math.degrees( self.state[2]) ) + ") = (" + str(self.state[0]) + "," + str(self.state[1]) + ") "

		# Steering command should be outputteed in DEGREES
		return [adjusted_velocity, steering_command]

	