import time, math, threading

import bodyclasses, smartvehicle, dummyvehicle

class SimulatorModule:
	'''
	This class simulates the vehicles and their kinematics/dynamics
	if the SML World. It also simulates their sensor readings.
	'''

	def __init__(self, sml_world, simulation_rate = 50.):

		# The desired rate of the Simulation Module
		self.simulation_rate = simulation_rate

		self.sml_world = sml_world
		self.bodies_dict = sml_world.bodies_dict
				
		# Launches the tread that will run the main
		# loop of the SimulatorModule
		t = threading.Thread( target = self.thread_loop, args=([]) )
		t.daemon = True
		t.start()

	def thread_loop(self):
		'''
		This function is executed in a thread, thus running 
		in parallel with the rest of the SML World.
		It consists of a loop, where the simulation of the vehicles
		movements and the simulation of the sensor readings are 
		executed.
		'''

		# Time keepers
		current_time = time.time()
		previous_time = time.time()

		while not self.sml_world.close:

			current_time = time.time()

			# time_to_simulate is the ammount of time passed (in seconds)
			# since the last simulation step
			time_to_simulate = current_time - previous_time

			# Simulates the vehicles during time_to_simulate seconds
			simulated_bodies = self.simulate_vehicles(time_to_simulate)

			# Simulates the sensor readings of every vehicle, according
			# to its current state and the state of it's vicinity
			self.simulate_sensor_readings()

			previous_time = time.time()

			time_elapsed = previous_time - current_time

			time_to_sleep = 1./self.simulation_rate - time_elapsed

			if time_to_sleep > 0:

				time.sleep(time_to_sleep)

			else:

				print "SimulatorModule failed desired rate."


		self.sml_world.vehicle_simulation_connected = False

		return

	def simulate_vehicles(self, time_to_simulate):
		'''
		Simulates the vehicles over the period of time_to_simulate seconds.
		The simulation is based on the current state and inputs of each 
		vehicle, and will update the vehicle's state to the resulting
		simulated state
		'''

		# Notes:
		# the vehicle state coordinates for position come in meters 
		# (meters from the real world, not the SML World)
		# the vehicle state orientatio for the yaw comes in degrees

		

		# Since dictionary might change while I'm going through it, I have to access it "indirectly"
		# First I get a copy of the current dictionary keys
		bodies_dict_keys = self.bodies_dict.keys()

		# I will go over each of these keys
		for body_id in bodies_dict_keys:

			# If this key still exists in the dictionary then I'll work on the element
			if body_id in self.bodies_dict:

				current_body = self.bodies_dict[body_id]

				if current_body.simulated:
				# The body needs to be simulated

					current_body.vehicle_state_update(time_to_simulate)


		return

	


	def simulate_sensor_readings(self):
		'''
		Simulates the sensor readings of the vehicles.
		The sensor readings are simulated based on the current
		state of the vehicle and on the current state of the SML World.
		'''

		try:

			current_body_ids = self.bodies_dict.keys()
			# To avoid dictionary size change error

			for ego_body_id in current_body_ids:

				if ego_body_id in self.bodies_dict:
				# To avoid dictionary size change error

					ego_body = self.bodies_dict[ego_body_id]

					if not ( isinstance(ego_body, smartvehicle.SmartVehicle) or isinstance(ego_body, dummyvehicle.DummyVehicle) ):

						continue

				else:

					continue

				current_body_readings = []

				for other_body_id in current_body_ids:

					if ego_body_id == other_body_id:

						continue

					if other_body_id in self.bodies_dict:
					# To avoid dictionary size change error

						other_body = self.bodies_dict[other_body_id]

					else:

						continue

					other_body_reading = self.get_sensor_reading_as_dict(other_body.id, other_body.x, other_body.y, other_body.yaw, other_body.x_speed, other_body.y_speed)
					
					# If in omniscient mode we simply add every body in the SML World
					# as a sensor reading
					if ego_body.omniscient_sensor:

						current_body_readings.append( other_body_reading )

						continue

					# If there is a velodyne sensor, only cars in a vicinity of 
					# ego_body.velodyne_range meters are added as sensor readings
					if ego_body.velodyne_sensor:

						# Check if the other vehicle is closer than ego_body.velodyne_range meters
						if math.hypot( ego_body.x - other_body.x , ego_body.y - other_body.y ) < ego_body.velodyne_range:

							current_body_readings.append( other_body_reading )

							continue

					# If there is a radar sensor, only cars within a distance of 
					# ego_body.radar_range meters and inside a front facing cone
					# shaped are with angle of ego_body.radar_angle, are added 
					# as sensor readings
					if ego_body.radar_sensor:

						# Check if the other vehicle is closer than ego_body.radar_range meters
						if math.hypot( ego_body.x - other_body.x , ego_body.y - other_body.y ) < ego_body.radar_range:

							angle =  math.degrees(math.atan2( other_body.y - ego_body.y , other_body.x - ego_body.x ))

							# Check if the other vehicle is inside the cone shape
							if abs(angle - ego_body.yaw) < ego_body.radar_angle/2.:
								
								current_body_readings.append( other_body_reading )

				ego_body.set_sensor_readings(current_body_readings)

		except:

			print "simulatorclass.py simulate_sensor_readings() error"
			print "Tip: Maybe the dictionary of bodies_dict changed during the loop"
			raise

	def get_sensor_reading_as_dict(self, vehicle_id, vehicle_x, vehicle_y, vehicle_yaw, vehicle_x_speed, vehicle_y_speed):
		'''
		Creates a sensor reading as a dictionary based on the
		sensed vehicle id and states.
		'''

		body_reading_dict = dict()

		body_reading_dict['id'] = vehicle_id
		body_reading_dict['x'] = vehicle_x
		body_reading_dict['y'] = vehicle_y
		body_reading_dict['yaw'] = vehicle_yaw
		body_reading_dict['x_speed'] = vehicle_x_speed
		body_reading_dict['y_speed'] = vehicle_y_speed

		return body_reading_dict

	