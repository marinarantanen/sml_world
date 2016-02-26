import math
import smartvehicle
import numpy



class VehiclePerceptionModule:
	"This class is the Perception Module, that is running in a child thread of "
	"the Smart Vehicle module. The Perception Module should be in charge of getting"
	"the current sensor readings and V2V messages, process them and generate useful"
	"information to be sent to the Supervisory Module and/or Control Module"

	# just guesses right now

	def __init__(self, vehicle, intersection_vehicle = False):

		self.module_ready = False
		self.vehicle = vehicle
		self.current_index = None

		self.intersection_vehicle = intersection_vehicle


		# Different classifications for classifyRoadCoords, is then transferred to the perception_grid in vehicle
		self.fl = []
		self.fm = []
		self.fr = []
		self.l = []
		self.r = []
		self.rl = []
		self.rm = []
		self.rr = []

		self.road_x = 0
		self.road_y = 0

		self.center_lane_traj = self.vehicle.center_lane_traj

		self.mean_dists = {}
		self.tot_dists = {}

		self.generateMeanDistances()


		self.new_perceived_objects = {}
		self.closest_objects = {}

		print "VehiclePerceptionModule started"

	def generateMeanDistances(self):
		# Just calculates the mean distance between two consecutive points in a trajectory and the total length of a trajectory from that

		traj_list = ['left_lane', 'center_lane', 'right_lane']
		for traj_string in traj_list:
			tot_distance = 0.
			traj = self.vehicle.getLaneTrajectory(traj_string)

			# print "first "+ str((traj[0][0], traj[1][0]))
			for i in range(len(traj[0]) - 1):
				dist = math.hypot(traj[0][i+1] - traj[0][i], traj[1][i+1] - traj[1][i])
				tot_distance += dist

			# print "last: " + str((traj[0][len(traj[0]) - 1], traj[1][len(traj[0]) - 1]))

			self.tot_dists[traj_string] = tot_distance
			self.mean_dists[traj_string] = tot_distance/(len(traj[0]) - 1)


	def step(self):
		# This is the step function that is running at each cycle of the thread
		# in SmartVehicle

		self.fl = []
		self.fm = []
		self.fr = []
		self.l = []
		self.r = []
		self.rl = []
		self.rm = []
		self.rr = []


		try:
			body_data = self.vehicle.bodies_dict[self.vehicle.id]
		except:
			body_data = None
			pass

		if bool(body_data):
			# check for initialization

			self.x = body_data.x
			self.y = body_data.y
			coord_vector = [self.x, self.y]
			self.theta = math.radians(body_data.yaw)
			self.current_lane_string = self.vehicle.getCurrentLaneTrajectoryString()

			# get the road relative coordinates
			self.updateRoadCoord(coord_vector, self.current_lane_string)
			if abs(self.road_y) < self.vehicle.lanekeeping_margin:
				self.vehicle.lanekeeping_ready.set()
			else:
				self.vehicle.lanekeeping_ready.clear()

			for reading in body_data.sensor_readings:
				# classifying every reading
				other_coords = [reading['x'], reading['y']]

				rw_coords = [other_coords[0], other_coords[1]]

				other_body_id = reading['id']

				# converts into road relative coordinates
				road_coords = self.convertToRoadCoord(rw_coords, self.current_lane_string)

				# relative to vehicle, takes into account the periodiciy of the oval
				relative_road_coords = self.getRelativeRoadCoordsOval(road_coords, self.current_lane_string)

				# calculating velocity of other vehicle
				x_speed = reading['x_speed']
				y_speed = reading['y_speed']
				other_velocity = math.hypot(x_speed, y_speed)

				# self.classify(rw_coords);
				self.classifyRoadCoords(other_body_id, relative_road_coords, other_velocity)

			# Just sets the data structures in smartvehicle aswell
			self.setPerceivedObjects()
			self.setPerceptionGrid()
			self.findClosestObjects()

			'''
			Show case of how to get longitudinal distances
			to perceived vehicles.

			# if self.vehicle.id == -2:
			# 	target_id = -1
			# 	if target_id in self.vehicle.perceived_objects:
			# 		print self.getLongitudinalDistanceToTarget(target_id)

			'''
			self.module_ready = True

		if self.intersection_vehicle and self.vehicle.id == -2:
			self.vehicle.control_module.VACCC = True
			vehicle_target_id = -1
			self.vehicle.control_module.VACCC_distance = self.compute_virtual_distance_to_vehicle(vehicle_target_id)
			self.vehicle.control_module.VACCC_desired_distance = 10
			self.VACCC_target_velocity = None # Should change
		'''
		Show case of how to use the VACCC
		First we compute our virtual distance
		to the target vehicle.
		And then we tell the Control module, to turn
		on the VACCC mode.
		We also need to provide the VACCC desired distance
		and current distance.
		In the future we can also send the velocity of
		the target for improved controller performance
		'''

		if self.intersection_vehicle and self.vehicle.id == -2:

			self.vehicle.control_module.VACCC = True
			vehicle_target_id = -1
			self.vehicle.control_module.VACCC_distance = self.compute_virtual_distance_to_vehicle(vehicle_target_id)
			self.vehicle.control_module.VACCC_desired_distance = 10
			self.VACCC_target_velocity = None # Should change
		
		





	def compute_virtual_distance_to_vehicle(self, target_id):
		'''
		Functionality specific to the Intersection Scenario
		Given a vehicle id, as target_id, we will poll
		it's perceived distance to it's anchor.
		We then poll our own perceived distance to our anchor.
		Given that the anchors of both vehicles are the same
		we will have a faithful estimation of the VACCC distance.
		NOTE: It is the responsibility of the method caller
		to provide a target_id that makes sense.
		This method will not check if the anchors of both
		vehicles are the same, instead it will assume so.
		'''

		# Poll the distances to the anchors
		other_vehicle_distance_to_anchor = self.vehicle.bodies_dict[target_id].control_module.distance_to_anchor
		current_distance_to_anchor = self.vehicle.control_module.distance_to_anchor

		# In case the control module has not been initialized yet.
		if current_distance_to_anchor == None:

			return 0

		# The virtual distance is simply the difference of the distances
		# to the anchors
		distance_to_vehicle = current_distance_to_anchor - other_vehicle_distance_to_anchor

		return distance_to_vehicle


	def getRelativeRoadCoordsOval(self, road_coords, trajectory_string):

		# This takes road coordinates and gets the relative coordinate from the current vehicle to this
		# Also takes into account the periodicity of the trajectories

		if road_coords[0] > self.road_x:
			x_alt1 = road_coords[0] - self.road_x
			x_alt2 = road_coords[0] - self.tot_dists[trajectory_string]- self.road_x
			relative_road_x = x_alt1 if abs(x_alt1) < abs(x_alt2) else x_alt2
		else:
			x_alt1 = self.road_x - road_coords[0]
			x_alt2 = self.road_x - self.tot_dists[trajectory_string]- road_coords[0]
			relative_road_x = -x_alt1 if abs(x_alt1) < abs(x_alt2) else -x_alt2

		return [relative_road_x, road_coords[1] - self.road_y]

	def getLongitudinalDistanceToTarget(self, target_id):
		'''
		Specific to the Highway Scenario (at the present moment)
		Author: Rui
		Gets the longitudinal distance along my trajectory to
		another perceived vehicle with identifier target_id
		'''
		return self.vehicle.perceived_objects[target_id][0]

	def classifyRoadCoords(self, other_body_id, relative_coords, other_velocity):
		# Takes relative road coords and classifies them into 8 classes:
		# fwd left, fwd middle, fwd right, left, right, rear left, rear middle, rear right
		# Appends the id to the lists specified in the beginning of __init__()

		self.addPerceivedBody(other_body_id, relative_coords, other_velocity)

		if relative_coords[0] > 4*smartvehicle.SmartVehicle.length/5:
			# If object is in front of vehicle

			if relative_coords[1] > smartvehicle.SmartVehicle.width:
				# If it is to left

				self.fl.append(other_body_id)

			elif relative_coords[1] < -smartvehicle.SmartVehicle.width:
				# to the right

				self.fr.append(other_body_id)

			else:
				# middle
				self.fm.append(other_body_id)

		elif relative_coords[0] < -smartvehicle.SmartVehicle.length/5:
			# to the rear

			if relative_coords[1] > smartvehicle.SmartVehicle.width:
				# left

				self.rl.append(other_body_id)

			elif relative_coords[1] < -smartvehicle.SmartVehicle.width:
				#right

				self.rr.append(other_body_id)

			else:
				# middle
				self.rm.append(other_body_id)

		else:
			# to the sides
			if relative_coords[1] > smartvehicle.SmartVehicle.width:
				# left

				self.l.append(other_body_id)

			elif relative_coords[1] < -smartvehicle.SmartVehicle.width:
				#right

				self.r.append(other_body_id)

	def addPerceivedBody(self, other_body_id, relative_road_coords, other_velocity):
		# Adds a new body to the new_perceived_objects dict
		other_info = relative_road_coords + [other_velocity]
		self.new_perceived_objects[other_body_id] = other_info

	def setPerceivedObjects(self):
		# Transfers the infor in new_perceived_objects to smartvehicle
		self.vehicle.perceived_objects = self.new_perceived_objects
		self.new_perceived_objects = {}

	def getPerceivedObject(self, identity):
		# Gets perceived object from dict

		if identity in self.vehicle.perceived_objects:
			return self.vehicle.perceived_objects[identity]

		else:
			return None

	def findClosestObjects(self):
		# Calculates the closest object for each of the classes specified in classify road coords
		# These can then be accessed by e.g. self.vehicle.closest_objects['fl'] for front left

		new_closest_objects = {}
		zero_vector = [0, 0]

		directions = ['fm', 'l', 'r', 'rm', 'fl', 'fr', 'rl', 'rr']
		for direction in directions:
			if self.vehicle.perception_grid[direction]:
				closest_id = self.vehicle.perception_grid[direction][0]
				closest_distance = VehiclePerceptionModule.distance(zero_vector, self.getPerceivedObject(closest_id))
				index = 1
				while index < len(self.vehicle.perception_grid[direction]):
					new_body_id = self.vehicle.perception_grid[direction][index]
					new_distance = VehiclePerceptionModule.distance(zero_vector, self.getPerceivedObject(new_body_id))

					if new_distance < closest_distance:
						closest_id = new_body_id
						closest_distance = new_distance

					index += 1

				new_closest_objects[direction] = closest_id

		self.vehicle.closest_objects = new_closest_objects

	def getClosestId(self, direction):
		# Returns closest id from the specified direction (see classify objects and findClosestObjects

		if self.vehicle.closest_objects[direction]:
			return self.vehicle.closest_objects[direction]
		else:
			return None


	def setPerceptionGrid(self):
		"""Sets the perception grid of the vehicle object
		:returns: TODO

		"""

		perception_grid = dict()

		perception_grid['fl'] = self.fl
		perception_grid['fm'] = self.fm
		perception_grid['fr'] = self.fr
		perception_grid['l'] = self.l
		perception_grid['r'] = self.r
		perception_grid['rl'] = self.rl
		perception_grid['rm'] = self.rm
		perception_grid['rr'] = self.rr


		self.vehicle.perception_grid = perception_grid

	def convertToRoadCoord(self, coords, lane_traj_string):
		"""
		Takes absolute coordinates and transforms them into (approximate) road coords
		road coords is if you think of the road as a straight line
		x indicates the longitudinal coordinates
		y indicates the transversal (with y > 0 is left of the center lane)

		"""
		np_lane_traj = self.vehicle.getNumpyLaneTrajectory(lane_traj_string)

		np_state = numpy.array([[coords[0]],[coords[1]]])

		temp_distance = numpy.sum((np_lane_traj[0:2, :] - np_state)**2, axis = 0)

		closest_index = numpy.argmin(temp_distance)

		x = closest_index*self.mean_dists[lane_traj_string]
		y = self.getRoadY(closest_index, lane_traj_string, coords)
		return (x, y, closest_index)


	def updateRoadCoord(self, coordinates, lane_traj_string):
		""" an attempt to make convertToRoadCoord more effective when computing for own vehicle
			Does the same thing as convertToRoadCoordOld but just searches through the first 100 consecutive points of current closest point

		"""
		# Needs the lane traj
		lane_traj = self.vehicle.getLaneTrajectory(lane_traj_string)

		# Just checks for initialization
		if self.current_index == None or lane_traj_string != self.old_lane_string:
			self.current_index = self.convertToRoadCoord(coordinates, lane_traj_string)[2]

		else:
			counter = 0
			v1 = [lane_traj[0][self.current_index], lane_traj[1][self.current_index]]
			v2 = coordinates
			old_d = VehiclePerceptionModule.distance( v1, v2 )
			counter += 1
			closest_counter = 0

			# Change 100 to other to change number of points ahead to search
			while counter < 100:
				new_index = (self.current_index + counter)%len(lane_traj[0])
				v1 = [lane_traj[0][(self.current_index+counter)%len(lane_traj[0])], lane_traj[1][(self.current_index + counter)%len(lane_traj[0])]]
				new_d = VehiclePerceptionModule.distance(v1, v2)

				if new_d < old_d:
					old_d = new_d
					closest_counter = counter
				else:
					break

				counter += 1

			self.current_index = (self.current_index + closest_counter)%len(lane_traj[0])
		self.road_x = self.current_index*self.mean_dists[lane_traj_string]
		self.road_y = self.getRoadY(self.current_index, lane_traj_string, coordinates)

		self.old_lane_string = lane_traj_string


	def getRoadY(self, index, lane_traj_string, coordinates):
		""" get the road relative y coordinates based on the selected lane, a positive value means left of lane trajectory """

		# Needs the trajectory
		lane_traj = self.vehicle.getLaneTrajectory(lane_traj_string)

		# Checks to see that index is not the last index of the trajectory list
		if not index == len(lane_traj[0]) - 1:
			# Calculates the tangent by taking two consecutive points
			tangent = [lane_traj[0][index+1] - lane_traj[0][index], lane_traj[1][index + 1] - lane_traj[1][index]]
		else:
			# Does same thing but for simplicity takes the previous index in the trajectory as a reference
			tangent = [lane_traj[0][index] - lane_traj[0][index-1], lane_traj[1][index] - lane_traj[1][index-1]]

		# Normalizing the tangent
		tangent = [tangent[0]/math.hypot(tangent[0], tangent[1]), tangent[1]/math.hypot(tangent[0], tangent[1])]
		# Calculating the orthogonal vector to the tangent
		normal = [-tangent[1], tangent[0]]

		# Relative coords being calculated
		relCoords = [coordinates[0] - lane_traj[0][index], coordinates[1] - lane_traj[1][index]]

		# Takes the scalar product
		road_y = relCoords[0]*normal[0] + relCoords[1]*normal[1]

		return road_y


	@staticmethod
	def distance(v1, v2):
		""" calculates the distance between two different objects

		:v1: vector 1 [x, y]
		:v2: vector 2 [x, y]
		:returns: euclidean distance

		"""
		return math.hypot(v1[0] - v2[0], v1[1] - v2[1])
