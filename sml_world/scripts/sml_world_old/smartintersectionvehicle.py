import socket, sys, time, math, random, threading

import vehiclesupervisorymodule_old
import vehiclecontrolmodule
import vehicleperceptionmodule
import bodyclasses
import smartvehicle
import vehiclesupervisorymodule

import numpy


# TODO
# add more to handle message function

class SmartIntersectionVehicle(smartvehicle.SmartVehicle):
	"This class is supposed to encompass three other classes which are running in threads"
	"The three other classes are the SupervisoryModule, the PerceptionModule and the "
	"ControlModule"
	"This class manages all the three Modules at the same time, making sure they are able to"
	"communicate with each other"
	
	def __init__(self, sml_world, vehicle_id, start_section, end_section, anchor, lane_start_point_index = 0, real_vehicle = False, emergency_vehicle = False, intersection_vehicle = True):

		self.start_section = start_section
		self.end_section = end_section
		self.anchor = anchor

		# Call parent class constructor
		smartvehicle.SmartVehicle.__init__(self, sml_world, vehicle_id, lane_start_point_index , real_vehicle , emergency_vehicle, intersection_vehicle)

		self.cycle_traj = False

		self.set_vehicle_on_trajectory_state(meters = 40)
	

	'''
	override of SmartVehicle method
	'''
	def getLaneTrajectorys(self, road_module):
		"""
		sets the lane trajectorys of the vehicle

		:osm_info: some kind of info
		:returns: boolean True if it was successful, has never happened that it failed without crashing though

		"""

		start_node_ids_list = road_module.osm_node_tag_dict[self.start_section]
		start_node_id = start_node_ids_list[0]
		end_node_ids_list = road_module.osm_node_tag_dict[self.end_section]
		end_node_id = end_node_ids_list[0]

		anchor_node_id = road_module.osm_node_tag_dict[self.anchor][0]
		self.anchor_node = road_module.osm_node_dict[anchor_node_id]

		[lane_traj_x, lane_traj_y] = road_module.get_path_between_node_ids(start_node_id, end_node_id, points_per_meter = 5.0)

		points_per_meter = 5.

		right_lane_traj_x = lane_traj_x
		right_lane_traj_y = lane_traj_y
		center_lane_traj_x = lane_traj_x
		center_lane_traj_y = lane_traj_y
		left_lane_traj_x = lane_traj_x
		left_lane_traj_y = lane_traj_y

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

	'''
	Another override
	'''
	def set_vehicle_on_trajectory_state(self, index = None, meters = None):
		# This function is used to place the vehicle in the initial position
		# of the trajectory

		if index == None:

			if meters != None:

				if meters < 0:

					raise NameError("set_vehicle_on_trajectory_state receiving a negative value of meters to offset")

				index = int( meters*5 )

			else:

				index = 0

		index = index % len(self.traj[0])
		self.control_module.current_trajectory_id = index


		if len( self.traj ) == 0:

			raise NameError('In class SmartVehicle: trying to use set_vehicle_on_trajectory_state when trajectory is not yet defined')

		# print "index = " + str(index)

		#index = 50

		index = index%len(self.traj[0])

		self.x = self.traj[0][index]
		self.y = self.traj[1][index]
		self.yaw = math.degrees(self.traj[2][index])

		if self.emergency_vehicle:
			self.x = 33.
			self.y = -37.5
			self.yaw = 0.
		self.z = 0.0
		self.roll = 0.0
