#!/usr/bin/env python
import rospy
import threading
import sys
# import os
import os.path
import time
# import math

sys.path.append('RoadModule')
import RoadModule

import SimulatorModule
import V2VModule
import UnityModule

import smartvehicle
import smartintersectionvehicle

import dummyvehicle

import VisualisationModule, VisualisationModuleSender

import multiprocessing

import MotionCaptureModule

import Configuration 
import hmi

from Constants import *

# noinspection PyBroadException,PyArgumentList
class SMLWorld(object):
	"""SMLWorld project"""

	def __init__(self):
		super(SMLWorld, self).__init__()

		self.close = False

		self.qualisys_info = None

		self.close_sml_world = False

		# This is the dictionary which will have all of the bodies (real/simulated, smart/dummy, bus/persons, etc...) in the World
		self.bodies_dict = dict()

		self.hmi = hmi.HMI(self)

		self.scenario = 1

	def start_thread(self, handler, args=()):
		t = threading.Thread(target=handler, args=args)
		t.daemon = True
		t.start()

	def highway_map_vehicle_initializer(self, num_cars):
		'''
		This function will create and place the
		smart vehicles in the highway map.
		'''

		# The desired number of vehicles to place
		num_smart_vehicles = num_cars

		# The id of the vehicles, starting at -1, and
		# decreasing for each vehicle placed: -2, -3, ...
		vehicle_id = -1

		# The first car will be placed in the starting (zero index)
		# trajectory point
		current_traj_idx = 0
		# Each vehicle will be placed delta_traj trajectory points apart
		delta_traj = 30

		for i in range(num_smart_vehicles-1):

			# Creating a vehicle with id vehicle_id
			smart_vehicle = smartvehicle.SmartVehicle(self, vehicle_id,
				lane_start_point_index = 0,init_velocity=10.0, init_lane=Lane.CENTER)

			# Adding the vehicle to the bodies array
			self.bodies_dict[vehicle_id] = smart_vehicle

			# Placing the vehicle in the trajectory point with id current_traj_idx
			smart_vehicle.set_vehicle_on_trajectory_state(current_traj_idx)

			# The next vehicle to place will be delta_traj trajectory points away
			current_traj_idx += delta_traj

			# For each vehicle placed, the vehicle id has to decrease a unit
			vehicle_id -= 1

		current_traj_idx = 0
		for i in range(num_smart_vehicles):

			# Creating a vehicle with id vehicle_id
			smart_vehicle = smartvehicle.SmartVehicle(self, vehicle_id,
				lane_start_point_index = 0,init_velocity=10.0, init_lane=Lane.RIGHT)

			# Adding the vehicle to the bodies array
			self.bodies_dict[vehicle_id] = smart_vehicle

			# Placing the vehicle in the trajectory point with id current_traj_idx
			smart_vehicle.set_vehicle_on_trajectory_state(current_traj_idx)

			# The next vehicle to place will be delta_traj trajectory points away
			current_traj_idx += delta_traj

			# For each vehicle placed, the vehicle id has to decrease a unit
			vehicle_id -= 1

		return

	def intersection_map_vehicle_initializer(self):
		'''
		This function will create and place the
		smart vehicles in the intersection map.
		'''

		# The desired number of vehicles to place
		num_smart_vehicles = 2

		# The id of the vehicles, starting at -1, and
		# decreasing for each vehicle placed: -2, -3, ...
		vehicle_id = -1

		# The first car will be placed in the starting (zero index)
		# trajectory point
		start_index = 0

		possible_starts = ['top_right_start', 'top_left_start', 'bottom_start']
		possible_ends = ['top_right_end', 'top_left_end', 'bottom_end']
		possible_anchors = ['top_right_anchor', 'top_left_anchor', 'bottom_anchor']

		for i in range(num_smart_vehicles):

			# Creates the vehicle with the id vehicle_id
			# start_section defines where the vehicle is going to start
			# and it is taken from on of the options in possible_starts
			# end_section defines where the vehicle destination and it
			# is taken from on of the options in possible_ends
			ending_index = 0

			if i == 0:

				smart_vehicle = smartintersectionvehicle.SmartIntersectionVehicle(self, vehicle_id,
					start_section = possible_starts[2], end_section = possible_ends[ending_index],
					anchor = possible_anchors[ending_index])

				start_index = 0

			else:

				smart_vehicle = smartintersectionvehicle.SmartIntersectionVehicle(self, vehicle_id,
					start_section = possible_starts[1], end_section = possible_ends[ending_index],
					anchor = possible_anchors[ending_index])

				start_index = 50

			self.bodies_dict[vehicle_id] = smart_vehicle

			# Placing the vehicle in the trajectory point with id start_index
			smart_vehicle.set_vehicle_on_trajectory_state(start_index)

			start_index += 100

			# For each vehicle placed, the vehicle id has to decrease a unit
			vehicle_id -= 1

		return

	def close_world(self):
		self.close_sml_world = True

	def start_sml_world(self):

		'''
		Starting the SML Word
		'''

		#self.scenario = scenario
		
		# we read from here!

		# If true loads the intersection map, otherwise
		# it loads the highway map
		#if scenario==2:
	#		self.intersection_map = True
	#	else:
	#		self.intersection_map = False
		
		self.use_real_trucks = False

		file_location = "RoadModule/HighwaySML"

		# Create the road Module
		self.road_module = RoadModule.RoadModule(file_location, image_pixel_per_meter=5.)

		'''
		# node_1_id = self.road_module.osm_node_tag_dict['top_left_end']
		# node_2_id = self.road_module.osm_node_tag_dict['top_left_start']

		# node_1 = self.road_module.osm_node_dict[node_1_id[0]]
		# print "node_1 = " + str(node_1)
		# print "node_1.x = " + str(node_1.x)
		# print "node_1.y = " + str(node_1.y)
		# node_2 = self.road_module.osm_node_dict[node_2_id[0]]
		# print "node_2.x = " + str(node_2.x)
		# print "node_2.y = " + str(node_2.y)
		
		# print "math.hypot(node_2.x - node_1.x, node_2.y - node_1.y) = " + str(math.hypot(node_2.x - node_1.x, node_2.y - node_1.y))
		'''


		# Place the cars on the world
		#if self.intersection_map:

			# Use the intersection map vehicle placer
		#	self.intersection_map_vehicle_initializer()

		#else:

			# Use the highway map vehicle placer
		#	self.highway_map_vehicle_initializer(num_cars)

		# Create the v2vModule
		self.v2v_module = V2VModule.V2VModule(self, v2v_wifi_range = 50.)

		# Create the SimulationModule
		self.simulator_module = SimulatorModule.SimulatorModule(self, simulation_rate = 50.)

		'''
		To start the Visualisation
		'''

		# Starting the VisualisationSenderModule
		self.visualisation_module = VisualisationModuleSender.VisualisationModuleSender(self, visualisation_address = '127.0.0.1')


		# display_width = 1400
		# display_height = 1050
		display_width = 800
		display_height = 600
		display_pixel_per_meter = 5

		# Set this Flag to True, otherwise it might cause bugs
		# Issue #12 Major Bug
		ground_projection = True
		
		args = (file_location, display_width, display_height, display_pixel_per_meter, ground_projection)
		print args

		self.visualisation_thread = multiprocessing.Process(target=VisualisationModule.VisualisationModule, args=args)

		# self.visualisation_thread = multiprocessing.Process(target=VisualisationModule.VisualisationModule, args=(file_location, 1100, 800, 7))
		# self.visualisation_thread = multiprocessing.Process(target=VisualisationModule.VisualisationModule, args=(file_location, 1920, 1200, 10))
		# self.visualisation_thread = multiprocessing.Process(target=VisualisationModule.VisualisationModule, args=(file_location, 1024, 768, -1 ,True))

		self.visualisation_thread.start()
		# self.visualisation_thread.join()
		import random
		for i in range(-35,0):
			vehicle_id = i
			dummy_vehicle = dummyvehicle.DummyVehicle(self, 10, vehicle_id)
			self.bodies_dict[vehicle_id] = dummy_vehicle
			r = random.randint(1,3)
			if r == 1:
				dummy_vehicle.set_trajectory_on_lane('left')
			elif r == 2:
				dummy_vehicle.set_trajectory_on_lane('center')
			else:
				dummy_vehicle.set_trajectory_on_lane('right')
			
		
		for id in self.bodies_dict:
			dummy_vehicle = self.bodies_dict[id]
			dummy_vehicle.set_vehicle_on_trajectory_state(id*-40)
			dummy_vehicle.cruise_velocity = random.randint(30,60)
			dummy_vehicle.start_control_loop()

		'''
		Starting the interface with the Driver in the Loop
		'''
		# Create the UnityModule
		# If Unity running on the same computer
		# self.unity_module = UnityModule.UnityModule(self, unity_address = '127.0.0.1')
		# If Unity running on a different computer
		# self.unity_module = UnityModule.UnityModule(self, unity_address = '130.237.50.246')

		# print 'Starting Connections Server'
		# self.start_thread(self.start_connections_server, args=([serv]))

		'''
		Starting the Motion Capture System
		'''

		#self.motion_capture_module = MotionCaptureModule.MotionCaptureModule(self)


		#self.hmi.start()
		#self.hmi.update()
		return


def main():
	smlworld = SMLWorld()
	print 'Object created successfully'

	smlworld.main_connected = True

	#print "Load "+conf_file+" configuration file."
	#conf = Configuration.read_configuration_file()
	#timestep = conf["step"]
	#scenario = conf["scenario"]

	## decide what scenario(merging 1, intersection 2) and how many cars (only for scenario 1 )
	smlworld.start_sml_world()

	try:

		while not smlworld.close_sml_world:

			tic_main = time.time()

			toc_main = time.time()

			time.sleep(0.1)

	except KeyboardInterrupt:
		# smlworld.hmi.kill()
		smlworld.timed_print('KeyboardInterrupt')


if __name__ == "__main__":

	conf_file = "default.ini"
	if len(sys.argv)>1:
		if os.path.isfile(sys.argv[1]) == True:
			conf_file = sys.argv[1]
		else:
			print "Could not load configuration file "+str(conf_file)
			print "Default configuration file will be used instead"
	
	# Configuration.init_config_file(conf_file)

	main()
