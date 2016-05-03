#!/usr/bin/env python
# license removed for brevity
'''
This file implements functions to read the OSM Map file
that describes the SML World environment.

This file is used by RoadModule.py

'''

import sys
import os
import numpy as np
import copy
import rospy

from sml_modules.road_module.RoadModule import RoadModule
from sml_world.srv import GetMapLocation, GetMapLocationResponse
from sml_world.srv import StartBusRoute, StartBusRouteResponse
from sml_modules.bus_vehicle_model import BusVehicle
from sml_world.msg import BusInformation, BusStops
from sml_world.srv import SpawnVehicle, CalculateRoute, CalculateRouteResponse
from sml_world.srv import BusRouting, BusRoutingResponse
from sml_modules.demand_model import DemandModel



class BusNetworkModule(RoadModule):
	"""
	Extention of the RoadModule class to describe bus stop locations
	args: 
	"""

	def __init__(self, file_location):

		'''
		rospy.wait_for_service('/get_map_location')
		try:
			get_map = rospy.ServiceProxy('/get_map_location', GetMapLocation)
			resp = get_map()
			base_path = resp.base_path
			map_location = resp.map_location
			file_location = base_path + map_location
		except rospy.ServiceException, e:
			raise "Service call failed: %s" % e
		'''
		self.map_location = file_location
		self.base_path = os.path.dirname(__file__)
		super(BusNetworkModule, self).__init__(self.base_path, file_location)

		#Get all id tags with stop tag
		self.bus_station_nodes = self.osm_node_tag_dict['stop']
		#Let sml central know about the bus stops
		pub_stops = rospy.Publisher('current_bus_stops', BusStops, queue_size=10)

		#Tracks the nodes that a bus id has been assigned by user
		self.responsible_nodes_by_bus = {}

		#Complete OSM node for stations
		self.bus_stations = [self.osm_node_dict[ index ] for index in self.bus_station_nodes]

		self.num_stops = len(self.bus_station_nodes)

		rospy.Service('/bus_rerouting', BusRouting,
								self.bus_needs_rerouting)

		self.serv_states = {} #Dictionary of bus ids to service states

		#Dictionary of tuples to ints that represents distances between node_ids
		self.distance_dict = {}
		for i in self.bus_station_nodes:
			for j in self.bus_station_nodes:
				if(i != j):
					#CHECK
					self.distance_dict[(i, j)] = self.get_path_distance_between_node_ids(i, j)
				else:
					self.distance_dict[(i, j)] = 0

		self.demand_model = DemandModel()
		for nodeid in self.bus_station_nodes:
			self.demand_model.register_bus_stop(nodeid, 50)

		# Wait until sml world is running
		rospy.wait_for_service('spawn_vehicle')
		#Sleep for 10 ms because concurrency is hard
		r = rospy.Rate(10)
		r.sleep()


		self.bus_stop_status = BusStops()
		self.bus_stop_status.bus_stops = self.bus_station_nodes
		self.get_and_publish_demands(pub_stops)
		
		#Example how to add bus:
		#self.add_bus(181, self.bus_station_nodes, -282)
		#Example how to add demand (use negatives to subtract)
		#self.add_demand(-282, 10)


	def handle_get_map_location(self, req):
		"""
        Handle the map location request.

        @param req: I{(GetMapLocation)} Request of the service that provides
        the map location to client.
		"""
		return GetMapLocationResponse(self.base_path, self.map_location)

	def get_and_publish_demands(self, publisher):
		self.bus_stop_status.bus_stop_demands = self.demand_model.get_all_demands(self.bus_station_nodes)
		publisher.publish(self.bus_stop_status)

	def add_demand(self, node, num):
		self.demand_model.add_demand(node, num)


	def add_bus(self, bus_id, responsible_nodes, startnode):
		'''
		Service for adding a bus with a list of nodes that it is responsible for
		picking up passengers from
		'''
		for node in responsible_nodes:
			if not node in self.bus_station_nodes:
				raise NameError(str(node) + ' is not the location of a bus node!')

		self.responsible_nodes_by_bus[bus_id] = responsible_nodes

		bus_launch_service = '/spawn_vehicle'
		rospy.wait_for_service(bus_launch_service)
		try:
			bus_launch = rospy.ServiceProxy(bus_launch_service, SpawnVehicle)
			# All the 0s will be unused in bus creation because startnode accounts for it
			response = bus_launch(bus_id, BusVehicle.__name__, 0, 0, 20, 20., None, True)
		except rospy.ServiceException, e:
			raise Exception(e)
		return StartBusRouteResponse()

	def bus_needs_rerouting(self, data):
		return self.bus_needs_rerouting_logic(data.busid, data.current_node_id)

	def bus_needs_rerouting_logic(self, bus_id, start_node):
		responsible_nodes = copy.deepcopy(self.responsible_nodes_by_bus[bus_id])
		new_cycle = self.calculate_best_cycle(responsible_nodes, start_node)
		stations_cycle = [self.osm_node_dict[i] for i in new_cycle]
		return BusRoutingResponse(new_cycle)

	def bus_information_update(self, bus_info):
		# Implement some stuff later
		print('Nothing yet')

	def calculate_best_cycle(self, responsible_nodes, start_node):
		'''
		Finds lowest cost cycle on responsible_nodes that touches all nodes.
		Note that this is similar to the traveling salesman problem, which is famously
		NP-hard.
		Thus, we use a greedy algorithm to get an approximate answer.
		There may be a point in time when we want to consider the Christofides algorithm,
		which includes the use of spanning trees.
		@param responsible_nodes: List of integer nodes for bus to go to
		@param start_node: int of start node: Does not necessarily have to be a bus stop
		'''
		cycle = []
		dist = 0
		nodes_left = list(responsible_nodes)

		#Explicitly define first move since starting node may not be a bus stop
		possible_first_moves = [(i, self.get_path_distance_between_node_ids(start_node, i))
													for i in nodes_left]
		min_first_move = min(possible_first_moves, key = lambda t: t[1])
		cur_node = min_first_move[0]
		dist += min_first_move[1]
		ret = self.calculate_best_cycle_exhaustively(responsible_nodes, cur_node)
		return ret[1]

		'''
		#Simple search algorithm that is time efficient but may not return a good result
		nodes_left.remove(cur_node)
		cycle.append(cur_node)
		while nodes_left:
			possible_moves = [(i, self.distance_dict[(cur_node, i)]) for i in nodes_left]
			min_move = min(possible_moves, key = lambda t: t[1])
			cur_node = min_move[0]
			cycle.append(cur_node)
			nodes_left.remove(cur_node)
			dist += min_move[1]
		if start_node in responsible_nodes:
			cycle.append(start_node)
			dist += self.distance_dict[(cur_node, start_node)]
		
		return cycle
		'''

	def calculate_best_cycle_exhaustively(self, responsible_nodes, start_node):
		'''
		NP-hard solution that searches through all possible paths to find the optimal osm_node_dict

		returns tuple of the cost of the best path and the path itself as a list

		'''
		responsible_nodes.remove(start_node)
		if not responsible_nodes:
			return (0, [start_node])

		possible_paths = []
		for target in responsible_nodes:
			dist_to_next = self.get_path_distance_between_node_ids(start_node, target)
			if dist_to_next != float('inf'):
				ret = self.calculate_best_cycle_exhaustively(copy.deepcopy(responsible_nodes), target)
				cost_found = ret[0] + dist_to_next
				path_found = ret[1]
				path_found.insert(0, start_node)
				possible_paths.append((cost_found, path_found))
		
		if not possible_paths:
			return (float('inf'), possible_paths)
		return min(possible_paths, key = lambda t: t[0])


def bus_network(file_location):
	rospy.init_node('bus_network')
	bus_module = BusNetworkModule(file_location)
	rospy.Service('/start_bus_route', StartBusRoute,
		bus_module.add_bus)
	rospy.Subscriber('/bus_information', BusInformation,
				bus_module.bus_information_update)
	rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        file_location = sys.argv[1]
    else:
        msg = "Usage: rosrun sml_world bus_network.py <file_location>"
        raise Exception(msg)
    bus_network(file_location)
