'''
This file implements functions to read the OSM Map file
that describes the SML World environment.

This file is used by RoadModule.py

'''

import sys
import os
import numpy as np

import rospy
from sml_modules.road_module.RoadModule import RoadModule
from sml_world.srv import GetMapLocation, GetMapLocationResponse

class BusNetworkModule(RoadModule):
	"""
	Extention of the RoadModule class to describe bus stop locations
	args: 
	"""

	def __init__(self, file_location):
		self.map_location = file_location
		self.base_path = os.path.dirname(__file__)
		super(RoadModuleExtend, self).__init__(base_path, file_location)

		print(self.osm_node_tag_dict)

		#Get all tags with stop tag
		self.bus_station_nodes = self.osm_node_tag_dict['stop']
		self.num_stops = self.bus_station_nodes.length
		rospy.loginfo("This is a message")
		self.distance_matrix = np.zeros((self.num_stops, self.num_stops)) 
		for i in range(self.num_stops):
			for j in range(self.num_stops):
				self.distance_matrix[i, j] = self.get_path_distance_between_node_ids(self.bus_station_nodes[i], self.bus_station_nodes[j])

	def handle_get_map_location(self, req):
        """
        Handle the map location request.

        @param req: I{(GetMapLocation)} Request of the service that provides
                    the map location to client.
        """
        return GetMapLocationResponse(self.base_path, self.map_location)


def bus_network(file_location):
	road_module = BusNetworkModule(file_location)
	rospy.init_node('bus_network')

	rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        file_location = sys.argv[1]
    else:
        msg = "Usage: rosrun sml_world bus_network.py <file_location>"
        raise Exception(msg)
    bus_network(file_location)