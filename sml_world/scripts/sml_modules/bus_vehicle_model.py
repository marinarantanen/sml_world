"""
Module containing bus vehicle
Created on Mar 5, 2016

@author: U{Daniel Carballal<carba@kth.se>}
@organization: KTH
"""

import rospy
from sml_modules.vehicle_models import BaseVehicle
from sml_world.srv import SetDestination
from sml_world.srv import GetNearestNodeId, BusRouting, AddDemand
from sml_world.msg import BusInformation
from sml_world.srv import GetCoordinates, SetDestination, SetDestinationResponse, SetVehicleState

from copy import deepcopy

WAIT_AT_BUS_STATION_DURATION = 3


class BusVehicle(BaseVehicle):
	def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=10.):
		'''
		Same as parent, with route being an list of bus nodes to visit
		and network an instanciation of the main bus network
		stops are list of OSM nodes 
		'''
		self.id = vehicle_id
		super(BusVehicle, self).__init__(namespace, vehicle_id,
                                           simulation_rate, x, y, yaw, v)
		self.stops = []

		self.pub_state_info = rospy.Publisher('/bus_information', BusInformation, queue_size=100)
		self.request_new_stops()

		while self.stops == None:
			i = 0

		rospy.logwarn('Stops order is ' + str(self.stops))
		self.cur_destination = self.stops.pop(0)
		(self.x, self.y) = self.get_node_coordinates(self.cur_destination)
		req = type("SetState", (object,),{})
		setattr(req, 'x', self.x)
		setattr(req, 'y', self.y)  
		setattr(req, 'yaw', 0)  
		setattr(req, 'v', 10)
		self.handle_set_state(req)

		self.go_to_node(self.cur_destination)


	def simulation_step(self):
		super(BusVehicle, self).simulation_step()
		if self.at_dest:
			#Stop to allow for passengers to get on/off
			saved_v = self.v
			self.v = 0
			req = type("SetState", (object,),{})
			setattr(req, 'speed', 0)
			self.handle_set_speed_kph(req)
			super(BusVehicle, self).simulation_step()

			# "Remove" some demand at that bus station
			self.add_demand_to_model(self.cur_destination, -20)
			rospy.sleep(WAIT_AT_BUS_STATION_DURATION)
			setattr(req, 'speed', 56)
			self.handle_set_speed_kph(req)
			if not self.stops:
				self.request_new_stops()
			new_dest = self.stops.pop(0)
			self.go_to_node(new_dest, self.cur_destination)
			self.cur_destination = new_dest

	def add_demand_to_model(self, bus_id, demand_added):
		rospy.wait_for_service('/add_demand')
		d_add = rospy.ServiceProxy('/add_demand', AddDemand)
		d_add(bus_id, demand_added)


	def request_new_stops(self):
		'''
		Sends a bus information message, and 
		'''
		#Waiting is probably not required, but safe
		rospy.wait_for_service('/get_nearest_nodeid')
		try:
			get_nodeid = rospy.ServiceProxy('/get_nearest_nodeid', GetNearestNodeId)
			current_node = get_nodeid(self.x, self.y, -10).node_id
		except rospy.ServiceException, e:
			raise "Service call failed: %s" % e
		rospy.wait_for_service('/bus_rerouting')
		try:
			reroute_command = rospy.ServiceProxy('/bus_rerouting', BusRouting)
			new_path = list(reroute_command(current_node, self.id).ordered_stops)
		except rospy.ServiceException, e:
			raise NameError("Could not get bus routing from bus central: %s" % e)
		self.set_new_route(new_path)
		#Central system will now calculate a new route and respond to it

	def set_new_route(self, new_route):
		self.stops = deepcopy(new_route)

	def go_to_node(self, node_id, origin_id = 0):
		'''
		Use vehicle's parent class' set destination service to set the current destination
		@param node_id: node_id to set trajectory towards
		@param origin_id: optional, enables user to specify id of origin node
		'''
		rospy.wait_for_service(self.namespace + 'set_destination')
		dest = SetDestination()
		dest.origin_id = origin_id
		dest.dest_id = node_id
		super(BusVehicle, self).handle_set_destination(dest)

		#set_dest = rospy.ServiceProxy(self.namespace + 'set_destination', SetDestination)
		#resp = set_dest(node_id, origin_id)

	def get_node_coordinates(self, node_id):
		'''
		Talks with road network to get xy coordinates of nodes
		Returns tuple with x and y coordinates in form (x,y)
		'''
		rospy.wait_for_service('/get_node_coordinates')
		try:
			get_coordinates = rospy.ServiceProxy('/get_node_coordinates', GetCoordinates)
			coords = get_coordinates(int(node_id))
		except rospy.ServiceException, e:
			raise e
		return (coords.x, coords.y)

if __name__ == '__main__':
	# Filter sys.argv to remove automatically added arguments
	sys.argv = [arg for arg in sys.argv if str(arg).find(':=') < 0]
	args = {}
	if len(sys.argv) == 3:
		args['vehicle_id'] = int(sys.argv[1])
		args['stops'] = sys.argv[2].split(',')
		args['startnode'] = int(sys.argv[3])
	else:
		msg = ("Usage: rosrun sml_world bus_vehicle_model.py " +
                "<vehicle_id> <vehicle_class> <stops>" +
                "Stops are in the form 1,2,3,4")
		raise Exception(msg)
	args['simulation_rate'] = 20
	BusVehicle(**args)