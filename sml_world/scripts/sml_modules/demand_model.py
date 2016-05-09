#!/usr/bin/env python
# license removed for brevity
'''
Class for modeling the demand of various bus stops
Demand is modeled as a scale from 0 to 100
Theoretically this represents the number of passengers per hour
'''

import rospy
from sml_world.srv import AddDemand, AddDemandResponse
from sml_world.msg import BusStops

class DemandModel():
	def __init__(self):
		self.demand_dict = dict()
		# Simple boolean to let object know whether the service provided is up to date
		self.modern_demands = True
		rospy.Service('/add_demand', AddDemand,
			self.add_demand_request)

		#Publisher that will inform the bus network to update it's demand stats
		self.pub_state = rospy.Publisher('/update_demand_stats', BusStops, queue_size=5)



	def register_bus_stop(self, bus_stop_id, initial_demand):
		self.demand_dict[bus_stop_id] = initial_demand

	def get_demand(self, bus_stop_id):
		return self.demand_dict[bus_stop_id]

	def get_all_demands(self, bus_stop_ids):
		'''
		Returns all demands in the same order as the bus stop array arg
		'''
		ret = []
		for stop in bus_stop_ids:
			ret.append(self.get_demand(stop))
		return ret

	def add_demand(self, bus_stop_id, demand_change):
		'''
		Can be used to add or subtract demand for a particular bus stop 
		'''
		self.demand_dict[bus_stop_id] = min(100, max(0, self.demand_dict[bus_stop_id] + demand_change))
		self.modern_demands = False
		self.pub_state.publish()

	def add_demand_global(self, demand_change):
		'''
		Can be used to add or subtract demand for all bus stops.
		For instance, a national celebration day increases demand for all passengers.
		'''
		for stop_id in self.demand_dict.keys():
			self.add_demand(stop_id, demand_change)

	def add_demand_request(self, data):
		self.add_demand(data.bus_id, data.demand_change)
		rospy.logwarn('Success!')
		return AddDemandResponse()