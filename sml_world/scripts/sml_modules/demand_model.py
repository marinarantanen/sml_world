#!/usr/bin/env python
# license removed for brevity
'''
Class for modeling the demand of various bus stops
Demand is modeled as a scale from 0 to 100
Theoretically this represents the number of passengers per hour
'''

import rospy
from sml_world.srv import AddDemand, AddDemandResponse
from sml_world.msg import BusStops, GetTime, ProjectedDemand
from threading import Thread
import time

class DemandModel():
	def __init__(self):
		self.demand_dict = dict()
		# Simple boolean to let object know whether the service provided is up to date
		self.modern_demands = True
		rospy.Service('/add_demand', AddDemand,
			self.add_demand_request)

		#Publisher that will inform the bus network to update it's demand stats
		self.pub_state = rospy.Publisher('/update_demand_stats', BusStops, queue_size=5)
		self.klockan = SMLClock(700)

	def register_bus_stop(self, bus_stop_id, initial_demand):
		self.demand_dict[bus_stop_id] = initial_demand
		self.klockan.add_bus_stop(bus_stop_id)

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
		return AddDemandResponse()

#Simple clock class used by DemandModel to model variance throughout the day
#It uses a simple int32 to talk into and out of.
#For example time= 1356 means the time is 13:56  
class SMLClock():
	def __init__(self, starttime):
		self.cur_time = starttime
		self.publisher = rospy.Publisher('/get_time', GetTime, queue_size=5)
		self.proj_demand = rospy.Publisher('/projected_demand', ProjectedDemand, queue_size = 5)
		self.bus_stops = []

		loop_thread = Thread(target=self.loop)
		loop_thread.start()

		self.static_demand = [3, 4, 7, 2, 1, 4, 8, 7, 15, 24, 15, 10, 29,
						13, 10, 11, 16, 20, 21, 23, 15, 17, 14, 9]

		self.proj_demand.publish(ProjectedDemand(self.get_next_four_hours()))


	def loop(self):
		while not rospy.is_shutdown():
			self.increment_min()
			time.sleep(.5)

	def increment_min(self):
		cur_hour = self.cur_time / 100
		cur_min = (self.cur_time % 100) + 1
		if cur_min == 60:
			self.add_demand_to_model_all_buses(self.get_static_demand())
			self.cur_time = ((cur_hour + 1) % 24) * 100
			self.proj_demand.publish(ProjectedDemand(self.get_next_four_hours()))
		else:
			self.cur_time = cur_hour * 100 + cur_min
		self.publisher.publish(GetTime(self.cur_time))

	def get_static_demand(self):
		hour = self.cur_time / 100
		return self.static_demand[hour]

	def get_next_four_hours(self):
		hour = self.cur_time / 100
		return self.static_demand[hour:((hour + 5) % 24)]

	def add_demand_to_model_all_buses(self, demand_added):
		rospy.wait_for_service('/add_demand')
		d_add = rospy.ServiceProxy('/add_demand', AddDemand)
		for bus_id in self.bus_stops:
			d_add(bus_id, demand_added)

	def add_bus_stop(self, bus_id):
		self.bus_stops.append(bus_id)


