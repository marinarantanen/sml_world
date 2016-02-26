
import time
import math

import vehiclesupervisorymodule

#Example of a dummy-supervisory

class GCDCEX:
	'''dummy supervisory to test the SML'''
	def __init__(self, vehicle):
		self.vehicle=vehicle
		self.state=None
		self.error=None
		self.message_incoming=None
		self.ID=None
		self.distance_desired=5
		self.speed_desired=5
		self.target_distance=None


	def step(self):
		self.state=[self.vehicle.position, self.vehicle.orientation, self.vehicle.velocity, self.vehicle.acceleration, self.vehicle.target_distance]
		self.error=[self.vehicle.perception_error, self.vehicle.control_error, self.vehicle.communication_error]
		self.message_incoming=self.vehicle.message_incoming

		self.target_distance=self.vehicle.target_distance
		self.ID=self.vehicle.get_MIO_id()

		if self.target_distance<7:
			follower=True
		else:
			follower=False


		if isinstance(self.ID,int):
			if follower:
				self.vehicle.set_ACCC_control(self.ID, self.distance_desired)
			else:
				self.vehicle.set_CC_control(self.speed_desired)