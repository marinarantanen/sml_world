import socket, sys, time, math, random, threading

import bodyclasses

class BlocklyVehicle(bodyclasses.WheeledVehicle):

	def __init__(self, body_id, simulated = True):
	
		self.id = body_id

		self.simulated = simulated

		self.axles_distance = 4.7

		self.x = 0
		self.y = 0
		self.yaw = 0

		self.commands = dict()
		self.commands['speed'] = 0.
		self.commands['steering'] = 0.

		self.radar_sensor = False
		self.velodyne_sensor = False
		self.omniscient_sensor = False

	'''
	Vehicle State Update Interface
	Author: Rui
	This method allows the SimulatorModule to interact
	with the SmartVehicle in a cleaner / more
	Object Oriented way
	'''
	def vehicle_state_update(self, time_to_simulate):
		'''
		Defines the state update of the vehicle, given 
		the time_to_simulate in seconds.

		This vehicle state update assumes the vehicle 
		model is an acceleration based unicycle model.

		This method is called by the SimulatorModule, for 
		at every simulation loop.
		'''

		# Knowing the current linear speed, I can know the speed components in x and y
		self.x_speed = math.cos( math.radians( self.yaw ) )*self.commands['speed']
		self.y_speed = math.sin( math.radians( self.yaw ) )*self.commands['speed']
		
		# Update my position
		self.x = self.x + self.x_speed*time_to_simulate
		self.y = self.y + self.y_speed*time_to_simulate

		linear_speed = math.hypot(self.x_speed, self.y_speed)

		# Update my yaw, assuming a simple kinematic car model
		self.yaw = self.yaw + math.degrees( (linear_speed/self.axles_distance)*math.tan(self.commands['steering'])*time_to_simulate )
		
		# Limit the yaw to the range between 0 and 360
		while self.yaw > 360:

			self.yaw -= 360

		while self.yaw < 0:

			self.yaw += 360