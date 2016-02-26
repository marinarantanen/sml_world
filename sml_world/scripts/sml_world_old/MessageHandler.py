import time, math
import threading, Queue
import collections

from message_protocols.command_protocols import *
from message_protocols.misc_protocols import *
from message_protocols.merge_protocols import *
from message_protocols.request_protocols import *


class MessageHandler:

	""" This class is devided into two specific parts.
		
		One broadcast generator part which generates a broadcast message (it is a dict with current values of some selected vehicle variables)

		One handler for sending and receiving targeted messages. I.e. vehicle 1 can send a message to vehicle 2.
	"""

	def __init__(self, vehicle):
		self.vehicle = vehicle

		# Below are variables used for targeted communications


		print "initializing events"
		self.incoming_message = threading.Event()

		print "initializing message queues"
		self.in_queue = collections.deque()
		self.handle_queue = collections.deque()

		print "initializing messageing threads"
		self.done = False
		self.in_thread = threading.Thread(target = self.incomingThread)
		self.in_thread.daemon = True
		self.in_thread.start()


	def generateMessage(self):
		""" This function generates a dict that contains all the values listed below. This is for broadcasting """
		# make sure all entries for GCDC communication are here

		message_dict = dict()
		message_dict['time'] = time.time()
		message_dict['message_id'] = hash(message_dict['time'])
		message_dict['vehicle_id'] = self.vehicle.id
		message_dict['vehicle_length'] = self.vehicle.length
		message_dict['vehicle_width'] = self.vehicle.width
		message_dict['smartvehicle'] = True
		message_dict['x'] = self.vehicle.bodies_dict[self.vehicle.id].x
		message_dict['y'] = self.vehicle.bodies_dict[self.vehicle.id].y
		message_dict['velocity'] = self.vehicle.bodies_dict[self.vehicle.id].commands['throttle']
		message_dict['yaw_rate'] = 0.
		message_dict['desired_lane'] = self.vehicle.desired_lane
		message_dict['platoon_id'] = 0
		message_dict['fwd_pair_partner'] = self.vehicle.fwd_pair_partner
		message_dict['bwd_pair_partner'] = self.vehicle.bwd_pair_partner
		message_dict['new_bwd_pair_partner'] = self.vehicle.new_bwd_pair_partner
		message_dict['new_fwd_pair_partner'] = self.vehicle.new_fwd_pair_partner
		message_dict['platooned_vehicle'] = self.vehicle.platooned_vehicle


		return message_dict


	##### Here we have targeted communications over "network" or "wifi" #####


