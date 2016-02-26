import math
import smartvehicle

import threading
import time

class V2VModule:
	'''
	The V2VModule simulates the communication exchanges between vehicles 
	in the SML World.
	The communication mechanisms available are:
	Network Communications: Every car can communicate with
	every other car (global range), at a slow rate
	Wi-Fi Communications: Every car can communicate with
	cars in its vicinity (Wi-Fi range), at a high rate

	'''

	def __init__(self, sml_world, v2v_wifi_range):

		self.sml_world = sml_world
		self.bodies_dict = sml_world.bodies_dict

		self.v2v_wifi_range = v2v_wifi_range
		# self.sml_world.v2v_info['rate'] = str(0)

		# self.sml_world.v2v_connected = True

		# Defines the rate at which messages
		# are exchanged over the network
		self.v2v_network_rate = 1.

		# Defines the rate at which messages
		# are exchanged over Wi-Fi
		self.v2v_wifi_rate = 10.

		# Auxiliary time keeper attributes to help
		# comply with the desired communication rates
		self.last_network_time = time.time() - 10 
		self.last_wifi_time = time.time() - 10

		# Launches the tread that will run the main
		# loop of the V2VModule
		t = threading.Thread( target = self.thread_loop, args=([]) )
		t.daemon = True
		t.start()

	def thread_loop(self):
		'''
		This function is executed in a thread, thus running 
		in parallel with the rest of the SML World.
		It consists of a loop, where the Network and
		Wi-Fi communications are simulated.
		'''

		# Run the loop while the SML World does not shut down
		while not self.sml_world.close:

			current_time = time.time()

			# Check if it is time to run the Wifi Step
			if current_time - self.last_wifi_time > 1./self.v2v_wifi_rate:

				self.wifi_step()
				self.last_wifi_time = current_time
				self.wifi_step()

			# Check if it is time to run the Network Step
			if current_time - self.last_network_time > 1./self.v2v_network_rate:

				self.network_step()
				self.last_network_time = current_time
				self.network_step()

			time.sleep( (1./self.v2v_wifi_rate)/2. )

		self.sml_world.v2v_connected = False

	def network_step(self):
		'''
		The network step will simulate a communications network
		Basically the output of every car, given by v2v_network_output, will
		be the input of every other car (v2v_network_input)
		At the end of the step, every car's v2v_network_output is emptied
		'''

		# I will get all of the messages that are being outputted by vehicles
		current_network_inputs_dict = dict()

		for body_id in self.bodies_dict:

			body = self.bodies_dict[body_id]

			if not isinstance(body, smartvehicle.SmartVehicle) and body_id != -100:

				continue

			# current_output_messages will be a list of messages, or None
			# if there are no messages to output
			current_output_messages = body.get_network_output_messages()

			if current_output_messages:

				current_network_inputs_dict[body_id] = current_output_messages

		# I will put every message in every vehicle input
		for body_id in self.bodies_dict:

			if isinstance(self.bodies_dict[body_id], smartvehicle.SmartVehicle) or body_id == -100:

				current_input_messages = []

				for sender_id in current_network_inputs_dict.keys():

					if sender_id != body_id:

						current_input_messages.extend( current_network_inputs_dict[sender_id] )

				if current_input_messages:

					self.bodies_dict[body_id].set_network_input_messages(current_input_messages)

		return
	
	def wifi_step(self):
		'''
		The wifi step will simulate a communications network
		Basically the output of every car, given by v2v_wifi_output, will
		be the input of every other car in a close vicinity (v2v_wifi_range)
		At the end of the step, every car's v2v_wifi_output is emptied
		'''


		current_wifi_output_dict = dict()

		for body_id in self.bodies_dict:

			body = self.bodies_dict[body_id]

			if isinstance(body, smartvehicle.SmartVehicle):

				current_wifi_output_dict[body_id] = body.get_wifi_output_messages()


		current_body_ids = self.bodies_dict.keys()

		for ego_body_id in current_body_ids:

			if ego_body_id in self.bodies_dict:
			# Need to check if dictionary still has the item

				ego_body = self.bodies_dict[ego_body_id]

				if isinstance(ego_body, smartvehicle.SmartVehicle):

					current_wifi_inputs = []

					ego_body = ego_body

					for other_body_id in current_body_ids:

						if other_body_id in self.bodies_dict:
						# Need to check if dictionary still has the item
							other_body = self.bodies_dict[other_body_id]

							if ego_body_id == other_body_id or not isinstance(other_body, smartvehicle.SmartVehicle):

								continue

							distance = math.hypot(ego_body.x - other_body.x, ego_body.y - other_body.y) 

							if distance > self.v2v_wifi_range:

								continue

							# v2v_wifi_output will be a list of messages, or None
							# if there are no messages to output
							v2v_wifi_output = current_wifi_output_dict[other_body_id]

							if v2v_wifi_output:
								
								current_wifi_inputs.extend(v2v_wifi_output)

					if current_wifi_inputs:

						ego_body.set_wifi_input_messages(current_wifi_inputs)

		return

		