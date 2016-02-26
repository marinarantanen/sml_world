#__author__ = 'Carolina Eriksson'

'''Important! check so that all values exists in vheiclesupervisorymodule'''

import time


#needs to be fixed

class MessageModule:

#instansiating a message class, that is used for creating each message that is to be sent
	def __init__(self, vehicle,vehiclesupervisorymodule):
		self.supervisory = vehiclesupervisorymodule
		self.vehicle = vehicle

	##### These functions are for broadcasting #####


	def generateMessage(self, frequency):
		""" This function generates a dict that contains all the values listed below. This is for broadcasting """
		# make sure all entries for GCDC communication are here

		#messages sent at 25 Hz
		message_dict = dict()
		message_dict['header']='header'																#id 1 header
											#might be part of the header?
		message_dict['message_id'] = hash(message_dict['time'])
		message_dict['vehicle_id'] = self.vehicle.id												#id 3 Station ID
		message_dict['station_type']=5																#id 4 unknown(0), pedestrian(1), cyclist(2),moped(3), motorcycle(4), passengerCar(5), bus(6), lightTruck(7),heavyTruck(8), trailer(9), specialVehicles(10), tram(11),


		message_dict['vehicle_length'] = self.vehicle.length										#id 6
		message_dict['vehicle_rear_axle_location']=0													#id 7
		message_dict['vehicle_width'] = self.vehicle.width											#id 8
		message_dict['controller_type']=self.supervisory.type_controller															#id 9
		message_dict['vehicle_response_time_constant']=0												#id 10
		message_dict['vehicle_response_time_delay']=0												#id 11
		message_dict['reference_position']=0															#id 12
		#if hasattr(self.vehicle, 'bodies_dict'):
		#	message_dict['x'] = self.vehicle.bodies_dict[self.vehicle.id].x									#reference positions probably
		#	message_dict['y'] = self.vehicle.bodies_dict[self.vehicle.id].y
		message_dict['heading']=0																	#id 13
	#		message_dict['speed'] = self.vehicle.bodies_dict[self.vehicle.id].commands['throttle']		#id 14 speed =velocity
	#	message_dict['yaw_rate'] = self.vehicle.yaw																#id 15
		message_dict['long_vehicle_acc']=0															#id 16
		message_dict['desired_long_vehicle_acc']=0													#id 17
		message_dict['MIO_id']=self.supervisory.MIO_id										#id 18 Most Iportant Object in front needs to be defined in supervisorymodule
		message_dict['MIO_range']=self.supervisory.MIO_range								#id 19
		message_dict['MIO_bearing']=self.supervisory.MIO_bearing							#id 20
		message_dict['MIO_range_rate']=self.supervisory.MIO_range_rate						#id 21 velocity of MIO
		message_dict['time_headway']=self.supervisory.time_headway							#id 22 time to vehicle in front
		message_dict['cruise_speed']=self.supervisory.cruise_speed							#id 23 nominal speed of platoon
		message_dict['travelled_distance_CZ']=self.supervisory.travelled_CZ				#id 32
		message_dict['new_bwd_pair_partner'] = self.vehicle.new_bwd_pair_partner
		message_dict['new_fwd_pair_partner'] = self.vehicle.new_fwd_pair_partner
		message_dict['platooned_vehicle'] = self.vehicle.platooned_vehicle

		if frequency==1:
			#messages sent with 1Hz
			message_dict['vehicle_role']=0																#id 5 0 deafault, 6 emergency
			message_dict['merge_request_flag']=self.supervisory.merge_flag_request			#id 24
			message_dict['STOM']=self.supervisory.STOM										#id 25
			message_dict['merging_flag']=self.supervisory.merging_flag						#id 26
			message_dict['fwd_pair_partner'] = self.vehicle.fwd_pair_partner							#id 27
			message_dict['bwd_pair_partner'] = self.vehicle.bwd_pair_partner							#id 28
			message_dict['tail_vehicle_flag'] = self.vehicle.tail_vehicle					#id 29
			message_dict['head_vehicle_flag'] = self.vehicle.platoon_leader					#id 20
			message_dict['platoon_id'] = self.supervisory.platoon_id						#id 31
			message_dict['intention'] =self.supervisory.intention							#id 33
			message_dict['lane_entering_CZ'] = self.supervisory.lane_entering_CZ			#id 34
			message_dict['intersection_vehicle_counter'] = self.supervisory.intersection_vehicle_counter #id 35
			#message_dict['pair_acknowledge_flag'] = self.supervisory.pair_acknowledge_flag	#id 36 #currently not used in GCDC
			message_dict['participants_ready'] = True													#id 41 Not really used
			message_dict['start_scenario'] = True														# id 42
			message_dict['EoS'] = False																	#id 43

		return message_dict

	def generateMessage10(self):
		message_dict=dict()
		message_dict['reference_time']=self.supervisory.reference_time						#id 37
		message_dict['event_type'] = self.supervisory.event_type							#id 38
		message_dict['closed_lanes'] = False															#id 39
		message_dict['lane_position'] = False															#id 40

		return message_dict

	def rsu_message(self, event):
		message_dict=dict()

		if event=='start_scenario':
			message_dict['start_scenario'] = True														# id 42
			message_dict['EoS'] = False

		if event=='end_Scenario':
			message_dict['start_scenario'] = False														# id 42
			message_dict['EoS'] = True

		return message_dict