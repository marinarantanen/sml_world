import time, math, random, threading
import xml.etree.ElementTree as ET
from PingReturn import PingReturn
import smartvehicle

import pprint


# This will be an interface only!


# TIPS:

# Starting velocity and lane is set in smartvehicle (currently lines 120-125)
# To set velocity use the self.setVelocity(new_velocity) function
# self.getVelocty() returns your current velocity, self.vehicle.desired_velocity points to the velocity the control module is trying to keep
# To change lane use the self.addLaneChange(new_lane) function
# To change/set a new pair partner, use the self.setPairPartner(target_id, fwd = True/False, bwd = True/False, new = True/False) function

# Things from perception_module:
# 	- self.vehicle.perception_grid is a dictionary with keys 'fl','fm','fr','l','r','rl','rm','rr', with values lists of vehicle ids in that part of the grid
#	- self.vehicle.perceived_objects is a dictionary with vehicle ids as keys, and lists of relative x_coord, relative y_coord and velocity as values
#	- self.vehicle.closest_objects is a dictionary with the same keys as perception_grid, but only contains the id of the vehicle closest to you in that part of the grid
#	- self.vehicle.sensor_readings is a list with each element as a list of readings. Those readings contain vehicle id, x, y, speeds etc

# right lane is 2026 points, center lane is 1899 points, so to keep the vehicles approximately at the same distance from each other (over all) a center lane vehicle should have 0.937 of the right lane vehicle velocity


class VehicleSupervisoryModule:
	"This class is the Platooning Vehicle class used by the Platooning Manager Module"
	def __init__(self, vehicle):

		## gains access to the vehicle variables
		self.vehicle = vehicle

		## Definition of string constants, which are keys for the trajectory dictionary in smartvehicle (used to change lanes)
		self.RIGHT_LANE_STRING = "right_lane"
		self.CENTER_LANE_STRING = "center_lane"
		self.LEFT_LANE_STRING = "left_lane"

		self.platoon_merge_distance_factor = 7.3 ## Determines distance from obstacle at which platoon merge is initiated. Good value: 4.30

		## Below are minimum values for the variables that determine the distance used as reference by respective safety checks
		self.min_emergency_brake_distance = 5.
		self.min_emergency_velocity_distance = 15.
		self.min_emergency_velocity_distance_platoon_merge = 20.
		self.min_overtake_distance = 18.
		self.min_front_lc_safe_dist = 10. ## It seems to work better if this is slightly greater than the rear safe distance
		self.min_rear_lc_safe_dist = 10.

		## Not used, just for testing
		self.dummy_time = time.time()
		self.dummy_variable = False


		self.module_start_time = time.time()
		self.modules_initiation_delay = 2.5 ## kind of includes time it takes for vehicles to accelerate to starting velocity
		self.modules_initiated = False
		self.messaging_initiation_delay = 3.5 ## after this amount of seconds it is assumed that broadcasting is working properly, and smartvehicles are aware of all other smartvehicles
		self.messaging_initiated = False
		self.broadcastMergePeriod = 7. ## vehicles will only use broadcasting to merge during this amount of seconds after the module start time

		self.ping_time = time.time() ## used as reference time to ping at a certain frequencey, specified by the ping_rate
		self.ping_rate = 10. ## amount of times per second vehicles ping info to their bwd and new bwd partners

		self.closeby_emergency_vehicle_distance = 100.
		self.avoid_emergency_vehicle_distance = 50.

		self.received_broadcast_ids = []
		self.broadcast_sent = False

		print "VehicleSupervisoryModule started"





	def step(self):
		## This is the step function that is running at each cycle of the thread in SmartVehicle

		if hasattr(self.vehicle,'control_module'):
			if self.vehicle.perception_grid!=[]:
				if 'fm' in self.vehicle.closest_objects:
					id = self.vehicle.closest_objects['fm']
					if isinstance(id,int):
						if id!=self.vehicle.control_module.ACCC_target_id:
							self.setACCCTarget(id, 30.0)

		if time.time() - self.dummy_time > 5.0 and self.vehicle.id==-3:
			self.vehicle.desired_lane = self.CENTER_LANE_STRING
			self.vehicle.control_module.update_lane()

	def setVelocity(self, new_velocity_mps, temporary = None, alert = True): # you can specify if the velocity change is temporary, and if you want to alert bwd/new bwd partners of your new velocity
		'''
		This function should be used whenever you want to change velocity, it performs the necessary checks to make sure the velocity change is allowed.
		If you still want to change the velocity regardless of these flags you have to set the desired_velocity variable outside of this function
		'''

		## limit, so that you can't set too high a velocity (the vehicles have trouble keeping their trajectories at higher velocities)
		high_limit_velocity = 40.
		if new_velocity_mps > high_limit_velocity:

			new_velocity_mps = high_limit_velocity
			print "WARNING - For vehicle",self.vehicle.id,": setVelocity() got velocity greater than 50 m/s, used the limit value", new_velocity_mps, "m/s instead"

		## a new velocity will not be set if you have an emergency velocity or a temporary velocity. the temporary velocity flag points to the vehicle that sent the temporary velocity in the first place
		if not self.vehicle.emergency_velocity_flag and self.vehicle.temporary_velocity_flag == None:
			self.vehicle.desired_velocity = new_velocity_mps
			if alert:
				self.alertFollowers(new_velocity = new_velocity_mps, temporary = temporary)

		## this should take care of the case where the temporary_id changes its own velocity to new temp velocity
		if temporary == self.vehicle.temporary_velocity_flag:
			self.vehicle.desired_velocity = new_velocity_mps
			if alert:
				self.alertFollowers(new_velocity = new_velocity_mps, temporary = temporary)


	def getVelocity(self):
		''' Returns the current velocity '''

		velocity = math.hypot(self.vehicle.x_speed, self.vehicle.y_speed)
		return velocity


	def setACCCTarget(self, ID, desired_distance): ## ID is the id of the vehicle you want to follow, desired_distance is the distance the controller will try to keep
		''' Sets ACCC target and necessary variables, clears ACCC_ready and returns True if it succeeded. Decomment lateral_following if you want, though it doesn't work well '''

		if self.vehicle.control_module:
			print str(self.vehicle.id)+" setting accc on "+str(ID)
			control_module = self.vehicle.control_module
			control_module.ACCC_target_id = ID
			control_module.ACCC_desired_distance = desired_distance
			control_module.ACCC = True

			# control_module.lateral_following = True
			self.vehicle.ACCC_ready.clear()
			return True
		else:
			print "No control module"
			return False

	def resetACCC(self):
		''' Turns of ACCC and resets all the necessary variables '''

		if self.vehicle.control_module:
			self.vehicle.control_module.ACCC = False
			self.vehicle.control_module.ACCC_target_velocity = None
			self.vehicle.control_module.lateral_following = False
			self.vehicle.control_module.ACCC_desired_distance = None
			self.vehicle.control_module.ACCC_distance = None
			self.vehicle.control_module.ACCC_target_id = None
			self.vehicle.ACCC_ready.clear()




	def handleBroadcasts(self):
		''' Handles messages sent through the wifi and network. Note: not private messages. '''

		# Check for initialization
		if self.vehicle.id in self.vehicle.bodies_dict:

			if self.modules_initiated:
				## Saves set of smartvehicle ids to use as reference for other functions
				self.saveSmartvehicleIDS()


				## If you are not merged/merging you will try to merge with someone during the first few seconds of the program runtime, specified by self.broadcastMergePeriod
				if not self.vehicle.fwd_pair_partner and not self.vehicle.new_fwd_pair_partner and time.time() < self.module_start_time + self.broadcastMergePeriod:


					candidate = self.searchFwdPairPartnerBroadcast(self.vehicle.v2v_wifi_input)

					if candidate == None:
				 		candidate = self.searchFwdPairPartnerBroadcast(self.vehicle.v2v_network_input)

				 	## REAL VEHICLES WILL NOT SEARCH FOR PARTNERS, AND SIMULATED VEHICLES WILL NOT CHOOSE REAL VEHICLES
					if self.vehicle.id > 0 or candidate > 0:
						candidate = None

				 	if candidate != None and candidate != self.vehicle.id:
				 		self.vehicle.messenger.sendDesiredMerge(candidate)


				## Checking for obstacles or emergency vehicles
				self.checkSpecialMessages(self.vehicle.v2v_wifi_input)
				self.checkSpecialMessages(self.vehicle.v2v_network_input)

			## Clearing the input
			self.vehicle.v2v_wifi_input = []
			self.vehicle.v2v_network_input = []

			## Creating the output messages to be read by other vehicles
			# print "UNCOMMENT"
			# self.vehicle.bodies_dict[self.vehicle.id].v2v_network_output = [self.vehicle.messenger.generateMessage()]
			# self.vehicle.bodies_dict[self.vehicle.id].v2v_wifi_output = [self.vehicle.messenger.generateMessage()]


	def saveSmartvehicleIDS(self):
		''' Goes through the network broadcasts and saves smartvehicle ids in a set() '''

		for broadcast in self.vehicle.v2v_network_input:
			if not 'smartvehicle' in broadcast:
				continue
			else:
				if broadcast['vehicle_id'] not in self.vehicle.saved_smartvehicle_ids:
					self.vehicle.saved_smartvehicle_ids.add(broadcast['vehicle_id'])


		# Added by Rui
		for vehicle_id in self.vehicle.bodies_dict:

			if isinstance(self.vehicle.bodies_dict[vehicle_id], smartvehicle.SmartVehicle):

				self.vehicle.saved_smartvehicle_ids.add(vehicle_id)


	def checkSpecialMessages(self, broadcasts):
		''' Used to check if special messages like GCDC messages are in the broadcast, then saves them in some dict '''

		for broadcast in broadcasts:
			if 'obstacle' in broadcast:
				self.vehicle.saved_obstacle_messages[broadcast['obstacle_id']] = broadcast
			if 'emergency_vehicle' in broadcast:
				self.vehicle.saved_emergency_vehicle_messages[broadcast['vehicle_id']] = broadcast


	def handleSpecialMessages(self):
		''' Handles special messages like obstacle or emergency vehicle messages, right now used only for GCDC scenarios. Called from step() '''

		## Checking obstacles. Assume that the construction site will send info about when obstacle is cleared, at which point it will be deleted from saved_obstacle_message
		if self.vehicle.saved_obstacle_messages:
			if self.vehicle.platoon_leader:
				self.handleObstacles()

		## Handles emergency vehicles. Does more than just message handling
		self.handleEmergencyVehicles()



	def searchFwdPairPartnerBroadcast(self, broadcasts): ## broadcasts is network or wifi broadcast (list of dictionaries)
		'''
		This function is only called during the first few seconds of the modules runtime, specified by the self.broadcastMergePeriod variable
		It goes through the input broadcast (wifi or network) and compares the relative coordinates of other vehicles, then returns the closest one
		Only chooses a candidate if it has positive relative coords, meaning it is in front of you, and if it is in the same lane as you and doesn't already have a new bwd partner
		'''

		#return None

		candidate = None
		opt_road_coords = None

		for broadcast in broadcasts:
			if not 'smartvehicle' in broadcast:
				continue

			if broadcast['vehicle_id'] != self.vehicle.id:
				road_coords = self.vehicle.perception_module.convertToRoadCoord([broadcast['x'], broadcast['y']], self.vehicle.getCurrentLaneTrajectoryString())
				rel_road_coords = self.vehicle.perception_module.getRelativeRoadCoordsOval(road_coords, self.vehicle.getCurrentLaneTrajectoryString())


				if rel_road_coords[0] > 0 and rel_road_coords[0] < self.vehicle.send_desired_merge_distance and not broadcast['new_bwd_pair_partner'] and broadcast['desired_lane'] == self.vehicle.desired_lane:
					if candidate == None:
						candidate = broadcast['vehicle_id']
						opt_road_coords = rel_road_coords
					elif rel_road_coords[0] < opt_road_coords[0]:
						candidate = broadcast['vehicle_id']
						opt_road_coords = rel_road_coords

		return candidate


	def continuousSearchFwdPairPartner(self):
		''' Called by a new thread through the waitForEvent thread. Calls the searchFwdPairPartner function untill a new pair partner is found '''

		self.vehicle.merge_distance_reached.clear()

		while True: ## True can be exchanged for some abort parameter

			found = self.searchFwdPairPartner(should_print = False)
			if found == 1 or found == -1:
				return
			else:
				time.sleep(0.1)


	def searchFwdPairPartner(self, should_print = True): ## if should_print is checked to False the function will not print if it fails to send a desired merge
		'''
		Tries to find a new fwd pair partner by calling findBestPairPartner
		Returns -1 if you already have a partner, fwd or new
		Returns 1 if a partner was found and desired merge was sent
		Returns 0 if no partner was found
		'''

		if self.vehicle.fwd_pair_partner or self.vehicle.new_fwd_pair_partner:

			if should_print:
				print self.vehicle.id, "already has a fwd pair partner, will not search for new"
			return -1

		best_id = self.findBestPairPartner()

		if best_id != None:
			self.vehicle.messenger.sendDesiredMerge(best_id)
			return 1

		else:
			if should_print:
				print "didn't find someone to merge with"
			return 0


	def findBestPairPartner(self):
		'''
		Goes through all the vehicles in the parts of the perception grid in front of you, and tries to find the closest vehicle relative to you
		Only picks a candidate if relative coords are positive, candidate is a smartvehicle and not oneself. If no candidate is found None is returned.
		'''


		grid_check_list = ['fm','fl','fr','l','r']
		best_id = None

		for grid_part in grid_check_list:
			for grid_part in self.vehicle.perception_grid:
				for other_id in self.vehicle.perception_grid[grid_part]:
					if other_id not in self.vehicle.saved_smartvehicle_ids or other_id == self.vehicle.id:
						continue

					if self.vehicle.perceived_objects[other_id][0] > 0:
						if best_id == None:
							best_id = other_id
						elif self.vehicle.perceived_objects[other_id][0] < self.vehicle.perceived_objects[best_id][0]:
							best_id = other_id

		return best_id



	def alertFollowers(self, new_velocity = None, temporary = None): # temporary may point on id of vehicle that sent the temp velocity
		""" Should alert followers, i.e. the new bwd and bwd partners about velocity change etc. For now only alerts about velocity change and if its a temporary change

		"""

		if new_velocity != None:

			## if there is a platooning vehicle behind
			if self.vehicle.bwd_pair_partner:
				self.vehicle.messenger.sendNewVelocity(self.vehicle.bwd_pair_partner, new_velocity, temporary = temporary)
			## if there is an incoming merging vehicle
			if self.vehicle.new_bwd_pair_partner:
				self.vehicle.messenger.sendNewVelocity(self.vehicle.new_bwd_pair_partner, new_velocity, temporary = temporary)


	def sendEndTempVelocityFwd(self, temporary_id):
		''' Sends end temporary velocity forward in a chain, and passes on the ID of the vehicle that initiated the temporary velocity change '''

		if self.vehicle.fwd_pair_partner:
			self.vehicle.messenger.sendEndTempVelocity(self.vehicle.fwd_pair_partner, temporary_id, fwd = True)
		if self.vehicle.new_fwd_pair_partner:
			self.vehicle.messenger.sendEndTempVelocity(self.vehicle.new_fwd_pair_partner, temporary_id, fwd = True)


	def sendEndTempVelocityBwd(self, temporary_id):
		''' Sends end temporary velocity backwars in a chain, and passes on the ID of the vehicle that initiated the temporary velocity change '''

		if self.vehicle.bwd_pair_partner:
			self.vehicle.messenger.sendEndTempVelocity(self.vehicle.bwd_pair_partner, temporary_id, bwd = True)
		if self.vehicle.new_bwd_pair_partner:
			self.vehicle.messenger.sendEndTempVelocity(self.vehicle.new_bwd_pair_partner, temporary_id, bwd = True)


	def pingPartnerInfo(self, extra_info = {}): ## extra info can be whatever extra info you want to send
		''' Pings useful information about self to bwd and new bwd partner '''

		if self.vehicle.bwd_pair_partner:
			self.vehicle.messenger.sendForwardPartnerInfo(self.vehicle.bwd_pair_partner, extra_info)
			self.ping_time = time.time()
		if self.vehicle.new_bwd_pair_partner:
			self.vehicle.messenger.sendForwardPartnerInfo(self.vehicle.new_bwd_pair_partner, extra_info)
			self.ping_time = time.time()


	def handleFwdPartnerInfo(self, info): ## info is either self.vehicle.forward_partner_info or self.vehicle.new_forward_partner_info
		''' Handles info sent by fwd or new fwd pair partner '''

		other_id = info['vehicle_id']

		## Saves the lane of fwd or new fwd pair partner, which the vehicle tries to keep if it doesn't have any pending lane changes. The variables are reset when a new partner is set
		if self.vehicle.fwd_pair_partner == other_id:
			self.vehicle.fwd_partner_lane = info['lane']
		if self.vehicle.new_fwd_pair_partner == other_id:
			self.vehicle.new_fwd_partner_lane = info['lane']

		## Saves the platoon lane, if you are merging you take it from your new partner, otherwise from your fwd partner. The vehicle tries to keep the platoon lane if it doesn't have any pending lane changes
		if self.vehicle.merging and self.vehicle.new_fwd_pair_partner == other_id and info['platoon_lane'] != None:
			self.vehicle.platoon_lane = info['platoon_lane']
		elif not self.vehicle.merging and self.vehicle.fwd_pair_partner == other_id and info['platoon_lane'] != None:
			self.vehicle.platoon_lane = info['platoon_lane']


		## new_velocity is used to set a new velocity if you are too far away, and to update accc_velocity. current_velocity is taken from the getVelocity() function in the supervisory module
		## If you are at your desired ACCC distance (ACCC_ready is set) or the other vehicle has a much higher desired velocity than your current velocity, you will use the desired_velocity as reference
		if self.vehicle.ACCC_ready.isSet() or abs(info['desired_velocity']-self.getVelocity())>self.getVelocity()*0.25 or other_id>0:
			new_velocity = info['desired_velocity']
		else:
			new_velocity = info['current_velocity']

		## Outside of the catchup_distance the catching_up mode will be activated, within it the ACCC can be set and catching_up deactivated. Max value specified in the checkLimitValues below. Note that it must be smaller than wifi range.
		catchup_distance = 2.5*self.vehicle.merge_distance
		catchup_distance = self.checkLimitValues(catchup_distance, max_value = 30)

		## redundant check really
		if other_id == self.vehicle.fwd_pair_partner or other_id == self.vehicle.new_fwd_pair_partner:

			## catches up with fwd pp if a bunch of emergency flags are not set and you are more than the catchup distance behind your partner.
			if (not other_id in self.vehicle.perceived_objects or (other_id in self.vehicle.perceived_objects and abs(self.vehicle.perceived_objects[other_id][0])>catchup_distance) ) and not self.vehicle.overtaking_vehicle_flag and not self.vehicle.emergency_velocity_flag and not self.vehicle.temporary_velocity_flag:
				if not self.vehicle.merging or other_id == self.vehicle.new_fwd_pair_partner: ## if you are not merging you won't have a new fwd pair partner

					## If other vehicle is also catching up with someone you will not increase your velocity
					if info['catching_up']:
						catchup_factor = 1
					else:
						catchup_factor = self.vehicle.catchup_factor ## currently 1.3

					## uses old_velocity for the catchup if it exists, meaning it doesn't care about the other vehicles emergency velocity change when it's catching up
					if info['old_velocity'] != None:
						self.setVelocity(catchup_factor*info['old_velocity'])
					else:
						self.setVelocity(catchup_factor*new_velocity)

					## setting catching_up flag
					self.vehicle.catching_up = True

			## setting accc if other car is visible and closer than the catchup distance, and accc is not set, and not upcoming plat merge or em vel flag. Also resets the catching_up flag
			if other_id in self.vehicle.perceived_objects and not self.vehicle.emergency_velocity_flag:
				if abs(self.vehicle.perceived_objects[other_id][0]) < catchup_distance and not self.vehicle.control_module.ACCC_target_id == other_id:

					## Trying to set ACCC to new fwd pp if in merging process
					if self.vehicle.merging and other_id == self.vehicle.new_fwd_pair_partner:
						self.setACCCTarget(other_id, self.vehicle.merge_distance)

					## if ACCC is not set and you are not merging it will be set to either fwd or new fwd pair partner
					if not self.vehicle.merging and not self.vehicle.control_module.ACCC_target_id:
						self.setACCCTarget(other_id, self.vehicle.merge_distance)

					self.vehicle.catching_up = False

					## ACCC velocity should be set right away with the code below

		## setting accc target velocity and desired velocity
		if self.vehicle.control_module.ACCC_target_id == other_id:

			self.setVelocity(new_velocity, alert = False)
			self.vehicle.control_module.ACCC_target_velocity = new_velocity


	def waitForEvent(self, event, function_to_run, abort_parameter, function_params = (), time_out = None):
		"""
		This function starts a thread that waits on event and then runs function_to_run with arguments function_params.

		:event: Thread waits for this event
		:function_to_run: Function to run when event is triggered
		:abort_parameter: Thread checks this to be False before running function_to_run
		:function_params: tuple or list of the parameters passed to function_to_run
		"""

		temp_thread = threading.Thread(target= self.waitForEventThread, args = (function_to_run, event, abort_parameter, function_params, time_out))
		temp_thread.daemon = True
		temp_thread.start()

	def waitForEventThread(self, function_to_run, event, abort_parameter, function_params, time_out):

		event.wait(time_out)

		if not abort_parameter:# and event.isSet():
			function_to_run(*function_params)


 	## OBSTACLE STUFF

	def checkObstacles(self, broadcasts): ## Broadcasts wifi or networks broadcasts (list of dictionaries)
		''' Goes through input broadcasts and saves brodcast if it is tagged with 'obstacle' '''

		for broadcast in broadcasts:
			if 'obstacle' in broadcast:
				self.vehicle.saved_obstacle_messages[broadcast['obstacle_id']] = broadcast


	def handleObstacles(self):
		'''
		Called from step(). Goes through the saved obstacle broadcasts and performs necessary operations
		Right now it starts an emergency merge if the obstacle is in the same lane as you and in front of you, and you are not merging and platoon leader or not platooning
		It also sets a restricted velocity if within the restricted zone, to keep with the GCDC specs. Note that the restricted velocity is kept until new velocity is sent
		'''

		for obstacle_id in self.vehicle.saved_obstacle_messages:
			message = self.vehicle.saved_obstacle_messages[obstacle_id]
			other_coords = [message['x'], message['y']]
			road_coords = self.vehicle.perception_module.convertToRoadCoord(other_coords, self.vehicle.desired_lane)
			rel_coords = self.vehicle.perception_module.getRelativeRoadCoordsOval(road_coords, self.vehicle.desired_lane)
			if message['lane'] == self.vehicle.desired_lane and rel_coords[0]>0:
				if (self.vehicle.platoon_leader or not self.isPlatooning()) and not self.vehicle.merging and not self.vehicle.emergency_merge_initiation:

					print self.vehicle.id,"STARTING EMERGENCY MERGE"

					self.vehicle.emergency_merge_initiation = True ## makes sure this is not performed more than once. Is reset when desired merge protocol is done or if merge initiation is aborted

					self.vehicle.abort_merge_start = False ## abort parameter for thread below
					## Starting a thread that waits for vehicle to count the vehicles in the platoon, then calls self.startEmergencyMerge(). If more than 2 seconds pass the function is called regardless
					self.waitForEvent(self.vehicle.platoon_length_count_completed, self.startEmergencyMerge, self.vehicle.abort_merge_start, time_out = 2)

					## Begins the platoon length count
					self.forwardRequestPlatoonLengthInfo(True, self.vehicle.id)

			## sets restricted velocity, only done by platoon leader or non platooning vehicles
			if 'restricted_velocity' in message and (self.vehicle.platoon_leader or not self.isPlatooning()) and rel_coords[0]<0 and abs(rel_coords[0])<message['restriction_length']:
				self.setVelocity(message['restricted_velocity'])
				self.setVelocity(message['restricted_velocity'])
				self.setVelocity(message['restricted_velocity'])


	def handleCollisionAvoidance(self):
		'''
		Calls self.headOnCollisionAvoidance() with specified distance references and chosen ignored objects, which depend on whether or not you are merging
		The getter functions return a distance based on your velocity, with a minimum threshold
		'''

		if not self.vehicle.merging:
			self.headOnCollisionAvoidance(self.getEmergencyVelocityDistance(), ignored_objects = self.vehicle.saved_smartvehicle_ids)

		if self.vehicle.merging:
			self.headOnCollisionAvoidance(self.getEmergencyVelocityDistancePlatoonMerge(), ignored_objects = self.vehicle.saved_smartvehicle_ids)

		## head on collision avoidance for other smartvehicles, with much lower emergency velocity distance. Comment if it causes trouble
		# self.headOnCollisionAvoidance(self.getEmergencyVelocityDistance()*0.5, target_objects = self.vehicle.saved_smartvehicle_ids)



	def headOnCollisionAvoidance(self, emergency_distance, emergency_brake_distance = None, ignored_objects = [], target_objects = []): # ignored_objects/target_objects can be set, list or dict
		'''
		Checks if there is someone in front of you who is in target objects not in ignored objects, and if it's too close do either emergency brake or set an emergency velocity
		Only input either target objects or ignored objects, not both at the same time. Call the function twice otherwise.
		When an emergency velocity or brake is set other vehicles connected to self through platoon are alerted with a velocity change tagged with 'temporary' set to selfs id,
			and only self can end the velocity change by doing sendEndTempVelocity, which is called in self.collisionAverted()
		The velocity the vehicle had before the emergency velocity changed is saved in a variable called old_velocity, in smartvehicle.
		An emergency velocity flag is set if emergency velocity change or brake occurs
		Once nothing is too close self.collisionAverted() is called and vehicle reverts to old velocity and resets necessary variables

		'''


		if 'fm' in self.vehicle.closest_objects:
			other_id = self.vehicle.closest_objects['fm']
			if (ignored_objects and other_id not in ignored_objects) or (target_objects and other_id in target_objects):

				## Getting default emergency brake distance if no input was given
				if emergency_brake_distance == None:
					emergency_brake_distance = self.getEmergencyBrakeDistance()

				## Doing emergency brake if other vehicle is too close, resetting ACCC and alerting followers (tagging velocity as temporary). Doesn't call setVelocity because it needs to override the emergency/temporary velocity flags
				if self.vehicle.perceived_objects[other_id][0]<emergency_brake_distance:
					if self.vehicle.old_velocity == None:
						self.vehicle.old_velocity = self.vehicle.desired_velocity
					self.vehicle.desired_velocity = 0 # needs to override the emergency_velocity_flag
					self.alertFollowers(new_velocity = 0, temporary = self.vehicle.id)
					self.vehicle.emergency_velocity_flag = True
					if self.vehicle.control_module.ACCC_target_id != None:
						# print self.vehicle.id,"RESETTING ACCC"
						self.resetACCC()
					# print "EMERGENCY BRAKE FOR VEHICLE", self.vehicle.id

				## Does an emergency velocity change if the other vehicle is sufficiently close, resets ACCC (if it is not set to other_id), sets velocity to other velocity*0.9 and alerts followers (tagging velocity as temporary). Doesn't call setVelocity for same reason as above
				elif self.vehicle.perceived_objects[other_id][0]<emergency_distance:
					if self.vehicle.old_velocity == None:
						self.vehicle.old_velocity = self.vehicle.desired_velocity

					## Adjusting to other velocity
					other_velocity = self.vehicle.perceived_objects[other_id][2]
					self.vehicle.desired_velocity = other_velocity*0.9 ## lower so that overtaking can occur, also to make sure you don't crash

					## To make sure you don't stop completely if other vehicle has 0 velocity. Overtaking and emergency brake should save you from death.
					if self.vehicle.desired_velocity<3:
						#self.setVelocity(0)
						self.vehicle.desired_velocity = 3

					# print "EMERGENCY VELOCITY CHANGE FOR VEHICLE", self.vehicle.id, "SETTING VELOCITY:",self.vehicle.desired_velocity
					self.alertFollowers(self.vehicle.desired_velocity, temporary = self.vehicle.id)
					self.vehicle.emergency_velocity_flag = True
					if self.vehicle.control_module.ACCC_target_id != other_id:
						# print self.vehicle.id,"RESETTING ACCC"
						self.resetACCC()

				## no more emergency if sufficiently far away, or if it is fwd_pp and kinda far away. Also different rules for fwd/new fwd partner to work with merge.
				elif self.vehicle.perceived_objects[other_id][0]>emergency_distance*1.2 or ((other_id == self.vehicle.fwd_pair_partner or other_id == self.vehicle.new_fwd_pair_partner) and self.vehicle.perceived_objects[other_id][0]>emergency_brake_distance):
					self.collisionAverted()

		else:
			## no more emergency if no vehicles in 'fm'
			self.collisionAverted()



	def collisionAverted(self):
		''' Resets necessary variables for the headOnCollisionAvoidance(), and reverts to old velocity '''

		if self.vehicle.emergency_velocity_flag:
			# print "COLLISION AVERTED FOR VEHICLE", self.vehicle.id, "RETURNING TO VELOCITY", self.vehicle.old_velocity
			self.vehicle.emergency_velocity_flag = False
			self.sendEndTempVelocityFwd(self.vehicle.id)
			self.sendEndTempVelocityBwd(self.vehicle.id)
			self.setVelocity(self.vehicle.old_velocity)
			self.vehicle.old_velocity = None


	''' Lane change functions '''

	def handleLaneChanging(self):
		''' Handles lane changing. Lanes are put in a queue (actually just an array), which this function pops from. Checks if lane is safe and performs change. Also checks fwd/new fwd partner lane and platoon lane. '''
		## It would be nice to use a python deque instead of array for new_desired_lanes, but really doesn't matter since it doesn't get much larger than 5 elements or so.


		## Cleaning up new_desired_lanes, removing the first element in the queue if it is the lane you are currently in.
		if self.vehicle.new_desired_lanes:
			while self.vehicle.new_desired_lanes[0] == self.vehicle.desired_lane:
				self.vehicle.new_desired_lanes.pop(0)
				if not self.vehicle.new_desired_lanes:
					break

		## Do a lane change if you are not currently changing lane (meaning lanekeeping_ready is set)
		if self.vehicle.lanekeeping_ready.isSet():

			## If there are pending lane changes, try to change to that lane. If it is not safe, safeLaneChange() will put it on top of the queue again (so not last)
			if self.vehicle.new_desired_lanes:
				self.safeLaneChange(self.vehicle.new_desired_lanes.pop(0))
				return


			## Changes to either platoon_lane, fwd partner lane or new fwd partner lane if not currently catching up and no pending lane changes in the queue
			if not self.vehicle.catching_up:
				## if no pending lane changes and not in platoon lane and all is clear, change to that lane. Prioritizes platoon lane over fwd/new fwd partner lane. This is only done if vehicle is merging or platooned, and ACCC is set to fwd or new fwd partner
				if self.vehicle.platoon_lane != None and (self.vehicle.platooned_vehicle or self.vehicle.merging):
					if self.vehicle.platoon_lane != self.vehicle.desired_lane and not self.vehicle.overtaking_vehicle_flag and (self.vehicle.control_module.ACCC_target_id == self.vehicle.fwd_pair_partner or self.vehicle.control_module.ACCC_target_id == self.vehicle.new_fwd_pair_partner):
						self.safeLaneChange(self.vehicle.platoon_lane , fwd_pp_change = True)
					return

				## if no pending lane changes and not in fwd partners lane and all is clear, change to that lane. Prioritizes fwd pp over new fwd pp
				elif self.vehicle.fwd_partner_lane != None and self.vehicle.fwd_pair_partner != None:
					if self.vehicle.fwd_partner_lane != self.vehicle.desired_lane and not self.vehicle.overtaking_vehicle_flag and self.vehicle.control_module.ACCC_target_id == self.vehicle.fwd_pair_partner:# and not self.vehicle.merging and self.vehicle.platooned_vehicle:
						self.safeLaneChange(self.vehicle.fwd_partner_lane, fwd_pp_change = True)
					return

				## if no pending lane changes and not in NEW fwd partners lane and all is clear, change to that lane.
				elif self.vehicle.new_fwd_partner_lane != None and self.vehicle.new_fwd_pair_partner != None:
					if self.vehicle.new_fwd_partner_lane != self.vehicle.desired_lane and not self.vehicle.overtaking_vehicle_flag and self.vehicle.control_module.ACCC_target_id == self.vehicle.new_fwd_pair_partner:# and not self.vehicle.merging and self.vehicle.platooned_vehicle:
						self.safeLaneChange(self.vehicle.new_fwd_partner_lane, fwd_pp_change = True)
					return



	def getNewLaneLeft(self, input_lane):
		''' Returns lane LEFT of input lane, if input is left lane returns center lane '''

		if input_lane == self.RIGHT_LANE_STRING:
			return self.CENTER_LANE_STRING
		if input_lane == self.CENTER_LANE_STRING:
			return self.LEFT_LANE_STRING
		if input_lane == self.LEFT_LANE_STRING:
			return self.CENTER_LANE_STRING

	def getNewLaneRight(self, input_lane):
		''' Returns lane RIGHT of input lane, if input is right lane returns center lane '''

		if input_lane == self.RIGHT_LANE_STRING:
			return self.CENTER_LANE_STRING
		if input_lane == self.CENTER_LANE_STRING:
			return self.RIGHT_LANE_STRING
		if input_lane == self.LEFT_LANE_STRING:
			return self.CENTER_LANE_STRING



	def addLaneChange(self, lane):
		''' This is the function you should use to do a lane change. Adds it to the queue if it is not the same lane as the one last in the queue '''

		if self.vehicle.new_desired_lanes:
			if not self.vehicle.new_desired_lanes[-1] == lane:
				self.vehicle.new_desired_lanes.append(lane)
			else:
				return
		else:
			self.vehicle.new_desired_lanes.append(lane)


	def safeLaneChange(self, lane, fwd_pp_change = False): ## 'lane' is lane you want to change to
		'''
		This function does the actual lane change, but first checks if it is safe to do so by calling self.checkLaneSafety(). If not safe puts lane back in front of the queue
		If fwd_pp_change is checked to True it means you're changing to fwd/new fwd partner lane or platoon lane, and you should not put it in the queue.
		When calling the checkLaneSafety() it is specified if you're changing to the left or right, and how many lane changes you are performing (for instance, left -> right means 2 changes)
		'''
		if self.vehicle.desired_lane != lane:

			## this stuff could maybe be checked in a more neat way. Right now just a bunch of ifs
			if self.vehicle.desired_lane == self.CENTER_LANE_STRING:
				if lane == self.LEFT_LANE_STRING:
					safe = self.checkLaneSafety(1, left_change = True)
				elif lane == self.RIGHT_LANE_STRING:
					safe = self.checkLaneSafety(1, right_change = True)

			elif self.vehicle.desired_lane == self.LEFT_LANE_STRING:
				if lane == self.CENTER_LANE_STRING:
					safe = self.checkLaneSafety(1, right_change = True)
				elif lane == self.RIGHT_LANE_STRING:
					safe = self.checkLaneSafety(2, right_change = True)

			elif self.vehicle.desired_lane == self.RIGHT_LANE_STRING:
				if lane == self.CENTER_LANE_STRING:
					safe = self.checkLaneSafety(1, left_change = True)
				elif lane == self.LEFT_LANE_STRING:
					safe = self.checkLaneSafety(2, left_change = True)

			if safe:
				self.vehicle.desired_lane = lane
				self.vehicle.lanekeeping_ready.clear()


			if not safe and not fwd_pp_change:
				## put lane back in queue, in first place
				#print "Not currently safe to switch lanes"
				self.vehicle.new_desired_lanes.insert(0,lane)

	def checkLaneSafety(self, number_lane_changes, left_change = False, right_change = False):
		'''
		Checks if it is safe to change to the right or left, depending on how many lane changes you are performing
		Goes through perception grid and checks if other vehicles are too close to the side. Takes into account what lanes the other vehicles are in.
		When checking front side fwd/new fwd partners are ignored if you're not merging, when checking rear side bwd/new bwd partners are ignored if you're not merging
		Returns True if safe, False if not.
		NOTE: If lane width is changed, the variable lane_width_max needs to be changed here. Otherwise it might think it's safe/unsafe in a lane because it thinks it's width is different from real value
		'''

		lane_width_max = 5 ## Used as reference to figure out what lanes other vehicles are in. It should be slightly higher than the actual width of the lane to account for errors
		## With lane width ~ 4 a vehicle one lane away will have relative y-coord ~ 4, two lanes away rel y-coord 8

		## Specifying keys for the perception_grid
		if left_change:
			side = 'l'
			rear_side = 'rl'
			front_side = 'fl'
			print_str = "left"
		if right_change:
			side = 'r'
			rear_side = 'rr'
			front_side = 'fr'
			print_str = "right"


		## checking side
		for other_id in self.vehicle.perception_grid[side]:
			## check if vehicle is in first side lane, or second side lane and doing 2 lane changes
			if abs(self.vehicle.perceived_objects[other_id][1])<lane_width_max or (lane_width_max<abs(self.vehicle.perceived_objects[other_id][1])<2*lane_width_max and number_lane_changes == 2):
				## check if rel x-coord is less than 2*selfs length
				if abs(self.vehicle.perceived_objects[other_id][0])<self.vehicle.length*2: # redundant safety margin
					#print "For vehicle", self.vehicle.id,":"+"vehicle to the "+print_str+" side too close to switch lane"
				 	return False


		## Checking rear side
		for other_id in self.vehicle.perception_grid[rear_side]:
			if (other_id != self.vehicle.bwd_pair_partner and other_id != self.vehicle.new_bwd_pair_partner) or self.vehicle.merging:
				## check if vehicle is in first side lane, or second side lane and doing 2 lane changes
				if abs(self.vehicle.perceived_objects[other_id][1])<lane_width_max or (lane_width_max<abs(self.vehicle.perceived_objects[other_id][1])<2*lane_width_max and number_lane_changes == 2):
					## check if vehicle is less than some distance behind. Distance depends on if other_id is a smartvehicle
					if abs(self.vehicle.perceived_objects[other_id][0]) < self.getRearLCSafeDist(smartvehicle = other_id in self.vehicle.saved_smartvehicle_ids):
						#print "For vehicle", self.vehicle.id,":"+"vehicle in the rear "+print_str+" too close, cannot switch lane"
						return False
						'''CODE BELOW TAKES VELOCITY OF OTHER VEHICLE INTO CONSIDERATION, UNCOMMENT IF DESIRED'''
						# other_velocity = self.vehicle.perceived_objects[other_id][2]
						# if other_velocity > self.getVelocity():
						# 	print "For vehicle", self.vehicle.id,":"+"vehicle less than 8 meters in rear "+print_str+" moving too fast to switch lane"
						# 	return False



		## Checking front side
		for other_id in self.vehicle.perception_grid[front_side]:
			if (other_id != self.vehicle.fwd_pair_partner and other_id != self.vehicle.new_fwd_pair_partner) or self.vehicle.merging:
				## check if vehicle is in first side lane, or second side lane and doing 2 lane changes
				#print other_id, "in front side has rel coords (x,y):", abs(self.vehicle.perceived_objects[other_id][0]), str(abs(self.vehicle.perceived_objects[other_id][1]))
				if abs(self.vehicle.perceived_objects[other_id][1])<lane_width_max or (lane_width_max<abs(self.vehicle.perceived_objects[other_id][1])<2*lane_width_max and number_lane_changes == 2):
					## check if vehicle is less than some distance ahead. Distance depends on if other_id is a smartvehicle
					if abs(self.vehicle.perceived_objects[other_id][0]) < self.getFrontLCSafeDist(smartvehicle = other_id in self.vehicle.saved_smartvehicle_ids):
						#print "For vehicle", self.vehicle.id,":"+"vehicle in the front "+print_str+" too close, cannot switch lane"
						return False
						'''CODE BELOW TAKES VELOCITY OF OTHER VEHICLE INTO CONSIDERATION, UNCOMMENT IF DESIRED'''
						# other_velocity = self.vehicle.perceived_objects[other_id][2]
						# if other_velocity < self.getVelocity():
						# 	print "For vehicle", self.vehicle.id,":"+"vehicle in the front "+print_str+" too close and moving too slowly to switch lane"
						# 	return False


		return True

	''' End of lane change functions '''


	''' Overtaking functions '''

	def handleOvertaking(self):
		''' Calls either self.overtakeVehicle() or self.overtakeCheck() '''

		## Only overtakes if you are not in middle of lane change
		if self.vehicle.lanekeeping_ready.isSet():# and not self.vehicle.merging:
			if self.vehicle.overtaking_vehicle_flag:
				self.overtakeVehicle()
			else:
				self.overtakeCheck() ## is also called in the overtakeVehicle() function


	def overtakeCheck(self):
		''' Checks if it is necessary to overtake a vehicle '''


		if 'fm' in self.vehicle.closest_objects:
			other_id = self.vehicle.closest_objects['fm']
			## Doesn't overtake fwd pair partner or new fwd pair partner
			if not self.vehicle.fwd_pair_partner == other_id and not self.vehicle.new_fwd_pair_partner == other_id:

				## If other vehicle is too close, overtake it. distance returned by getOvertakeDistance() depends on current velocity
				if self.vehicle.perceived_objects[other_id][0]<self.getOvertakeDistance():

					# Tries to find the velocity of the other vehicle through sensor readings, aborts overtaking if it doesn't find the other vehicle in the readings
					other_velocity = self.vehicle.perceived_objects[other_id][2]

					if other_velocity != None:

						## If other vehicle is moving much slower than self it doesn't overtake, also compares to old_velocity (which it might revert to)
						if other_velocity < self.getVelocity()*1.2 or (self.vehicle.old_velocity and other_velocity < self.vehicle.old_velocity):

							## Saves velocity you had before overtaking in a stack (actually an array) which is popped from when overtaking is done
							if self.vehicle.old_velocity != None:
								self.vehicle.overtaking_original_velocities.append(self.vehicle.old_velocity)
							else:
								self.vehicle.overtaking_original_velocities.append(self.vehicle.desired_velocity)

							## Increases selfs velocity if you're not moving much faster than other vehicle
							if 1.4*other_velocity>self.getVelocity():
								self.setVelocity(1.4*other_velocity)

							self.vehicle.overtaking_vehicle_flag = True
							self.vehicle.overtaking_original_lanes.append(self.vehicle.desired_lane) ## Saves lane you were in before overtaking in a stack (actually an array)
							self.vehicle.overtaken_vehicles.append(other_id) ## Saves vehicle to be overtaken in a stack (actually an array)
							print self.vehicle.id,"trying to overtake", other_id
							self.overtakeVehicle() ## Perform overtaking
						else:
							return
					else:
						print "Couldnt determine velocity of vehicle in front, overtaking aborted"
						return



	def overtakeVehicle(self):
		'''
		Performs overtaking, uses stacks that keep track of original lane, velocity and vehicle to be overtaken, in order to be able to overtake multiple lanes recursively
		Uses checkLaneSafety before declaring overtaking to be finished. Calls self.finishOvertaking() when overtaking is done, which resets variabels and reverts to lane and velocity you had before overtaking
		Always tries to overtake by switching to left lane. If vehicle is already in left lane it just switches to right lane.
		Overtaking can be aborted if it is safe to the right, since the overtake_distance is usually greater than the safety margin for the lane change
		'''

		original_lane = self.vehicle.overtaking_original_lanes[-1]
		overtaken_vehicle = self.vehicle.overtaken_vehicles[-1]
		original_velocity = self.vehicle.overtaking_original_velocities[-1]

		## check below if there is a new vehicle to be overtaken during another overtaking, overtake recursively
		if 'fm' in self.vehicle.closest_objects:
		 	if not self.vehicle.closest_objects['fm'] in self.vehicle.overtaken_vehicles:
		 		safe = self.checkLaneSafety(1, right_change = True)
				if not safe:
		 			self.overtakeCheck()


		'''Solution below doesn't overtake left lane, just switches lane'''
		if original_lane != self.LEFT_LANE_STRING:
			## check if overtaken_vehicle is behind you, or simply out of sight. keep in mind that overtaken_vehicle might switch lanes.
			if overtaken_vehicle in self.vehicle.perception_grid['rl'] or overtaken_vehicle in self.vehicle.perception_grid['rr'] or overtaken_vehicle in self.vehicle.perception_grid['rm'] or not overtaken_vehicle in self.vehicle.perceived_objects:
				## overtaking finished
				safe_finish = self.checkLaneSafety(1, right_change = True) ## redundancy in safety is always nice. but seriously, this check is useful, do not remove.
				if safe_finish:
					self.finishOvertaking(original_lane, overtaken_vehicle, original_velocity)
					return

			## overtaking also finished if overtaken_vehicle is sufficiently far away
			if overtaken_vehicle in self.vehicle.perceived_objects:
				if self.vehicle.perceived_objects[overtaken_vehicle][0] > 2*self.getFrontLCSafeDist():
					safe_finish = self.checkLaneSafety(1, right_change = True) ## redundancy in safety is always nice. but seriously, this check is useful, do not remove.
					if safe_finish:
						self.finishOvertaking(original_lane, overtaken_vehicle, original_velocity)
						return

			## Do overtaking or abort it
			if overtaken_vehicle in self.vehicle.perception_grid['fm']:
				## aborting overtaking if clear to the right:
				if original_lane != self.RIGHT_LANE_STRING:
					safe_abort = self.checkLaneSafety(1, right_change = True)
					if safe_abort:
						print "ABORTING OVERTAKING OF", overtaken_vehicle
						new_lane_abort = self.getNewLaneRight(original_lane)
						self.finishOvertaking(new_lane_abort, overtaken_vehicle, original_velocity)
						return

				## overtaking is performed
				safe_overtake = self.checkLaneSafety(1, left_change = True)
				## this check is necessary so that it doesn't append a bunch of lane changes before it knows if it can actually just abort
				if safe_overtake:
					new_lane = self.getNewLaneLeft(original_lane)
					self.addLaneChange(new_lane)
					return

		## doesn't overtake on the inside, just switches lane
		if original_lane == self.LEFT_LANE_STRING:
			safe = self.checkLaneSafety(1, right_change = True)
			if safe:
				self.finishOvertaking(self.CENTER_LANE_STRING, overtaken_vehicle, original_velocity)
				return


	def finishOvertaking(self, new_lane, overtaken_vehicle, new_velocity):
		''' Called when overtaking is ready to be finished. Switches to original lane and velocity, and resets necessary variables '''

		print "OVERTAKING OF", overtaken_vehicle, "FINISHED, RETURNING TO VELOCITY", new_velocity
		self.addLaneChange(new_lane)
		self.vehicle.overtaken_vehicles.pop()
		self.vehicle.overtaking_original_lanes.pop()
		self.vehicle.overtaking_original_velocities.pop()
		self.setVelocity(new_velocity)
		if len(self.vehicle.overtaken_vehicles) == 0:
			self.vehicle.overtaking_vehicle_flag = False

	''' End of overtaking fncs '''


	''' Functions that update safety parameters '''

	def checkLimitValues(self, parameter, min_value = None, max_value = None):
		''' Returns minimum value if parameter is below it, return maximum value if parameter is above it, else return parameter unchanged '''

		if min_value != None and parameter < min_value:
			return min_value
		if max_value != None and parameter > max_value:
			return max_value
		else:
			return parameter


	def getEmergencyBrakeDistance(self):
		''' Returns the emergency brake distance, based on velocity '''

		## good value at 13.888 m/s: 5
		min_val = self.min_emergency_brake_distance
		pow_factor = 1.5
		mult_factor = 0.08333
		emergency_brake_distance = math.pow(self.getVelocity(),pow_factor)*mult_factor
		emergency_brake_distance = self.checkLimitValues(emergency_brake_distance, min_value = min_val)

		return emergency_brake_distance


	def getEmergencyVelocityDistance(self):
		''' Returns the emergency velocity distance for normal circumstances, based on velocity '''

		## good value at 13.888 m/s: 15
		min_val = self.min_emergency_velocity_distance
		pow_factor = 1.5
		mult_factor = 0.29
		emergency_velocity_distance = math.pow(self.getVelocity(),pow_factor)*mult_factor
		emergency_velocity_distance = self.checkLimitValues(emergency_velocity_distance, min_value = min_val)

		return emergency_velocity_distance


	def getEmergencyVelocityDistancePlatoonMerge(self):
		''' Returns the distance at which emergency velocity occurs during platoon merge, based on velocity '''

		## good value at 13.888 m/s: 30
		min_val = self.min_emergency_velocity_distance_platoon_merge
		pow_factor = 1.5
		mult_factor = 0.58
		emergency_velocity_distance_platoon_merge = math.pow(self.getVelocity(),pow_factor)*mult_factor
		emergency_velocity_distance_platoon_merge = self.checkLimitValues(emergency_velocity_distance_platoon_merge, min_value = min_val)

		return emergency_velocity_distance_platoon_merge


	def getOvertakeDistance(self):
		''' Returns the distance at which overtaking is initiated, based on velocity '''

		## good value at 13.888 m/s: 18
		min_val = self.min_overtake_distance
		pow_factor = 1.5
		mult_factor = 0.35
		overtake_distance = math.pow(self.getVelocity(),pow_factor)*mult_factor
		overtake_distance = self.checkLimitValues(overtake_distance, min_value = min_val)

		return overtake_distance


	def getFrontLCSafeDist(self, smartvehicle = False):
		''' Returns the safety margin for front side for lane changing, based on velocity. If smartvehicle is checked to True a lower value is returned, to make sure merge happens smoothly '''

		if self.vehicle.merging and smartvehicle == True:
			return self.vehicle.merge_distance*0.75

		## good value at 13.888 m/s: 14
		min_val = self.min_front_lc_safe_dist
		pow_factor = 1.5
		mult_factor = 0.27
		front_lc_safe_dist = math.pow(self.getVelocity(),pow_factor)*mult_factor
		front_lc_safe_dist = self.checkLimitValues(front_lc_safe_dist, min_value = min_val)

		return front_lc_safe_dist


	def getRearLCSafeDist(self, smartvehicle = False):
		''' Returns the safety margin for rear side for lane changing, based on velocity. If smartvehicle is checked to True a lower value is returned, to make sure merge happens smoothly '''

		if self.vehicle.merging and smartvehicle == True:
			return self.vehicle.merge_distance*0.75
		## good value at 13.888 m/s: 12
		min_val = self.min_rear_lc_safe_dist
		pow_factor = 1.5
		mult_factor = 0.23
		rear_lc_safe_dist = math.pow(self.getVelocity(),pow_factor)*mult_factor
		rear_lc_safe_dist = self.checkLimitValues(rear_lc_safe_dist, min_value = min_val)

		return rear_lc_safe_dist


	def getMergeDistance(self):
		''' Returns the merge distance, based on velocity '''

		# NOTE: This function is not currently used. Some problems:
		#	- the ACCC_desired_distance variable that control_module uses is not updated regularly
		#	- merge_distance is used as a distance reference for comparisons at certain places,
		#	  at those points it would be nice to be able to take the value directly from the variable in smartvehicle

		# good value at 13.888 m/s: 12
		min_val = 12.
		pow_factor = 1.5
		mult_factor = 0.23
		merge_distance = math.pow(self.getVelocity(),pow_factor)*mult_factor
		merge_distance = self.checkLimitValues(merge_distance, min_value = min_val)

		self.vehicle.merge_distance = merge_distance
		#return merge_distance

	''' End of functions that update safety parameters '''

	def setPairPartner(self, target, fwd = False, bwd = False, new = False):
		''' Use this function to set a pair partner. Specify if it's fwd or bwd, and if new. It resets necessary variables like fwd partner info and lane'''

		if fwd:
			self.resetFwdPPInfo(new)
			if new:
				self.vehicle.new_fwd_pair_partner = target
			else:
				self.vehicle.fwd_pair_partner = target
		elif bwd:
			self.resetBwdPPInfo(new)
			if new:
				self.vehicle.new_bwd_pair_partner = target
			else:
				self.vehicle.bwd_pair_partner = target


	def resetFwdPPInfo(self, new):
		''' Resets fwd/new fwd partner info. Called from self.setPairPartner() '''

		if new:
			self.vehicle.new_forward_partner_info = {}
			self.vehicle.new_fwd_partner_lane = None
			self.vehicle.new_fwd_pair_partner = None

		else:
			self.vehicle.forward_partner_info = {}
			self.vehicle.fwd_partner_lane = None
			self.vehicle.fwd_pair_partner = None

	def resetBwdPPInfo(self, new):
		''' Resets bwd/new bwd partner info. Called from self.setPairPartner() '''

		if new:
			self.vehicle.new_bwd_pair_partner = None
		else:
			self.vehicle.bwd_pair_partner = None



	''' Merging functions '''


	def forwardRequestPlatoonLengthInfo(self, in_platoon_info, leader_id):
		'''
		Sends info to leader about whether or not you're in their platoon, and tells bwd/new bwd to send the same info to leader.
		If no bwd and no new bwd send a message to leader that count is completed
		'''


		self.vehicle.messenger.sendPlatoonLengthInfo(leader_id, in_platoon_info)

		if self.vehicle.bwd_pair_partner or self.vehicle.new_bwd_pair_partner:

			if self.vehicle.bwd_pair_partner:

				self.vehicle.messenger.sendRequestPlatoonLengthInfo(self.vehicle.bwd_pair_partner, leader_id)

			if self.vehicle.new_bwd_pair_partner:

				self.vehicle.messenger.sendRequestPlatoonLengthInfo(self.vehicle.new_bwd_pair_partner, leader_id)

		else:
			self.vehicle.messenger.sendPlatoonLengthCountCompleted(leader_id, True)


	def abortMergeStart(self):
		''' Aborts emergency merge '''

		self.vehicle.abort_merge_start = True
		self.vehicle.emergency_merge_initiation = False


	def startEmergencyMerge(self):
		'''
		Starts emergency merge, called from a thread started in self.handleObstacles(), when the event self.vehicle.platoon_length_count_completed is set, which happens when leader receives a message that platoon length count is completed.
		This function is (or should be) also called if it takes longer than 2 seconds to count platoon vehicles. The count usually takes a very small fraction of a second.
		The function starts a thread that waits for a minimum distance at which the merge should occur, which then starts searching for a merge partner.
		It calculates a minimum distance from the obstacle at which a merge should occur (based on platoon length and own velocity), and then checks if that distance has been reached at which point the event is set.

		'''


		print self.vehicle.id, "waiting to reach platoon merge distance"
		self.vehicle.abort_fwd_pp_search = False ## Abort parameter for the self.vehicle.merge_distance_reached event below
		self.vehicle.platoon_length_count_completed.clear() ## clears event that calls this function
		self.vehicle.merge_distance_reached.clear() ## Clears event for below thread in case it is already set (can happen during aborting of emergency merge)
		self.waitForEvent(self.vehicle.merge_distance_reached, self.continuousSearchFwdPairPartner, self.vehicle.abort_fwd_pp_search) ## Starts the thread that waits for the distance to be reached

		while True:

			## If you have an emergency velocity flag the velocity factor depends on old_velocity, otherwise current velocity
			if self.vehicle.emergency_velocity_flag:
				velocity_factor = math.pow(self.vehicle.old_velocity,1)
			else:
				velocity_factor = math.pow(self.getVelocity(),1)

			## If something went wrong with platoon count, set the count to 1 so that the merge_distance is not 0
			if len(self.vehicle.counted_platoon_vehicles) == 0:
				counted_platoon_vehicles = 1
			else:
				counted_platoon_vehicles = len(self.vehicle.counted_platoon_vehicles) ## self.vehicle.counted_platoon_vehicles is a list of ids of vehicles in selfs platoon

			merge_distance = velocity_factor*math.pow(float(counted_platoon_vehicles),0.8)*self.platoon_merge_distance_factor ## The minimum distance at which merge should be initiated

			## Goes through obstacle messages to see if you're within the merge_distance. If there are no obstacle messages the merge is aborted
			if self.vehicle.saved_obstacle_messages:
				for obstacle_id in self.vehicle.saved_obstacle_messages:
					message = self.vehicle.saved_obstacle_messages[obstacle_id]
					other_coords = [message['x'], message['y']]
					road_coords = self.vehicle.perception_module.convertToRoadCoord(other_coords, self.vehicle.desired_lane)
					rel_coords = self.vehicle.perception_module.getRelativeRoadCoordsOval(road_coords, self.vehicle.desired_lane)

					## If you are in the same lane and the relative coordinates are positive (obstacle is in front of you) and less than merge_distance, start the fwd partner search
					if message['lane'] == self.vehicle.desired_lane and rel_coords[0]>0 and rel_coords[0]<merge_distance:
						print str(self.vehicle.id)+" searching for someone to merge with at distance: "+str(merge_distance)
						self.vehicle.counted_platoon_vehicles = []
						self.vehicle.merge_distance_reached.set()
						return

					else:
						time.sleep(0.05)

			## Aborting merge and resetting variables/clearing events
			else:
				print "No more obstacles, aborting merge"
				self.vehicle.platoon_length_count_completed.clear()
				self.vehicle.counted_platoon_vehicles = []
				self.vehicle.abort_fwd_pp_search = True
				self.vehicle.merge_distance_reached.set() # This is necessary so that the waitForEventThread function runs properly, but doesn't run the continuousSearchFwdPairPartner() function since the abort parameter is set to False
				self.vehicle.emergency_merge_initiation = False

				return


	def mergeDone(self):
		'''
		This function is called by vehicles are trying to merge when their ACCC distance is reached (ACCC_ready is set) from another thread that is started in the 'desired merge' protocol
		It checks if vehicle is in it's new fwd partner lane, and if so finishes the merge by sending 'merge done' to new fwd partner, at which point correct flags/variables are set/reset
		'''

		while True:

			if self.vehicle.new_fwd_partner_lane == self.vehicle.desired_lane:

				self.vehicle.merging = False

				# Only sent forward
				if self.vehicle.new_fwd_pair_partner != None:

					self.vehicle.messenger.sendMergeDone(self.vehicle.new_fwd_pair_partner)

				break

			else:
				time.sleep(0.1)
				self.vehicle.ACCC_ready.wait()


	def leavePlatoon(self, new_velocity = None, new_lane = None):
		'''
		This function is used to leave a platoon. A new velocity and a new lane can be input to be set after the vehicle has left the platoon
		If this function is called when a vehicle is trying to merge with another vehicle, it will tell it's bwd/new bwd to merge with its new fwd partner instead and still leave the platoon, and abort the merge
		Uses the notifyNewPairPartner protocol and the leavePlatoon protocol to communicate with pair partners about leaving the platoon and pairing the new partners with each other
		'''

		## If you are not a platooned vehicle and not merging you cannot leave a platoon/abort merge.
		if not self.vehicle.platooned_vehicle and not self.vehicle.merging:
			print self.vehicle.id, "is not in a platoon or not merging, cannot leave platoon or abort merge"
			return

		## Pairing bwd partner with new fwd or fwd if they exist.
		if self.vehicle.bwd_pair_partner != None:

			## Telling bwd pair partner to set None as fwd pair partner
			self.vehicle.messenger.notifyNewPairPartner(self.vehicle.bwd_pair_partner, None, fwd = True)

			## Informing fwd/new fwd partner about leaving platoon, and if possible pairing fwd or new fwd pair partner with old bwd pair partner
			if self.vehicle.fwd_pair_partner:
				self.vehicle.messenger.sendLeavePlatoon(self.vehicle.fwd_pair_partner, self.vehicle.bwd_pair_partner)
			elif self.vehicle.new_fwd_pair_partner:
				self.vehicle.messenger.sendLeavePlatoon(self.vehicle.new_fwd_pair_partner, self.vehicle.bwd_pair_partner)

			self.setPairPartner(None, bwd = True)

		## Pairing new bwd partner with new fwd or fwd if they exist.
		elif self.vehicle.new_bwd_pair_partner != None:

			## Telling new bwd pair partner to set None as new fwd pair partner
			self.vehicle.messenger.notifyNewPairPartner(self.vehicle.new_bwd_pair_partner, None, fwd = True, new = True)

			## Informing fwd/new fwd partner about leaving platoon, and if possible pairing fwd or new fwd pair partner with old new bwd pair partner
			if self.vehicle.fwd_pair_partner:
				self.vehicle.messenger.sendLeavePlatoon(self.vehicle.fwd_pair_partner, self.vehicle.new_bwd_pair_partner)
			elif self.vehicle.new_fwd_pair_partner:
				self.vehicle.messenger.sendLeavePlatoon(self.vehicle.new_fwd_pair_partner, self.vehicle.new_bwd_pair_partner)

			self.setPairPartner(None, bwd = True, new = True)

		## If no bwd or new bwd partner it just informs fwd/new fwd partner about leaving platoon
		else:
			if self.vehicle.fwd_pair_partner:
				self.vehicle.messenger.sendLeavePlatoon(self.vehicle.fwd_pair_partner)
			elif self.vehicle.new_fwd_pair_partner:
				self.vehicle.messenger.sendLeavePlatoon(self.vehicle.new_fwd_pair_partner)

		## Sets necessary variables, resets ACCC and updates platooning status (like reseting the merging flag etc)
		self.setPairPartner(None, fwd = True)
		self.setPairPartner(None, fwd = True, new = True)
		self.resetACCC()
		self.updatePlatooningStatus()


		## If you want a default lane change, decomment this
		if new_lane not in self.vehicle.lane_traj_dict:
			new_lane = self.getNewLaneLeft(self.vehicle.desired_lane)
			if not self.checkLaneSafety(1, left_change = True):
				new_lane = self.getNewLaneRight(self.vehicle.desired_lane)


		if new_lane in self.vehicle.lane_traj_dict: ## if new_lane is None it wont change lane either
			self.addLaneChange(new_lane)

		if new_velocity != None:
			self.setVelocity(new_velocity)


	def pingInfo(self, target, params, timeout = None):
		"""
		This function requests information about the state of another vehicle and returns it as a dict.

		A WORD OF CAUTION: This function locks the calling thread while waiting for response. Therefore,
		avoid calling it from anything related to messaging without specifying a timeout (number of seconds
		given as a float)

		:target: integer specifying the vehicle requesting state info from
		:params: a list of strings matching the ones in stateDict in SmartVehicle, specifying what info to be returned

		::RETURNS:: Dictionary with keys the same given in params

		"""
		pingReturnEvent = PingReturn()
		self.vehicle.messenger.pingInfo(target, params, pingReturnEvent)

		pingReturnEvent.wait()

		if pingReturnEvent.pingDict["vehicle_id"] == target:
			return pingReturnEvent.pingDict
		else:
			return None


	''' Functions for java commander '''

	def switchToRightLane(self):
		''' Called from java commander '''
		self.addLaneChange(self.RIGHT_LANE_STRING)

	def switchToCenterLane(self):
		''' Called from java commander '''
		self.addLaneChange(self.CENTER_LANE_STRING)

	def switchToLeftLane(self):
		''' Called from java commander '''
		self.addLaneChange(self.LEFT_LANE_STRING)


	def mergeSpecific(self, target_id = None):
		'''
		Called from java commander
		Used to merge with a specific target. Does not send merge if input is self or not smartvehicle, or if self already has fwd/new fwd partner
		'''

		if target_id == None:
			print "you must specify a target id to send desired merge"
			return
		if self.vehicle.fwd_pair_partner or self.vehicle.new_fwd_pair_partner:
			print self.vehicle.id, "already has a fwd pair partner, exit platoon before sending desired merge"
			return

		else:
			if target_id in self.vehicle.saved_smartvehicle_ids and target_id != self.vehicle.id:
				self.vehicle.messenger.sendDesiredMerge(target_id)
			else:
				print "Target "+str(target_id)+" is either not a smartvehicle or target is self, will not send desired merge"

	def toggleOvertaking(self):
		''' Called from java commander '''

		self.vehicle.overtaking_toggled = not self.vehicle.overtaking_toggled


	''' End of functions for java commander '''



	def breakPlatoon(self, fwd = 1):
		""" This function resets the variables new_<fwd or bwd>_pair_partner and <fwd or bwd>_pair_partner to break the platoon
			up in two either in front of the current vehicle. It also sends a message to affected partners to do break the link
			with the vehicle.
		"""

		if fwd:
			if self.vehicle.fwd_pair_partner != None:
				# Send break platoon message to fwd_pair_partner
				self.vehicle.messenger.sendBreakPlatoon(self.vehicle.fwd_pair_partner)
				self.setPairPartner(None, fwd = True, new = False)
				self.resetACCC()
			if self.vehicle.new_fwd_pair_partner != None:
				# Send break platoon message to new_fwd_pair_partner
				self.vehicle.messenger.sendBreakPlatoon(self.vehicle.new_fwd_pair_partner)
				self.setPairPartner(None, fwd = True, new = True)
				self.merging = False
				self.resetACCC()

		else:
			if self.vehicle.bwd_pair_partner != None:
				# Send break platoon message to bwd_pair_partner
				self.vehicle.messenger.sendBreakPlatoon(self.vehicle.bwd_pair_partner)
				self.setPairPartner(None, bwd = True, new = False)
			if self.vehicle.new_bwd_pair_partner != None:
				# Send break platoon message to new_bwd_pair_partner
				self.vehicle.messenger.sendBreakPlatoon(self.vehicle.new_bwd_pair_partner)
				self.setPairPartner(None, bwd = True, new = True)

		# update platooning status variables
		self.updatePlatooningStatus()

	def breakPlatoonReceived(self, vehicle_id):
		""" This function is called when a break platoon message has been received
			It resets affected pair partner variables
		"""

		if vehicle_id == self.vehicle.new_fwd_pair_partner:
			self.setPairPartner(None, fwd = True, new = True)
			self.vehicle.merging = False
			self.resetACCC()
			return True
		elif vehicle_id == self.vehicle.new_bwd_pair_partner:
			self.setPairPartner(None, bwd = True, new = True)
			return True
		elif vehicle_id == self.vehicle.fwd_pair_partner:
			self.setPairPartner(None, fwd = True, new = False)
			self.resetACCC()
			return True
		elif vehicle_id == self.vehicle.bwd_pair_partner:
			self.setPairPartner(None, bwd = True, new = False)
			return True
		else:
			return False


	def isPlatooning(self):
		""" Checks if vehicle is platooning based on if it has pair partners
		"""

		return self.vehicle.fwd_pair_partner != None or self.vehicle.bwd_pair_partner != None

	def updatePlatooningStatus(self):
		""" Updates platooning variables for platooning status: merging, platoon_lane and platooned_vehicle """

		self.vehicle.platooned_vehicle = self.isPlatooning()

		if self.vehicle.new_fwd_pair_partner != None:
			self.vehicle.merging = True

		if self.vehicle.platooned_vehicle and self.vehicle.platoon_leader:
			self.vehicle.platoon_lane = self.vehicle.desired_lane

		if not self.vehicle.platooned_vehicle:
			self.vehicle.platoon_lane = None



	''' Functions + class for emergency vehicle scenario '''


	def handleEmergencyVehicles(self):
		'''
		First goes through dict of saved emergency vehicle messages, checks relative coordinates. If the emergency vehicle is within the closeby_emergency_vehicles set, self does self.determineAvoidingRelevance()
		and if it returns True and self doesnt have its emergency_vehicle_offset_pointer set to the emergency vehicle id it adds it to the closeby_emergency_vehicles set.
		If the emergency vehicle is outside of the closeby_emergency_vehicle_distance the vehicle removes it from the set.
		The vehicle then goes through the closeby_emergency_vehicles set and checks if there are any vehicles in selfs perceived_objects, and tries to find the closest vehicle based on the relative x coord.
		If it found a closest vehicle and its relative x coord is within the avoid_emergency_vehicle_distance it calculates the necessary offset through self.calculateNecessaryOffset() and then
		calls self.smoothOffsetChange() to apply the offset. calculateNecessaryOffset sets an offset to the variable new_offset and sets the emergency_vehicle_offset_pointer to the emergency vehicle id.
		The smoothOffsetChange() sets a fraction of the new_offset based on the distance to the other vehicle (stays at exactly new_offset within some margin distance), and is called both when the emergency vehicle
		is approaching self and leaving self.
		Once the emergency vehicle is outside of the avoid_emergency_vehicle_distance setLaneOffset() is called without a parameter which sets offset to 0, just in case the smoothOffsetChange() doesn't manage
		to reach it. Variabels like new_offset and emergency_vehicle_offset_pointer are also reset. The same procedure occurs if there are no vehicles in the closeby_emergency_vehicles set.

		'''


		## checking the emergency vehicle messages and adding/removing vehicles to/from the closeby_emergency_vehicles set
		for em_vehicle_id in self.vehicle.saved_emergency_vehicle_messages:
			message = self.vehicle.saved_emergency_vehicle_messages[em_vehicle_id]
			other_coords = [message['x'], message['y']]
			road_coords = self.vehicle.perception_module.convertToRoadCoord(other_coords, self.vehicle.desired_lane)
			rel_coords = self.vehicle.perception_module.getRelativeRoadCoordsOval(road_coords, self.vehicle.desired_lane)

			if abs(rel_coords[0]) < self.closeby_emergency_vehicle_distance:
				if not em_vehicle_id in self.vehicle.closeby_emergency_vehicles and self.determineAvoidingRelevance(message['width'], rel_coords[1]) and self.vehicle.emergency_vehicle_offset_pointer != em_vehicle_id:
					self.vehicle.closeby_emergency_vehicles.add(em_vehicle_id)
			else:
				if em_vehicle_id in self.vehicle.closeby_emergency_vehicles:
					self.vehicle.closeby_emergency_vehicles.remove(em_vehicle_id)

		## handling the closeby emergency vehicles through sensor readings
		closest = None
		for closeby_em_vehicle in self.vehicle.closeby_emergency_vehicles:
			if closeby_em_vehicle in self.vehicle.perceived_objects:
				rel_x = self.vehicle.perceived_objects[closeby_em_vehicle][0]

				if closest == None:
					closest = closeby_em_vehicle
				elif abs(self.vehicle.perceived_objects[closest][0]) > abs(rel_x):
					closest = closeby_em_vehicle

		## applying offset
		if closest != None:

			rel_x = self.vehicle.perceived_objects[closest][0]
			rel_y = self.vehicle.perceived_objects[closest][1]

			if abs(rel_x) < self.avoid_emergency_vehicle_distance:

				if rel_x < 0:
					if self.vehicle.emergency_vehicle_offset_pointer != closest:
						self.calculateNecessaryOffset(closest , self.vehicle.saved_emergency_vehicle_messages[closest]['width'], rel_x, rel_y)

					self.smoothOffsetChange(rel_x, rel_y)


				elif rel_x > 0:
					if self.vehicle.emergency_vehicle_offset_pointer == closest:
						self.smoothOffsetChange(rel_x, rel_y)
			else:
				## Removing offset and resetting variabels, though smoothOffsetChange() should have already reached 0 offset
				self.setLaneOffset()
				self.vehicle.emergency_vehicle_offset_pointer = None
				self.vehicle.new_offset = None


		if not self.vehicle.closeby_emergency_vehicles:
			## Removing offset and resetting variabels, though smoothOffsetChange() should have already reached 0 offset
			self.vehicle.lane_offset = 0
			self.vehicle.emergency_vehicle_offset_pointer = None
			self.vehicle.new_offset = 0


	def determineAvoidingRelevance(self, other_width, rel_y):
		'''
		This function will determine whether self needs to add offset to its trajectory, and return True if so.
		It checks if the other vehicle will collide with self with respect to the relative y coords and the vehicles widths, and does it with some margin
		'''

		offset = self.vehicle.width/2 + other_width/2 - abs(rel_y)

		if offset < -self.vehicle.width/2: # roughly 1 m margin
			return False

		else:
			return True


	def calculateNecessaryOffset(self, other_id, other_width, rel_x, rel_y):
		''' should calculate necessary offset based on own lane, and set new offset if it is necessary, based on current offset. Keep offsets saved in dictionary, with em vehicle id as key '''

		offset = self.vehicle.width/2 + other_width/2 - abs(rel_y) # this is the absolute value of the final offset
		offset += self.vehicle.width/2 + other_width # adding margin of error

		offset = offset

		self.vehicle.emergency_vehicle_offset_pointer = other_id
		self.vehicle.new_offset = math.copysign(offset, -rel_y) ## Offset is in other direction from y


	def smoothOffsetChange(self, rel_x, rel_y):
		''' Sets an offset based on the distance to other vehicle, uses the new_offset as maximum and multiplies it with some factor that reaches 1 within some margin '''

		if rel_x<=0:
			offset_variance_factor = 2*(self.avoid_emergency_vehicle_distance - abs(rel_x))/self.avoid_emergency_vehicle_distance # to make it smooth
		if rel_x>0:
			offset_variance_factor = (self.avoid_emergency_vehicle_distance - abs(rel_x)*1.5)/(self.avoid_emergency_vehicle_distance) # to make it return fast, and smoothly

		if offset_variance_factor > 1:
			offset_variance_factor = 1
		if offset_variance_factor < 0:
			offset_variance_factor = 0

		self.setLaneOffset(self.vehicle.new_offset * offset_variance_factor)


	def setLaneOffset(self, offset = 0):
		''' Sets vehicle offset '''

		self.vehicle.lane_offset = offset



	''' Trying new overtaking algorithm '''

	def handleOvertakingNew(self):
		'''
		New overtaking algorithm, based on optimization. Checks perception grid to find vehicles and assumes their lane based on the margin lane_width_max.
		It then adds a score to left/right/center_lane_score (depending on other vehicles lane) depending on their relative x coord and velocity.
		The vehicle then tries to keep the lane with the lowest score if self.checkLaneSafety() returns True for that lane.

		PROBLEMS:
			- The scoring algorithm can be done better. Velocity can be factored in better.
			- Doesn't work well with platoons, if the leader gets stuck because laneChangeSafety returns False, but it returns True for the followers
		'''

		## Maybe the most pythonic code i've ever written

		## Only tries to overtake if you are not in the middle of a lane change
		if not self.vehicle.lanekeeping_ready.isSet():
			return

		## Defining objects to ignore
		if self.vehicle.merging or (self.vehicle.platooned_vehicle and not self.vehicle.platoon_leader):
			ignored_objects = self.vehicle.saved_smartvehicle_ids
		else:
			ignored_objects = set()

		## Defining score variables and giving right lane some small bias to try and keep that lane if no other cars are around
		## OBS: This bias depends heavily on the calculation of score. if you change that algorithm, change the bias!
		left_lane_score = 0.0002
		center_lane_score = 0.0001
		right_lane_score  = 0.

		## Will append the rel_x coords and velocity of vehicles in front of you to the corresponding lists. OBS: rel_x has to be first element, because it needs to sort with respect to that!!!
		left_lane_list = []
		center_lane_list = []
		right_lane_list = []

		## Defining constants
		lane_width_max = 5.


		## Defining objects to left, right and in front
		left_objects = self.vehicle.perception_grid['fl'] + self.vehicle.perception_grid['l']
		front_objects = self.vehicle.perception_grid['fm']
		right_objects = self.vehicle.perception_grid['fr'] + self.vehicle.perception_grid['r']


		## If you're in right lane
		if self.vehicle.desired_lane == self.RIGHT_LANE_STRING:

			## Checking right lane
			for vehicle_id in front_objects:
				if vehicle_id not in ignored_objects:
					rel_x = self.vehicle.perceived_objects[vehicle_id][0]
					other_vel = self.vehicle.perceived_objects[vehicle_id][2]
					if rel_x>0:
						right_lane_list.append([rel_x, other_vel])

			## Checking center lane and left lane
			for vehicle_id in left_objects:

				if vehicle_id not in ignored_objects:

					rel_x = self.vehicle.perceived_objects[vehicle_id][0]
					rel_y = self.vehicle.perceived_objects[vehicle_id][1]
					other_vel = self.vehicle.perceived_objects[vehicle_id][2]
					if rel_x > 0:
						## Checking center lane
						if abs(rel_y) < lane_width_max:
							center_lane_list.append([rel_x, other_vel])
						## Checking left lane
						elif lane_width_max < abs(rel_y) < 2.*lane_width_max:
							left_lane_list.append([rel_x, other_vel])


		## If you're in center lane
		elif self.vehicle.desired_lane == self.CENTER_LANE_STRING:

			## Checking right lane
			for vehicle_id in right_objects:
				if vehicle_id not in ignored_objects:
					rel_x = self.vehicle.perceived_objects[vehicle_id][0]
					other_vel = self.vehicle.perceived_objects[vehicle_id][2]
					if rel_x > 0:
						right_lane_list.append([rel_x, other_vel])

			## Checking center lane
			for vehicle_id in front_objects:
				if vehicle_id not in ignored_objects:
					rel_x = self.vehicle.perceived_objects[vehicle_id][0]
					other_vel = self.vehicle.perceived_objects[vehicle_id][2]
					if rel_x > 0:
						center_lane_list.append([rel_x, other_vel])

			## Checking left lane
			for vehicle_id in left_objects:
				if vehicle_id not in ignored_objects:
					rel_x = self.vehicle.perceived_objects[vehicle_id][0]
					other_vel = self.vehicle.perceived_objects[vehicle_id][2]
					if rel_x > 0:
						left_lane_list.append([rel_x, other_vel])


		## If you're in left lane
		elif self.vehicle.desired_lane == self.LEFT_LANE_STRING:

			## Checking left lane
			for vehicle_id in front_objects:
				if vehicle_id not in ignored_objects:
					rel_x = self.vehicle.perceived_objects[vehicle_id][0]
					other_vel = self.vehicle.perceived_objects[vehicle_id][2]
					if rel_x>0:
						left_lane_list.append([rel_x, other_vel])

			## Checking center lane and right lane
			for vehicle_id in right_objects:

				if vehicle_id not in ignored_objects:

					rel_x = self.vehicle.perceived_objects[vehicle_id][0]
					rel_y = self.vehicle.perceived_objects[vehicle_id][1]
					other_vel = self.vehicle.perceived_objects[vehicle_id][2]
					if rel_x > 0:
						## Checking center lane
						if abs(rel_y) < lane_width_max:
							center_lane_list.append([rel_x, other_vel])
						## Checking right lane
						elif lane_width_max < abs(rel_y) < 2.*lane_width_max:
							right_lane_list.append([rel_x, other_vel])


		## Calculating score
		left_lane_score += self.calculateLaneScore(left_lane_list)
		center_lane_score += self.calculateLaneScore(center_lane_list)
		right_lane_score += self.calculateLaneScore(right_lane_list)


		## Defining a dictionary with score corresponding to lane
		lane_dict = {right_lane_score:self.RIGHT_LANE_STRING, center_lane_score:self.CENTER_LANE_STRING, left_lane_score: self.LEFT_LANE_STRING}

		## Finding the new lane based on lowest score
		new_lane = lane_dict[min(right_lane_score, center_lane_score, left_lane_score)]

		## Checking amount of lane changes and if it's a left lane change (if False it's right change)
		[number_lane_changes, left_change] = self.checkNumberLaneChangesAndLeft(new_lane)

		# print "RIGHT: "+str(round(right_lane_score, 4))+", CENTER: "+str(round(center_lane_score, 4))+", LEFT: "+str(round(left_lane_score, 4))+", OPTIMAL: "+new_lane

		if number_lane_changes != 0:

			## Checking lane safety
			safe = False
			safe = self.checkLaneSafety(number_lane_changes, left_change = left_change, right_change = not left_change)
			if safe:
				self.addLaneChange(new_lane)

			## If not safe to do 2, try to do 1 lane change instead. Right now the new lane can only be center lane in that case.
			elif not safe and number_lane_changes == 2:
				safe = self.checkLaneSafety(1, left_change = left_change, right_change = not left_change)
				if safe:
					if left_change:
						new_lane = self.getNewLaneLeft(self.vehicle.desired_lane)
					else:
						new_lane = self.getNewLaneRight(self.vehicle.desired_lane)

					self.addLaneChange(new_lane)


	def calculateLaneScore(self, info_list):
		'''
		Calculates the total score from a list of vehicles and info about them.
		First sorts the list with highest score first, then adds each score to the total, powered to the pow_factor.
		Gives lower score for each successive element in the list, so that multiple vehicles in a row do not get an unnecessarily high score.
		The weighing of the vehicle score can definitly be improved. The choice of pow_factor hasn't been tested much.
		Now also takes velocity into account, a bit. Multiplies score by a velocity_factor, which is own velocity divided by other velocity. Own velocity could just be replaced by a constant at this point.

		OBS: If this calculation algorithm is changed, the added bias to right/center lanes needs to be changed in the handleOvertakingNew function!
		'''

		info_list.sort(reverse = True)

		pow_factor = -1.
		total_score = 0.
		idx = 1
		own_vel = self.getVelocity()

		for info in info_list:
			rel_x = info[0]
			other_vel = info[1]
			new_score = math.pow(rel_x,pow_factor)/idx

			if other_vel != 0 and own_vel != 0:
				velocity_factor = own_vel/other_vel
			elif own_vel == 0:
				velocity_factor = 1.
			elif other_vel == 0:
				velocity_factor = own_vel

			new_score = new_score*velocity_factor

			## Add 0 score if othe vehicle is moving at higher velocity than self
			# if (not self.vehicle.old_velocity and other_vel > self.getVelocity()) or (self.vehicle.old_velocity and other_vel > self.vehicle.old_velocity):
			# 	print "OTHER VEHICLE AT HIGH VELOCITY, WILL NOT OVERTAKE"
			# 	new_score = 0.

			total_score += new_score
			idx += 1

		return total_score


	def checkNumberLaneChangesAndLeft(self, new_lane):
		''' Returns an integer that tells number of lane changes, and True if left change, False if right change, None if no change '''

		## I am not proud of this function.

		current_lane = self.vehicle.desired_lane

		if (current_lane == self.RIGHT_LANE_STRING and new_lane == self.LEFT_LANE_STRING):

			return [2, True]

		if (current_lane == self.LEFT_LANE_STRING and new_lane == self.RIGHT_LANE_STRING):

			return [2, False]

		if (current_lane == self.RIGHT_LANE_STRING and new_lane == self.CENTER_LANE_STRING) or (current_lane == self.CENTER_LANE_STRING and new_lane == self.LEFT_LANE_STRING):

			return [1, True]

		if (current_lane == self.CENTER_LANE_STRING and new_lane == self.RIGHT_LANE_STRING) or (current_lane == self.LEFT_LANE_STRING and new_lane == self.CENTER_LANE_STRING):

			return [1, False]

		if current_lane == new_lane:

			return [0, None]


	def create_broadcast_message(self, message_id):

		root = ET.Element('message')

		root.set("type", "broadcast")

		ET.SubElement(root, "id", id = str(message_id))

		xml_string = ET.tostring(root)

		return xml_string

	def get_broadcast_message_id(self, broadcast_string):

		xml_root = ET.fromstring(broadcast_string)

		if xml_root.get("type") != "broadcast":

			raise NameError("Not a broadcast message")

		id_field = xml_root.find('id')

		id_string = id_field.get("id")

		return int(id_string)




''' OBS!!!!! FUNCTIONS BELOW THIS POINT WILL GO INTO A DIFFERENT SUPERVISORY MODULE '''

class EmergencyVehicleSupervisoryModule(VehicleSupervisoryModule):

	"""
	Small extension for the emergency vehicle scenario, used by the emergency vehicle.
	Has none of the functions of normal supervisory module, just the same init but with a specified offset, velocity and lane. Runs a step function that sends out a message
	"""

	def __init__(self, vehicle):
		"""TODO: to be defined1. """
		VehicleSupervisoryModule.__init__(self, vehicle)
		self.vehicle.lane_offset = 2.
		self.vehicle.desired_velocity = 70./3.6
		self.vehicle.desired_lane = self.vehicle.RIGHT_LANE_STRING

	def step(self):

		# Generating warning message
		emergencyMessage = dict()
		emergencyMessage['time'] = time.time()
		emergencyMessage['message_id'] = hash(emergencyMessage['time'])
		emergencyMessage['vehicle_id'] = self.vehicle.id
		emergencyMessage['emergency_vehicle'] = True
		emergencyMessage['x'] = self.vehicle.x
		emergencyMessage['y'] = self.vehicle.y
		emergencyMessage['velocity'] = self.getVelocity()
		emergencyMessage['width'] = self.vehicle.width
		emergencyMessage['length'] = self.vehicle.length
		emergencyMessage['offset'] = self.vehicle.lane_offset
		emergencyMessage['desired_lane'] = self.vehicle.desired_lane

		if self.vehicle.id in self.vehicle.bodies_dict:

			self.vehicle.bodies_dict[self.vehicle.id].v2v_network_output = [emergencyMessage]


	''' End of emergency vehicle functions '''
