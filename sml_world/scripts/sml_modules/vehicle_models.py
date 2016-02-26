import time, math, threading
import numpy
from sml_modules import bodyclasses

class DummyVehicle(bodyclasses.WheeledVehicle):
    '''
    This Class implements a simple traffic car.
    This car will be moving along a trajectory
    endlessly.
    It is somewhat able to avoid  other cars, by
    making a simple collision checking.
    '''

    def __init__(self, sml_world, control_rate, vehicle_id):

        if not isinstance(vehicle_id, int):
            raise NameError('In class Vehicle: constructor: id must be an integer')

        self.sml_world = sml_world
        self.bodies_dict = self.sml_world.bodies_dict

        # Defines the rate at which the
        # control loop thread will run
        self.control_rate = control_rate

        # The id of this vehicle
        self.id = vehicle_id

        # To keep track of the current trajectory
        # point in which we are
        self.current_trajectory_id = 0

        # Defines the preffered velocity that the car
        # will try to drive at (in km/h)
        self.cruise_velocity = 25.0

        # This flag tells the SimulatorModule that
        # this vehicle needs to be simulated
        self.simulated = True

        self.traj = []
        self.np_traj = []

        # The distance between the rear 
        # and front axles
        self.axles_distance = 1.9

        # The current car states. They are updated
        # by the SimulatorModule
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.x_speed = 0
        self.y_speed = 0

        # The commands that make the car move
        # Used by the SimulatorModule to predict
        # the next car state
        self.commands = dict()
        self.commands['speed'] = 0 # In m/s
        self.commands['steering'] = math.radians(0) # In radians
    
        # CC PID gain
        # The gains of the longitudinal velocity
        # controller
        self.CC_k_p = 500. # 250
        self.CC_k_i = 50. # 50
        self.CC_k_d = 150.
        self.reset_CC()

        # Defining the kind of sensors we have
        # This information will be used by the
        # SimulatorModule
        self.radar_sensor = False
        self.velodyne_sensor = True
        self.velodyne_range = 25
        self.omniscient_sensor = False
        self.sensor_readings = []

        # Starting the control loop thread
        # self.start_control_loop()
        

    def start_control_loop(self):
        '''
        Function that contains the thread creation
        '''

        t = threading.Thread(target=self.control_loop, args=([]))
        t.daemon = True
        t.start()

        return

    def control_loop(self):
        '''
        The control loop that will be running
        while the car exists
        '''

        while not self.sml_world.close:

            start_time = time.time()

            self.control_step()

            end_time = time.time()

            elapsed_time = end_time - start_time

            if elapsed_time > 1./self.control_rate:

                print "dummyvehicle.py failed desired rate"

            else:

                time.sleep( 1./self.control_rate - elapsed_time )

        return

    def set_trajectory_on_lane(self, desired_lane):
        '''
        Given the name of a lane, it will find a closed trajectory
        on this lane. Specific for the GCDC Highway scenario.
        This trajectory becomes associated to the vehicle
        which will follow it.
        
        Inputs:
        desired_lane:
            A string with the name of the desired lane, either 
            "right", "center" or "left".
        '''

        if desired_lane == "right":

            right_lane_node_ids_list = self.sml_world.road_module.osm_node_tag_dict['right_lane_node']
            lane_node_id = right_lane_node_ids_list[0]

        elif desired_lane == "center":

            center_lane_node_ids_list = self.sml_world.road_module.osm_node_tag_dict['center_lane_node']
            lane_node_id = center_lane_node_ids_list[0]

        elif desired_lane == "left":

            left_lane_node_ids_list = self.sml_world.road_module.osm_node_tag_dict['left_lane_node']
            lane_node_id = left_lane_node_ids_list[0]

        else:

            raise NameError("Unexpected lane.")

        self.set_closed_trajectory_on_node(lane_node_id, 5.)

        return


    def set_closed_trajectory_on_node(self, traj_node_id, trajectory_points_per_meter = 5.):
        '''
        Given a node id, it will find a closed trajectory
        that includes it.
        This trajectory becomes associated to the vehicle
        which will follow it.
        The trajectory starts next to the given node.
        
        Inputs:
        traj_node_id:
            the id of the node where the trajectory
            should start
        trajectory_points_per_meter:
            the resolution of the trajectory
        
        '''

        self.trajectory_points_per_meter = trajectory_points_per_meter
        [traj_x, traj_y] = self.sml_world.road_module.get_closed_path_from_node_id(traj_node_id, trajectory_points_per_meter)

        if len(traj_x) == 0 or len(traj_y) == 0:

            #Error, could not find a suitable trajectory
            raise NameError("Trajectory not found")

        traj_theta = []
        temp_theta = 0

        for idx in range( len ( traj_x ) - 1 ):

            delta_x = traj_x[idx+1] - traj_x[idx]
            delta_y = traj_y[idx+1] - traj_y[idx]

            temp_theta = math.atan2(delta_y, delta_x)
            traj_theta.append(temp_theta)
            
        traj_theta.append(temp_theta)

        if len(traj_y) != len(traj_x)  or len(traj_theta) != len(traj_x):

            raise NameError("DummyVehicle Trajectory creation resulted in a mistake!")

        self.traj = [traj_x, traj_y, traj_theta]
        self.np_traj = numpy.asarray([ traj_x , traj_y , traj_theta])

        return


    def set_vehicle_on_trajectory_state(self, trajectory_idx = 0):
        '''
        Given a trajectory point index, it will place the car 
        on this trajectory point.
        Useful to place several cars on the same trajectory
        but in different places.

        Inputs:
        trajectory_idx:
            The trajectory point index, where we will
            place the car.

        '''

        self.current_trajectory_id = trajectory_idx

        if len( self.traj ) == 0:

            raise NameError('DummyVehicle: trying to use set_vehicle_on_trajectory_state when trajectory is not yet defined')

        self.x = self.traj[0][trajectory_idx]
        self.y = self.traj[1][trajectory_idx]
        self.yaw = math.degrees( self.traj[2][trajectory_idx] )

        return


    def get_steering_command(self, current_reference, max_steering_radians = 0.5):
        '''
        Implements a lateral controller.

        Inputs:
        current_reference:
            The current reference state
        max_steering_radians: (OPTIONAL)
            The maximum steering angle in radians

        Output:
        steering_command:
            The steering command in radians
        '''

        tracking_error = self.get_tracking_error(current_reference)

        steering_command = 1.0* (1.0*tracking_error[1] + 1.5*tracking_error[2]) 

        # Need to limit the steering command to max_steering_radians
        if steering_command > max_steering_radians:
            steering_command = max_steering_radians

        if steering_command < -max_steering_radians:
            steering_command = -max_steering_radians

        return steering_command


    def get_controller_action(self, current_reference, desired_velocity):
        '''
        Get the combined controller action for
        the longitudinal and lateral control.

        Inputs:
        current_reference:
            The current reference point
        desired_velocity:
            The desired velocity for the vehicle

        Output:
        [throttle_command, steering_command]
            The throttle and steering commands
        '''

        steering_command = self.get_steering_command(current_reference)

        if desired_velocity == 0.:

            # Apply the full brakes
            throttle_command = -10000

        else:

            throttle_command = self.get_throttle_CC(desired_velocity)

            if throttle_command < 0. :

                throttle_command = 10.*throttle_command
        
        command_inputs = [throttle_command, steering_command]
        # print "command_inputs = " + str(command_inputs)
        return command_inputs

    def get_tracking_error(self, reference_state):
        '''
        Computes the tracking error from the current
        car state to a reference state

        Inptus:
        reference_state:
            The reference state

        Output:
        [error_x, error_y, error_theta]
            The tracking error state

        '''

        current_yaw = math.radians(self.yaw)

        error_x = math.cos( current_yaw )*(reference_state[0] - self.x) + math.sin( current_yaw )*(reference_state[1] - self.y)
        error_y = -math.sin( current_yaw )*(reference_state[0] - self.x) + math.cos( current_yaw )*(reference_state[1] - self.y)
        error_theta = reference_state[2] - current_yaw

        while error_theta > math.pi:
            error_theta -= 2*math.pi
        while error_theta < -math.pi:
            error_theta += 2*math.pi

        return [error_x, error_y, error_theta]

    
    def distance_to_car_in_future_trajectory(self, safety_distance):

        # I will compute the distance to a car that is overlapping with my close future trajectory
        # Returns 100.0 if no car is detected

        body_readings = self.sensor_readings

        best_distance = 10e10
        best_idx = 10e10


        # meters_ahead = 10.0
        meters_ahead = 1.5*safety_distance
        # colliding_distance = 1.5
        colliding_distance = 0.25
        squared_colliding_distance = colliding_distance**2
        mega_safe_distance = ( 2.0*meters_ahead )**2

        future_traj_np = self.get_future_traj(meters_ahead)
        
        closest_car_distance = 100.

        for idx, body_reading in enumerate( body_readings ):

            temp_body_reading = []
            temp_body_reading.append( body_reading['x'] )
            temp_body_reading.append( body_reading['y'] )
            temp_body_reading.append( math.radians( body_reading['yaw'] ) )

            axis_length_obstacle_avoidance = self.axles_distance

            if ( (future_traj_np[0,0] - temp_body_reading[0])**2 + (future_traj_np[1,0] - temp_body_reading[1])**2 ) > mega_safe_distance:

                # Vehicle is super far away, I do not need to check it
                continue

            # print "temp_body_reading[2] = " + str(temp_body_reading[2])

            distance_angle = math.atan2( future_traj_np[1, 0] - temp_body_reading[1], future_traj_np[0,0] - temp_body_reading[0] )

            angle_difference = distance_angle - future_traj_np[2, 0] 

            while angle_difference < 0:
                angle_difference = angle_difference + 2.0*math.pi

            while angle_difference > 2.0*math.pi:
                angle_difference = angle_difference - 2.0*math.pi

            if angle_difference < math.pi/2.0 or angle_difference > math.pi*(3.0/2.0):

                # print "Vehicle is behind me, I do not need to check it"
                continue

            other_vehicle_axle = numpy.array( [ [temp_body_reading[0]], [temp_body_reading[1]] ] )

            other_vehicle_front_axle = numpy.array([ [temp_body_reading[0] + axis_length_obstacle_avoidance*math.cos( temp_body_reading[2] )],[ 
                        temp_body_reading[1] + axis_length_obstacle_avoidance*math.sin( temp_body_reading[2] )]])

            # print "other_vehicle_front_axle = " + str(other_vehicle_front_axle)

            future_traj_front_axle = numpy.copy( future_traj_np[0:2, :] )

            future_traj_front_axle[0, :] = future_traj_front_axle[0, :] + axis_length_obstacle_avoidance*numpy.cos( future_traj_np[2, :] )
            future_traj_front_axle[1, :] = future_traj_front_axle[1, :] + axis_length_obstacle_avoidance*numpy.sin( future_traj_np[2, :] )

            # print "\n future_traj_np[0,0] = " + str(future_traj_np[0,0]) + "\nfuture_traj_front_axle[0,0] = " + str(future_traj_front_axle[0,0])

            # print " future_traj_np.shape[1] = " + str( future_traj_np.shape[1])

            # Check my front axle for collision with front axle        
            temp_distance = numpy.sum((future_traj_front_axle - other_vehicle_front_axle)**2, axis = 0)
            # Find the closest trajectory point that matches my desired speed and current heading
            # best_idx = numpy.argmin(temp_distance)
            danger_points = numpy.where( temp_distance < squared_colliding_distance )

            dangerest_distance = 100.

            if len(danger_points[0]) > 0:

                dangerest_distance = 0.2*danger_points[0][0]

            # Check my front axle for collision with rear axle
            temp_distance = numpy.sum((future_traj_front_axle - other_vehicle_axle)**2, axis = 0)
            # Find the closest trajectory point that matches my desired speed and current heading
            # best_idx = numpy.argmin(temp_distance)
            danger_points = numpy.where( temp_distance < squared_colliding_distance )

            if len(danger_points[0]) > 0:

                dangerest_distance = 0.2*danger_points[0][0]
            
            # Check my axle for collision with front axle        
            temp_distance = numpy.sum((future_traj_np[0:2,:] - other_vehicle_front_axle)**2, axis = 0)
            # Find the closest trajectory point that matches my desired speed and current heading
            # best_idx = numpy.argmin(temp_distance)
            danger_points = numpy.where( temp_distance < squared_colliding_distance )

            if len(danger_points[0]) > 0:

                dangerest_distance = 0.2*danger_points[0][0]

            # Check my axle for collision with rear axle
            temp_distance = numpy.sum((future_traj_np[0:2,:] - other_vehicle_axle)**2, axis = 0)
            # Find the closest trajectory point that matches my desired speed and current heading
            # best_idx = numpy.argmin(temp_distance)
            danger_points = numpy.where( temp_distance < squared_colliding_distance )

            if len(danger_points[0]) > 0:

                dangerest_distance = 0.2*danger_points[0][0]    

            if dangerest_distance < closest_car_distance:

                closest_car_distance = dangerest_distance

        return closest_car_distance

    def get_future_traj(self, meters_ahead):
        '''
        Returns the future trajectory up to 
        a given number of meters ahead.
        The future trajectory starts at the
        current car state.

        Inputs:
        meters_ahead:
            The length of the future trajectory

        Output:
        future_traj_np:
            The future trajectory, composed of
            an x, y and yaw, as a numpy array
        '''        
        start_idx = self.current_trajectory_id
        end_idx = int( self.current_trajectory_id + round(meters_ahead*self.trajectory_points_per_meter) )
        future_traj_ids = [0] # I will add a dummy number, in order to then overwrite it with my current state
        
        
        if end_idx > self.np_traj.shape[1]:

            end_idx = end_idx%self.np_traj.shape[1]
            future_traj_ids.extend(range( start_idx, self.np_traj.shape[1]) )
            future_traj_ids.extend(range( end_idx ) )

        else:

            future_traj_ids.extend(range( start_idx, end_idx) )


        future_traj_np = self.np_traj[:, future_traj_ids]

        future_traj_np[0, 0] = self.x
        future_traj_np[1, 0] = self.y
        future_traj_np[2, 0] = math.radians(self.yaw)
        
        return future_traj_np

    def find_closest_trajectory_point(self):
        '''
        This function will look for the closest point
        (in the trajectory) to the current state.

        Output:
        best_idx:
            The index of the closest trajectory point
            to the current car state
        '''

        np_state = numpy.array([[self.x],[self.y]])
        
        temp_distance = numpy.sum((self.np_traj[0:2, :] - np_state)**2, axis = 0)

        # Find the closest trajectory point that matches my desired speed and current heading
        best_idx = numpy.argmin(temp_distance)

        return best_idx


    def control_step(self):
        '''
        The control step, which computes the 
        input commands based on the current car state
        and on the cars surrounding me
        '''
        
        # Find the closest trajectory point
        current_state = [self.x, self.y]
        best_idx = self.find_closest_trajectory_point()
    
        traj_len = len( self.traj[0] )

        self.current_trajectory_id = best_idx

        # The reference state will be the trajectory point
        # 5 indexes ahead of the closest trajectory point
        # This improves the lateral controller performance
        best_idx += 5
        reference_state = [self.traj[0][ (best_idx)%traj_len ] , self.traj[1][ (best_idx)%traj_len ] , self.traj[2][ (best_idx)%traj_len ] ]

        # We start braking if other cars are closer than this distance
        safety_distance = 10.
        # We brake completely if other cars are closer than this distance
        full_stop_distance = 6.
        
        # Will check distance to cars in front
        distance_to_car = self.distance_to_car_in_future_trajectory(safety_distance)


        if distance_to_car < full_stop_distance:

            # If we are extremely close to another car, we need to brake completely
            desired_velocity = 0.
            
        elif distance_to_car < safety_distance:

            # If we are close to another car, we need to start braking
            desired_velocity = (self.cruise_velocity / 3.6)*(distance_to_car/safety_distance)

        else:

            # In case there are no cars close by, just 
            # drive at the cruise velocity
            desired_velocity = self.cruise_velocity / 3.6
                
        self.commands['speed'] = desired_velocity

        steering_command = self.get_steering_command(reference_state)
        self.commands['steering'] = steering_command

        '''
        This should be used if we wish to do throttle control
        and used an acceleration based car model
        
        [throttle_command, steering_command] = self.get_controller_action(reference_state, desired_velocity)
        self.bodies_array[self.id].commands['throttle'] = throttle_command
        self.bodies_array[self.id].commands['steering'] = steering_command
        '''

        return

    def get_throttle_CC(self, desired_velocity):
        '''
        This function is called when the CC mode is on.
        It simply runs a PID that outputs the longitudinal velocity
        The error source of this PID is the error in the current distance
        to the front vehicle, agains the desired distance

        Inputs:
        desired_velocity:
            The desired/reference velocity
            in m/s

        Output:
        control_action:
            The PID control action. Tuned to the
            particular case when it is a throttle command

        '''
        current_time = time.time()

        linear_velocity = math.hypot(self.x_speed, self.y_speed)

        current_error = desired_velocity - linear_velocity

        if self.last_CC_time:

            # Compute last sampling time
            time_passed = current_time - self.last_CC_time
            self.last_CC_time = current_time

            # Integrate the error
            self.CC_I_e += time_passed*current_error

            max_integral_action = 1000.
            integral_action = self.CC_k_i*self.CC_I_e

            if math.fabs(integral_action) > max_integral_action:

                integral_action = math.copysign(max_integral_action, integral_action)
                self.CC_I_e = integral_action/self.CC_k_i

            # Compute the derivative of the error
            if time_passed >0:
                self.derivative_error = (current_error - self.last_CC_e)/time_passed

            self.last_CC_e = current_error

            # Compute PID output
            control_action = 0 + self.CC_k_p*current_error + integral_action + self.CC_k_d*self.derivative_error

        else:

            # Initialization of the ACCC controller (first iteration of this controller)
            self.last_CC_time = current_time
            self.last_CC_e = current_error
            self.CC_I_e = 0.0
            self.derivative_error = 0.0

            control_action = 0


        return control_action

    def reset_CC(self):
        '''
        To make sure that the CC is reseted when it is not used
        It cleans up the PID states.
        '''

        self.last_CC_time = []
        self.CC_I_e = []
        self.last_CC_e = []

        return


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

        # Compute my old linear speed and my current acceleration
        # in order to know my new linear speed
    
        linear_speed = self.commands['speed']

        # Knowing the current linear speed, I can know the speed components in x and y
        self.x_speed = math.cos( math.radians( self.yaw ) )*linear_speed
        self.y_speed = math.sin( math.radians( self.yaw ) )*linear_speed
        
        # Update my position
        self.x = self.x + self.x_speed*time_to_simulate
        self.y = self.y + self.y_speed*time_to_simulate

        # Update my yaw, assuming a simple kinematic car model
        self.yaw = self.yaw + math.degrees( (linear_speed/self.axles_distance)*math.tan(self.commands['steering'])*time_to_simulate )
        

        # Limit the yaw to the range between 0 and 360
        while self.yaw > 360:

            self.yaw -= 360

        while self.yaw < 0:

            self.yaw += 360



    '''
    Vehicle Sensor Readings Interface
    Author: Rui
    This method allow the SimulatorModule to interact
    with the SmartVehicle in a cleaner / more
    Object Oriented way
    '''

    def set_sensor_readings(self, sensor_readings_list):
        '''
        Given a list of sensor readings, I will store them
        in my attribute sensor_readings
        '''
        self.sensor_readings = sensor_readings_list
        return