import time, math, threading, socket

import bodyclasses
import smartvehicle
import dummyvehicle
import BlocklyVehicle

# Shapely is a non standard library
# Needs installation to be used
import shapely.geometry

class BlocklyModuleHighway:
    '''
    Class that interfaces with the Blocky Python Code.
    '''

    def __init__(self, sml_world, simulated = True, blockly_vehicle_id = None):

        # Get a reference to the sml_world,
        # so that we are able to access the
        # bodies_dict
        self.sml_world = sml_world

        self.initial_car_x = 0
        self.initial_car_y = 0
        self.initial_car_yaw = 0

        self.simulated = simulated

        self.blockly_vehicle_id = blockly_vehicle_id

        self.loop_rate = 10.

        self.blockly_vehicle = None
        self.create_blockly_vehicle()

        self.set_blockly_vehicle_speed(0)
        self.set_blockly_vehicle_steering_degrees(0)

        # To add an ever increasing packet
        # number to every packet sent
        self.sent_packet_counter = 0
        # Initialize the sender socket
        # The ip of the computer where the 
        # Blockly code is running is detected
        # automatically once we receive messages
        # from it.
        self.blocky_udp_ip = None
        self.blocky_udp_port = 34514
        self.start_udp_sender_socket()

        # To keep track of the most recent
        # packet received, and to discart possible
        # our of order packets
        self.received_packet_counter = -1

        # The port in which we are going to listen
        # for Blockly messages
        self.sml_world_listening_port = 34513
        # Initialize the receiver socket
        self.start_udp_receiver_socket()
        # Keeps track of the time since last packet
        # received
        self.last_packet_received_time = time.time()
        # If the time since the last packet received
        # exceeds this time out, we restart the 
        # packet counter, and reset the vehicle to 
        # the initial state
        self.receiving_packet_timeout = 2.5


    def set_initial_blockly_vehicle_state(self, initial_blockly_x, initial_blockly_y, initial_blockly_yaw):

        self.initial_car_x = initial_blockly_x
        self.initial_car_y = initial_blockly_y
        self.initial_car_yaw = initial_blockly_yaw

        return



    def start_blockly_module(self):

        # Launches the tread that will run the main
        # loop of the SimulatorModule
        t = threading.Thread( target = self.thread_loop, args=([]) )
        t.daemon = True
        t.start()

        return

    def thread_loop(self):
        '''
        This function is executed in a thread, thus running 
        in parallel with the rest of the SML World.
        It consists of a loop, where messages with Blockly
        code are exchanged.
        '''

        # Time keepers
        start_loop_time = time.time()
        end_loop_time = time.time()

        patch = shapely.geometry.Point(0.0, 0.0).buffer(10.0)

        while not self.sml_world.close:

            start_loop_time = time.time()

            self.update_surroundings()

            self.send_vehicle_surroundings_packet()    

            self.process_incoming_packets()

            end_loop_time = time.time()

            time_elapsed = end_loop_time - start_loop_time

            time_to_sleep = 1./self.loop_rate - time_elapsed

            if time_to_sleep > 0:

                time.sleep(time_to_sleep)

            else:

                print "BlocklyModule failed desired rate."

        self.close_sockets()

        return


    def create_blockly_vehicle(self):

        if self.blockly_vehicle_id != None:

            new_id = self.blockly_vehicle_id

        else:

            body_ids = self.sml_world.bodies_dict.keys()

            if body_ids:

                new_id = min(body_ids) - 1

                if new_id >= 0:

                    new_id = -1

            else:

                new_id = -1


        if self.simulated:
            # Creating a simulated smart vehicle with id vehicle_id
            blockly_vehicle = smartvehicle.SmartVehicle(self.sml_world, new_id)
        else:
            # Creating a real smart vehicle with id self.blockly_vehicle_id
            blockly_vehicle = smartvehicle.SmartVehicle(self.sml_world, new_id, real_vehicle = True)

        print "new_id = " + str(new_id)

        # Adding the vehicle to the bodies array
        self.sml_world.bodies_dict[new_id] = blockly_vehicle
        # Setting it on a specific trajectory index
        blockly_vehicle.set_vehicle_on_trajectory_state(0)

        # blockly_vehicle = BlocklyVehicle.BlocklyVehicle(new_id, simulated = self.simulated)
        # self.sml_world.bodies_dict[new_id] = blockly_vehicle

        self.blockly_vehicle = blockly_vehicle

        self.blockly_vehicle.desired_lane = self.blockly_vehicle.RIGHT_LANE_STRING
        self.blockly_vehicle.desired_velocity = 15./3.6

        self.reset_blocky_state()

        return

    def place_one_vehicle(self, ahead):
        '''
        This function will create and place the
        smart vehicles in the intersection map.
        '''

        # The desired number of vehicles to place
        num_dummy_vehicles = 1

        # The id of the vehicles, starting at -1, and 
        # decreasing for each vehicle placed: -2, -3, ...
        vehicle_id = -1

        # The first car will be placed in the starting (zero index)
        # trajectory point      
        start_index = 0

        for i in range(num_dummy_vehicles):


            dummy_vehicle = dummyvehicle.DummyVehicle(self.sml_world, 10., vehicle_id)

            if i % 3 == 0:
                dummy_vehicle.set_trajectory('right')
            elif  i % 3 == 1:
                dummy_vehicle.set_trajectory('center')
            else:
                dummy_vehicle.set_trajectory('left')

            if ahead:

                dummy_vehicle.set_vehicle_on_trajectory_state(int(-200*vehicle_id))

            else:

                dummy_vehicle.set_vehicle_on_trajectory_state(int(200*vehicle_id))

            self.sml_world.bodies_dict[vehicle_id] = dummy_vehicle 

            dummy_vehicle.start_control_loop()

            # Placing the vehicle in the trajectory point with id start_index
            # smart_vehicle.set_vehicle_on_trajectory_state(start_index)

            # start_index += 100

            # For each vehicle placed, the vehicle id has to decrease a unit
            vehicle_id -= 1

    def place_several_vehicles(self, ahead):
        '''
        This function will create and place the
        smart vehicles in the intersection map.
        '''

        # The desired number of vehicles to place
        num_dummy_vehicles = 11

        # The id of the vehicles, starting at -1, and 
        # decreasing for each vehicle placed: -2, -3, ...
        vehicle_id = -1

        # The first car will be placed in the starting (zero index)
        # trajectory point      
        if ahead:
            start_index = 0
        else:
            start_index = -200

        for i in range(num_dummy_vehicles):


            dummy_vehicle = dummyvehicle.DummyVehicle(self.sml_world, 10., vehicle_id)

            # if i % 3 == 0:
            dummy_vehicle.set_trajectory('center')
            # elif  i % 3 == 1:
            #     dummy_vehicle.set_trajectory('center')
            # else:
            #     dummy_vehicle.set_trajectory('left')

            if ahead:

                # dummy_vehicle.set_vehicle_on_trajectory_state(int(start_index -20*vehicle_id))
                dummy_vehicle.set_vehicle_on_trajectory_state(int(start_index -150*vehicle_id))

            else:

                dummy_vehicle.set_vehicle_on_trajectory_state(int(start_index + 30*vehicle_id))

            self.sml_world.bodies_dict[vehicle_id] = dummy_vehicle 

            dummy_vehicle.start_control_loop()

            # Placing the vehicle in the trajectory point with id start_index
            # smart_vehicle.set_vehicle_on_trajectory_state(start_index)

            # start_index += 100

            # For each vehicle placed, the vehicle id has to decrease a unit
            vehicle_id -= 1


    def place_several_vehicles_all_over(self, num_vehicles):
        '''
        This function will create and place the
        smart vehicles in the intersection map.
        '''

        # The desired number of vehicles to place
        num_dummy_vehicles = num_vehicles

        # The id of the vehicles, starting at -1, and 
        # decreasing for each vehicle placed: -2, -3, ...
        vehicle_id = -1

        # The first car will be placed in the starting (zero index)
        # trajectory point      
        start_index = 200

        jump_value = int(round(1800./float(num_vehicles)))

        for i in range(num_dummy_vehicles):


            dummy_vehicle = dummyvehicle.DummyVehicle(self.sml_world, 10., vehicle_id)

            if i % 3 == 0:
                dummy_vehicle.set_trajectory('right')
            elif  i % 3 == 1:
                dummy_vehicle.set_trajectory('center')
            else:
                dummy_vehicle.set_trajectory('left')

            dummy_vehicle.set_vehicle_on_trajectory_state(int(start_index + jump_value*vehicle_id))

            self.sml_world.bodies_dict[vehicle_id] = dummy_vehicle 

            dummy_vehicle.start_control_loop()

            # Placing the vehicle in the trajectory point with id start_index
            # smart_vehicle.set_vehicle_on_trajectory_state(start_index)

            # start_index += 100

            # For each vehicle placed, the vehicle id has to decrease a unit
            vehicle_id -= 1

    def reset_blocky_state(self):

        if self.simulated:

            self.blockly_vehicle.commands['speed'] = 0.
            self.blockly_vehicle.commands['steering'] = 0.

            # self.blockly_vehicle.x = self.initial_car_x
            # self.blockly_vehicle.y = self.initial_car_y
            # self.blockly_vehicle.yaw = self.initial_car_yaw

            self.blockly_vehicle.set_vehicle_on_trajectory_state(0)

        return


    def set_blockly_vehicle_speed(self, desired_speed):

        # print "New blocky vehicle speed " + str(desired_speed)

        self.blockly_vehicle.commands['speed'] = desired_speed

        return 

    def set_blockly_vehicle_steering_degrees(self, desired_steering_degrees):

        # print "New blocky steering angle " + str(desired_steering_degrees)

        self.blockly_vehicle.commands['steering'] = math.radians(desired_steering_degrees)

        return 

    # def start_matlab_socket(self):
    #     '''
    #     Initializes the sending socket, that
    #     will send the vehicle states to the
    #     Simulator.
    #     '''

    #     self.udp_matlab_socket = socket.socket(socket.AF_INET, # Internet
    #     socket.SOCK_DGRAM) # UDP

    #     return

    def start_udp_sender_socket(self):
        '''
        Initializes the sending socket, that
        will send the vehicle states to the
        Simulator.
        '''

        self.udp_sender_socket = socket.socket(socket.AF_INET, # Internet
        socket.SOCK_DGRAM) # UDP

        return


    def start_udp_receiver_socket(self):
        '''
        Initializes the receiving socket, that
        will receive driver in the loop inputs
        (steering)
        '''

        UDP_IP = '' # Allows us to receive from any external PC
        UDP_PORT = self.sml_world_listening_port

        self.udp_reveiver_socket = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        self.udp_reveiver_socket.bind((UDP_IP, UDP_PORT))
        self.udp_reveiver_socket.settimeout(0.01)

        return

    def send_matlab_blockly_commands(self):

        message_string = str(self.sent_matlab_packet_counter)
        message_string += ";"

        if self.blockly_vehicle.commands['speed'] == 0:

            message_string += str(0)

        elif self.blockly_vehicle.commands['speed'] > 0:

            message_string += str(1)

        else:

            message_string += str(-1)

        message_string += ";"

        if self.blockly_vehicle.commands['steering'] == 0:

            message_string += str(0)

        elif self.blockly_vehicle.commands['steering'] > 0:

            message_string += str(1)

        else:

            message_string += str(-1)

        print "MATLAB message: " + message_string

        self.udp_matlab_socket.sendto(message_string, (self.matlab_udp_ip, self.matlab_udp_port) )

        # print "message_string = " + str(message_string)
        # print "self.sent_matlab_packet_counter = " + str(self.sent_matlab_packet_counter)

        self.sent_matlab_packet_counter += 1

        return

    def send_vehicle_surroundings_packet(self):        
        '''
        Creates the UDP packet containing
        the surroundings of the Blockly Vehicle
        '''
        
        if not self.blocky_udp_ip:

            if hasattr(self, 'blockly_failed_sends'):

                self.blockly_failed_sends += 1

            else:

                self.blockly_failed_sends = 1

            if self.blockly_failed_sends%50 == 0:

                print "BlocklyModuleHighway.py WARNING: Blockly Computer IP not found yet. No messages being exchanged."

            return

        self.car_in_right = False
        self.car_in_rear_right = False
        self.car_in_rear_middle = False
        self.car_in_rear_left = False
        self.car_in_left = False
        self.car_in_front_left = False
        self.car_in_front_middle = False
        self.car_in_front_right = False
        
        if 'r' in self.blockly_vehicle.perception_grid:

            if self.blockly_vehicle.perception_grid['r']:
                self.car_in_right = True
            if self.blockly_vehicle.perception_grid['rr']:
                self.car_in_rear_right = True
            if self.blockly_vehicle.perception_grid['rm']:
                self.car_in_rear_middle = True
            if self.blockly_vehicle.perception_grid['rl']:
                self.car_in_rear_left = True
            if self.blockly_vehicle.perception_grid['l']:
                self.car_in_left = True
            if self.blockly_vehicle.perception_grid['fl']:
                self.car_in_front_left = True
            if self.blockly_vehicle.perception_grid['fm']:
                self.car_in_front_middle = True
            if self.blockly_vehicle.perception_grid['fr']:
                self.car_in_front_right = True

        

        message_string = str(self.sent_packet_counter)
        message_string += ";"

        message_string += str(self.car_in_right)
        message_string += ";"
        message_string += str(self.car_in_rear_right)
        message_string += ";"
        message_string += str(self.car_in_rear_middle)
        message_string += ";"
        message_string += str(self.car_in_rear_left)
        message_string += ";"
        message_string += str(self.car_in_left)
        message_string += ";"
        message_string += str(self.car_in_front_left)
        message_string += ";"
        message_string += str(self.car_in_front_middle)
        message_string += ";"
        message_string += str(self.car_in_front_right)
        message_string += ";"

        if self.blockly_vehicle.desired_lane == self.blockly_vehicle.LEFT_LANE_STRING:

            message_string += "left"

        elif self.blockly_vehicle.desired_lane == self.blockly_vehicle.CENTER_LANE_STRING:

            message_string += "middle"

        elif self.blockly_vehicle.desired_lane == self.blockly_vehicle.RIGHT_LANE_STRING:
            
            message_string += "right"

        else:

            raise NameError("Unknown Lane")

        self.udp_sender_socket.sendto(message_string, (self.blocky_udp_ip, self.blocky_udp_port) )

        self.sent_packet_counter += 1

        return

    def process_incoming_packets(self):        
        '''
        It will see if there are any incoming 
        packets, and it will process them all.
        It is currently used to obtain the 
        steering from the driver in the loop.
        '''

        buffer_size = 1024

        time_since_last_packet = time.time() - self.last_packet_received_time

        # if time_since_last_packet > self.receiving_packet_timeout and self.received_packet_counter != -1:

        #     self.last_packet_received_time = time.time()
        #     self.received_packet_counter = -1
        #     print "Reseting scenario due to lack of commands"
        #     self.reset_blocky_state()


        try:

            while True:

                data, addr = self.udp_reveiver_socket.recvfrom(buffer_size) 
                
                if self.blocky_udp_ip != addr[0]:

                    print "Changing Blockly UDP IP from " + self.blocky_udp_ip + " to " + addr[0]

                    self.blocky_udp_ip = addr[0]

                print "Received from blockly code: " + str(data)

                tokens = data.split(';')

                packet_number = int(tokens[0])

                if packet_number <= self.received_packet_counter:

                    # Out of order packet, ignore    
                    print "BlockyModule received out of order packet with number " + str(packet_number)
                    # print "Will still accept it"         

                    if packet_number == 0:

                        print "Restarting packet counter."

                    else:

                        print "Ignoring packet."
                        continue

                self.last_packet_received_time = time.time()

                self.received_packet_counter = packet_number

                desired_lane_string = tokens[1]

                if desired_lane_string == 'left':

                    self.blockly_vehicle.desired_lane = self.blockly_vehicle.LEFT_LANE_STRING

                elif desired_lane_string == 'middle':

                    self.blockly_vehicle.desired_lane = self.blockly_vehicle.CENTER_LANE_STRING

                elif desired_lane_string == 'right':
                    
                    self.blockly_vehicle.desired_lane = self.blockly_vehicle.RIGHT_LANE_STRING

                else:

                    print "desired_lane_string = " + str(desired_lane_string)
                    raise NameError("Unexpected lane name!")


                desired_velocity_string = tokens[2]

                if desired_velocity_string == 'fast':

                    self.blockly_vehicle.desired_velocity = 20./3.6

                elif desired_velocity_string == 'normal':

                    self.blockly_vehicle.desired_velocity = 15./3.6

                elif desired_velocity_string == 'slow':

                    self.blockly_vehicle.desired_velocity = 10./3.6

                elif desired_velocity_string == 'stop':    

                    self.blockly_vehicle.desired_velocity = 0.

                else:

                    raise NameError("Unexpected lane name!")


        except socket.timeout:

            # If nothing to receive a timeout exception will 
            # be thrown, simply ignore this exception
            return

        return

    def rotate_point_by_yaw_radians(self, x, y, yaw_radians):

        x_r = math.cos(yaw_radians)*x - math.sin(yaw_radians)*y
        y_r = math.sin(yaw_radians)*x + math.cos(yaw_radians)*y

        return [x_r, y_r]

    def update_surroundings(self):

        

        return


