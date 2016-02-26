import time, math, threading, socket
import BlocklyVehicle
import bodyclasses

# Shapely is a non standard library
# Needs installation to be used
import shapely.geometry

class BlocklyModule:
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

        if not self.simulated:

            self.matlab_udp_ip = 'localhost'
            self.matlab_udp_port = 34515
            # To add an ever increasing packet
            # number to every packet sent
            self.sent_matlab_packet_counter = 0
            # Initialize the sender socket
            self.start_matlab_socket()

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

        self.start_blockly_module()


    def set_initial_blockly_vehicle_state(self, initial_blockly_x, initial_blockly_y, initial_blockly_yaw):

        self.initial_car_x = initial_blockly_x
        self.initial_car_y = initial_blockly_y
        self.initial_car_yaw = initial_blockly_yaw

        return



    def set_blockly_scenario_level(self, blockly_level):

        if blockly_level == 1:

            self.set_up_blocky_level_1()

        elif blockly_level == 2:

            self.set_up_blocky_level_2()

        elif blockly_level == 3:

            self.set_up_blocky_level_3()

        elif blockly_level == 4:

            self.set_up_blocky_level_4()

        elif blockly_level == 5:

            self.set_up_blocky_level_5()

        else:

            raise NameError('Unspecified Level')

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

            if not self.simulated:

                self.send_matlab_blockly_commands()

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

        blockly_vehicle = BlocklyVehicle.BlocklyVehicle(new_id, simulated = self.simulated)

        self.sml_world.bodies_dict[new_id] = blockly_vehicle

        self.blockly_vehicle = blockly_vehicle

        self.reset_blocky_state()

        return

    def reset_blocky_state(self):

        if self.simulated:

            self.blockly_vehicle.commands['speed'] = 0.
            self.blockly_vehicle.commands['steering'] = 0.

            self.blockly_vehicle.x = self.initial_car_x
            self.blockly_vehicle.y = self.initial_car_y
            self.blockly_vehicle.yaw = self.initial_car_yaw

        return


    def set_blockly_vehicle_speed(self, desired_speed):

        # print "New blocky vehicle speed " + str(desired_speed)

        self.blockly_vehicle.commands['speed'] = desired_speed

        return 

    def set_blockly_vehicle_steering_degrees(self, desired_steering_degrees):

        # print "New blocky steering angle " + str(desired_steering_degrees)

        self.blockly_vehicle.commands['steering'] = math.radians(desired_steering_degrees)

        return 

    def start_matlab_socket(self):
        '''
        Initializes the sending socket, that
        will send the vehicle states to the
        Simulator.
        '''

        self.udp_matlab_socket = socket.socket(socket.AF_INET, # Internet
        socket.SOCK_DGRAM) # UDP

        return

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

                print "BlocklyModule.py WARNING: Blockly Computer IP not found yet. No messages being exchanged."

            return

        message_string = str(self.sent_packet_counter)
        message_string += ";"

        message_string += str(self.is_obstacle_in_front())
        message_string += ";"
        message_string += str(self.is_obstacle_in_right())
        message_string += ";"
        message_string += str(self.is_obstacle_in_back())
        message_string += ";"
        message_string += str(self.is_obstacle_in_left())

        message_string += ";"
        message_string += str(self.distance_to_goal)
        message_string += ";"
        message_string += str(self.angle_to_goal)

        message_string += ";"
        message_string += str(self.distance_to_obstacle_in_front)
        message_string += ";"
        message_string += str(self.distance_to_obstacle_in_right)
        message_string += ";"
        message_string += str(self.distance_to_obstacle_in_back)
        message_string += ";"
        message_string += str(self.distance_to_obstacle_in_left)

        message_string += ";"
        message_string += str(self.is_obstacle_in_front_right())
        message_string += ";"
        message_string += str(self.is_obstacle_in_front_front())
        message_string += ";"
        message_string += str(self.is_obstacle_in_front_left())

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

        if time_since_last_packet > self.receiving_packet_timeout and self.received_packet_counter != -1:

            self.last_packet_received_time = time.time()
            self.received_packet_counter = -1
            print "Reseting scenario due to lack of commands"
            self.reset_blocky_state()


        try:

            while True:

                data, addr = self.udp_reveiver_socket.recvfrom(buffer_size) 
                
                if self.blocky_udp_ip != addr[0]:

                    print "Changing Blockly UDP IP from " + self.blocky_udp_ip + " to " + addr[0]

                    self.blocky_udp_ip = addr[0]

                tokens = data.split(';')

                packet_number = int(tokens[0])

                if packet_number <= self.received_packet_counter:

                    # Out of order packet, ignore    
                    print "BlockyModule received out of order packet"                
                    continue

                self.last_packet_received_time = time.time()

                self.received_packet_counter = packet_number

                if tokens[1] == '-1':

                    self.set_blockly_vehicle_speed(-5)

                elif tokens[1] == '0':

                    self.set_blockly_vehicle_speed(0)

                else:

                    self.set_blockly_vehicle_speed(5)

                if tokens[2] == '-1':

                    self.set_blockly_vehicle_steering_degrees(-30)

                elif tokens[2] == '0':

                    self.set_blockly_vehicle_steering_degrees(0)

                else:

                    self.set_blockly_vehicle_steering_degrees(30)

        except socket.timeout:

            # If nothing to receive a timeout exception will 
            # be thrown, simply ignore this exception
            return

        return

    def get_qualisys_big_box_shapely_poly(self, qualisys_big_box):
      
        box_x_length = 0.6*32.
        box_y_length = 0.4*32.

        return self.get_qualisys_box_shapely_poly(qualisys_big_box, box_x_length, box_y_length)

    def get_qualisys_small_box_shapely_poly(self, qualisys_big_box):
        
        box_x_length = 0.4*32.
        box_y_length = 0.3*32.

        return self.get_qualisys_box_shapely_poly(qualisys_big_box, box_x_length, box_y_length)

    def get_qualisys_box_shapely_poly(self, qualisys_big_box, box_x_length, box_y_length):

        x = qualisys_big_box.x
        y = qualisys_big_box.y
        yaw = qualisys_big_box.yaw

        yaw_radians = math.radians(yaw)

        pt1 = (x, y)

        [x_r, y_r] = self.rotate_point_by_yaw_radians(box_x_length, 0, yaw_radians)
        pt2 = ( x + x_r,  y + y_r)

        [x_r, y_r] = self.rotate_point_by_yaw_radians(box_x_length, box_y_length, yaw_radians)
        pt3 = ( x + x_r,  y + y_r)

        [x_r, y_r] = self.rotate_point_by_yaw_radians(0, box_y_length, yaw_radians)
        pt4 = ( x + x_r,  y + y_r)

        box_polygon = shapely.geometry.Polygon([pt1, pt2, pt3, pt4])

        return box_polygon

    def get_front_car_region_shapely_poly(self, car_x, car_y, car_yaw_radians):

        # Front car region will be a box located in front of the car
        box_width = 20
        box_length = 25

        pt_1 = [0, -box_width/2.]
        pt_2 = [box_length, -box_width/2.]
        pt_3 = [box_length, box_width/2.]
        pt_4 = [0, box_width/2.]

        [pt_1_x, pt_1_y] = self.rotate_point_by_yaw_radians(pt_1[0], pt_1[1], car_yaw_radians)
        pt_1_x = self.blockly_vehicle.x + pt_1_x
        pt_1_y = self.blockly_vehicle.y + pt_1_y

        [pt_2_x, pt_2_y] = self.rotate_point_by_yaw_radians(pt_2[0], pt_2[1], car_yaw_radians)
        pt_2_x = self.blockly_vehicle.x + pt_2_x
        pt_2_y = self.blockly_vehicle.y + pt_2_y

        [pt_3_x, pt_3_y] = self.rotate_point_by_yaw_radians(pt_3[0], pt_3[1], car_yaw_radians)
        pt_3_x = self.blockly_vehicle.x + pt_3_x
        pt_3_y = self.blockly_vehicle.y + pt_3_y

        [pt_4_x, pt_4_y] = self.rotate_point_by_yaw_radians(pt_4[0], pt_4[1], car_yaw_radians)
        pt_4_x = self.blockly_vehicle.x + pt_4_x
        pt_4_y = self.blockly_vehicle.y + pt_4_y

        pt1 = (pt_1_x, pt_1_y)
        pt2 = (pt_2_x, pt_2_y)
        pt3 = (pt_3_x, pt_3_y)
        pt4 = (pt_4_x, pt_4_y)
        
        car_region_front = shapely.geometry.Polygon([pt1, pt2, pt3, pt4])

        return car_region_front

    def get_front_center_car_region_shapely_poly(self, car_x, car_y, car_yaw_radians):

        # Front car region will be a box located in front of the car
        # box_width = 2
        box_width = 4
        box_length = 10
        # box_length = 20

        pt_1 = [0, -box_width/2.]
        pt_2 = [box_length, -box_width/2.]
        pt_3 = [box_length, box_width/2.]
        pt_4 = [0, box_width/2.]

        [pt_1_x, pt_1_y] = self.rotate_point_by_yaw_radians(pt_1[0], pt_1[1], car_yaw_radians)
        pt_1_x = self.blockly_vehicle.x + pt_1_x
        pt_1_y = self.blockly_vehicle.y + pt_1_y

        [pt_2_x, pt_2_y] = self.rotate_point_by_yaw_radians(pt_2[0], pt_2[1], car_yaw_radians)
        pt_2_x = self.blockly_vehicle.x + pt_2_x
        pt_2_y = self.blockly_vehicle.y + pt_2_y

        [pt_3_x, pt_3_y] = self.rotate_point_by_yaw_radians(pt_3[0], pt_3[1], car_yaw_radians)
        pt_3_x = self.blockly_vehicle.x + pt_3_x
        pt_3_y = self.blockly_vehicle.y + pt_3_y

        [pt_4_x, pt_4_y] = self.rotate_point_by_yaw_radians(pt_4[0], pt_4[1], car_yaw_radians)
        pt_4_x = self.blockly_vehicle.x + pt_4_x
        pt_4_y = self.blockly_vehicle.y + pt_4_y

        pt1 = (pt_1_x, pt_1_y)
        pt2 = (pt_2_x, pt_2_y)
        pt3 = (pt_3_x, pt_3_y)
        pt4 = (pt_4_x, pt_4_y)
        
        car_region_front = shapely.geometry.Polygon([pt1, pt2, pt3, pt4])

        return car_region_front

    def get_front_right_car_region_shapely_poly(self, car_x, car_y, car_yaw_radians):

        # Front car region will be a box located in front of the car
        # box_width = 2
        # box_length = 10
        box_width = 4
        box_length = 10

        pt_1 = [0, -box_width*(3./2.)]
        pt_2 = [box_length, -box_width*(3./2.)]
        pt_3 = [box_length, -box_width/2.]
        pt_4 = [0, -box_width/2.]

        [pt_1_x, pt_1_y] = self.rotate_point_by_yaw_radians(pt_1[0], pt_1[1], car_yaw_radians)
        pt_1_x = self.blockly_vehicle.x + pt_1_x
        pt_1_y = self.blockly_vehicle.y + pt_1_y

        [pt_2_x, pt_2_y] = self.rotate_point_by_yaw_radians(pt_2[0], pt_2[1], car_yaw_radians)
        pt_2_x = self.blockly_vehicle.x + pt_2_x
        pt_2_y = self.blockly_vehicle.y + pt_2_y

        [pt_3_x, pt_3_y] = self.rotate_point_by_yaw_radians(pt_3[0], pt_3[1], car_yaw_radians)
        pt_3_x = self.blockly_vehicle.x + pt_3_x
        pt_3_y = self.blockly_vehicle.y + pt_3_y

        [pt_4_x, pt_4_y] = self.rotate_point_by_yaw_radians(pt_4[0], pt_4[1], car_yaw_radians)
        pt_4_x = self.blockly_vehicle.x + pt_4_x
        pt_4_y = self.blockly_vehicle.y + pt_4_y

        pt1 = (pt_1_x, pt_1_y)
        pt2 = (pt_2_x, pt_2_y)
        pt3 = (pt_3_x, pt_3_y)
        pt4 = (pt_4_x, pt_4_y)
        
        car_region_front = shapely.geometry.Polygon([pt1, pt2, pt3, pt4])

        return car_region_front

    def get_front_left_car_region_shapely_poly(self, car_x, car_y, car_yaw_radians):

        # Front car region will be a box located in front of the car
        # box_width = 2
        # box_length = 10
        box_width = 4
        box_length = 10

        pt_1 = [0, box_width*(3./2.)]
        pt_2 = [box_length, box_width*(3./2.)]
        pt_3 = [box_length, box_width/2.]
        pt_4 = [0, box_width/2.]

        [pt_1_x, pt_1_y] = self.rotate_point_by_yaw_radians(pt_1[0], pt_1[1], car_yaw_radians)
        pt_1_x = self.blockly_vehicle.x + pt_1_x
        pt_1_y = self.blockly_vehicle.y + pt_1_y

        [pt_2_x, pt_2_y] = self.rotate_point_by_yaw_radians(pt_2[0], pt_2[1], car_yaw_radians)
        pt_2_x = self.blockly_vehicle.x + pt_2_x
        pt_2_y = self.blockly_vehicle.y + pt_2_y

        [pt_3_x, pt_3_y] = self.rotate_point_by_yaw_radians(pt_3[0], pt_3[1], car_yaw_radians)
        pt_3_x = self.blockly_vehicle.x + pt_3_x
        pt_3_y = self.blockly_vehicle.y + pt_3_y

        [pt_4_x, pt_4_y] = self.rotate_point_by_yaw_radians(pt_4[0], pt_4[1], car_yaw_radians)
        pt_4_x = self.blockly_vehicle.x + pt_4_x
        pt_4_y = self.blockly_vehicle.y + pt_4_y

        pt1 = (pt_1_x, pt_1_y)
        pt2 = (pt_2_x, pt_2_y)
        pt3 = (pt_3_x, pt_3_y)
        pt4 = (pt_4_x, pt_4_y)
        
        car_region_front = shapely.geometry.Polygon([pt1, pt2, pt3, pt4])

        return car_region_front

    def get_back_car_region_shapely_poly(self, car_x, car_y, car_yaw_radians):

        # Front car region will be a box located in front of the car
        box_width = 4
        box_length = 10

        pt_1 = [0, -box_width/2.]
        pt_2 = [-box_length, -box_width/2.]
        pt_3 = [-box_length, box_width/2.]
        pt_4 = [0, box_width/2.]

        [pt_1_x, pt_1_y] = self.rotate_point_by_yaw_radians(pt_1[0], pt_1[1], car_yaw_radians)
        pt_1_x = self.blockly_vehicle.x + pt_1_x
        pt_1_y = self.blockly_vehicle.y + pt_1_y

        [pt_2_x, pt_2_y] = self.rotate_point_by_yaw_radians(pt_2[0], pt_2[1], car_yaw_radians)
        pt_2_x = self.blockly_vehicle.x + pt_2_x
        pt_2_y = self.blockly_vehicle.y + pt_2_y

        [pt_3_x, pt_3_y] = self.rotate_point_by_yaw_radians(pt_3[0], pt_3[1], car_yaw_radians)
        pt_3_x = self.blockly_vehicle.x + pt_3_x
        pt_3_y = self.blockly_vehicle.y + pt_3_y

        [pt_4_x, pt_4_y] = self.rotate_point_by_yaw_radians(pt_4[0], pt_4[1], car_yaw_radians)
        pt_4_x = self.blockly_vehicle.x + pt_4_x
        pt_4_y = self.blockly_vehicle.y + pt_4_y

        pt1 = (pt_1_x, pt_1_y)
        pt2 = (pt_2_x, pt_2_y)
        pt3 = (pt_3_x, pt_3_y)
        pt4 = (pt_4_x, pt_4_y)
        
        car_region_back = shapely.geometry.Polygon([pt1, pt2, pt3, pt4])

        return car_region_back

    def get_right_car_region_shapely_poly(self, car_x, car_y, car_yaw_radians):

        # Front car region will be a box located in front of the car
        box_width = 20
        box_length = 4

        pt_1 = [-box_length/2, -box_width]
        pt_2 = [-box_length/2, 0]
        pt_3 = [box_length/2, 0]
        pt_4 = [box_length/2, -box_width]

        [pt_1_x, pt_1_y] = self.rotate_point_by_yaw_radians(pt_1[0], pt_1[1], car_yaw_radians)
        pt_1_x = self.blockly_vehicle.x + pt_1_x
        pt_1_y = self.blockly_vehicle.y + pt_1_y

        [pt_2_x, pt_2_y] = self.rotate_point_by_yaw_radians(pt_2[0], pt_2[1], car_yaw_radians)
        pt_2_x = self.blockly_vehicle.x + pt_2_x
        pt_2_y = self.blockly_vehicle.y + pt_2_y

        [pt_3_x, pt_3_y] = self.rotate_point_by_yaw_radians(pt_3[0], pt_3[1], car_yaw_radians)
        pt_3_x = self.blockly_vehicle.x + pt_3_x
        pt_3_y = self.blockly_vehicle.y + pt_3_y

        [pt_4_x, pt_4_y] = self.rotate_point_by_yaw_radians(pt_4[0], pt_4[1], car_yaw_radians)
        pt_4_x = self.blockly_vehicle.x + pt_4_x
        pt_4_y = self.blockly_vehicle.y + pt_4_y

        pt1 = (pt_1_x, pt_1_y)
        pt2 = (pt_2_x, pt_2_y)
        pt3 = (pt_3_x, pt_3_y)
        pt4 = (pt_4_x, pt_4_y)
        
        car_region_right = shapely.geometry.Polygon([pt1, pt2, pt3, pt4])

        return car_region_right

    def get_left_car_region_shapely_poly(self, car_x, car_y, car_yaw_radians):

        # Front car region will be a box located in front of the car
        box_width = 20
        box_length = 4

        pt_1 = [-box_length/2, box_width]
        pt_2 = [-box_length/2, 0]
        pt_3 = [box_length/2, 0]
        pt_4 = [box_length/2, box_width]

        [pt_1_x, pt_1_y] = self.rotate_point_by_yaw_radians(pt_1[0], pt_1[1], car_yaw_radians)
        pt_1_x = self.blockly_vehicle.x + pt_1_x
        pt_1_y = self.blockly_vehicle.y + pt_1_y

        [pt_2_x, pt_2_y] = self.rotate_point_by_yaw_radians(pt_2[0], pt_2[1], car_yaw_radians)
        pt_2_x = self.blockly_vehicle.x + pt_2_x
        pt_2_y = self.blockly_vehicle.y + pt_2_y

        [pt_3_x, pt_3_y] = self.rotate_point_by_yaw_radians(pt_3[0], pt_3[1], car_yaw_radians)
        pt_3_x = self.blockly_vehicle.x + pt_3_x
        pt_3_y = self.blockly_vehicle.y + pt_3_y

        [pt_4_x, pt_4_y] = self.rotate_point_by_yaw_radians(pt_4[0], pt_4[1], car_yaw_radians)
        pt_4_x = self.blockly_vehicle.x + pt_4_x
        pt_4_y = self.blockly_vehicle.y + pt_4_y

        pt1 = (pt_1_x, pt_1_y)
        pt2 = (pt_2_x, pt_2_y)
        pt3 = (pt_3_x, pt_3_y)
        pt4 = (pt_4_x, pt_4_y)
        
        car_region_left = shapely.geometry.Polygon([pt1, pt2, pt3, pt4])

        return car_region_left

    def get_distance_to_obstacle_along_direction(self, car_x, car_y, car_yaw_radians, list_poly_obstacles, direction):

        distance = 50

        check_radius = 2.5

        if direction == 'front':

            x_multiplier = 1
            y_multiplier = 0

            check_radius = 1

        elif direction == 'right':

            x_multiplier = 0
            y_multiplier = -1

        elif direction == 'back':

            x_multiplier = -1
            y_multiplier = 0

        elif direction == 'left':

            x_multiplier = 0
            y_multiplier = 1

        else:

            raise NameError('Unexpected direction parameter')

        distance_step = 2

        for current_distance in range(distance_step, 50, distance_step):

            [pt_x, pt_y] = self.rotate_point_by_yaw_radians(x_multiplier*current_distance, y_multiplier*current_distance, car_yaw_radians)
            pt_x = car_x + pt_x
            pt_y = car_y + pt_y

            point_to_check = shapely.geometry.Point(pt_x, pt_y)

            for poly_obstacle in list_poly_obstacles:

                circle_check = point_to_check.buffer(check_radius, resolution=4, cap_style=1, join_style=1, mitre_limit=1.0)

                # if point_to_check.within(poly_obstacle):
                if circle_check.intersects(poly_obstacle):

                    distance = current_distance

                    if direction == 'front':

                        return distance - 5

                    return distance

        return distance


    def rotate_point_by_yaw_radians(self, x, y, yaw_radians):

        x_r = math.cos(yaw_radians)*x - math.sin(yaw_radians)*y
        y_r = math.sin(yaw_radians)*x + math.cos(yaw_radians)*y

        return [x_r, y_r]

    def update_surroundings(self):

        box_polys_list = []

        qualisys_goal_body = None

        safety_distance = 50

        car_point = shapely.geometry.Point(self.blockly_vehicle.x, self.blockly_vehicle.y)

        for body_id in self.sml_world.bodies_dict:

            if isinstance(self.sml_world.bodies_dict[body_id], bodyclasses.QualisysBigBox):

                box_poly = self.get_qualisys_big_box_shapely_poly(self.sml_world.bodies_dict[body_id])

                distance_to_box = car_point.distance(box_poly)

                if distance_to_box < safety_distance:

                    box_polys_list.append( box_poly )

            if isinstance(self.sml_world.bodies_dict[body_id], bodyclasses.QualisysSmallBox):

                box_poly = self.get_qualisys_small_box_shapely_poly(self.sml_world.bodies_dict[body_id])

                distance_to_box = car_point.distance(box_poly)

                if distance_to_box < safety_distance:

                    box_polys_list.append( box_poly )

            if isinstance(self.sml_world.bodies_dict[body_id], bodyclasses.QualisysGoal):

                qualisys_goal_body = self.sml_world.bodies_dict[body_id]

        if qualisys_goal_body:

            self.distance_to_goal = math.hypot(qualisys_goal_body.y - self.blockly_vehicle.y, qualisys_goal_body.x - self.blockly_vehicle.x)
            self.distance_to_goal = int(round(self.distance_to_goal))


            self.angle_to_goal = math.atan2(qualisys_goal_body.y - self.blockly_vehicle.y, qualisys_goal_body.x - self.blockly_vehicle.x)
            self.angle_to_goal -= math.radians(self.blockly_vehicle.yaw)

            while self.angle_to_goal > math.pi:

                self.angle_to_goal -= 2.0*math.pi

            while self.angle_to_goal < -math.pi:

                self.angle_to_goal += 2.0*math.pi

            self.angle_to_goal = math.degrees(self.angle_to_goal)

            # Discretize the angle to the goal to angle_step
            angle_step = 15

            if self.angle_to_goal < angle_step and self.angle_to_goal > -angle_step:

                self.angle_to_goal = 0

            else:

                self.angle_to_goal = int(round(self.angle_to_goal))

                self.angle_to_goal = self.angle_to_goal/angle_step

                self.angle_to_goal = self.angle_to_goal*angle_step

        else:

            self.distance_to_goal = 100
            self.angle_to_goal = 0


        yaw_radians = math.radians(self.blockly_vehicle.yaw)

        self.distance_to_obstacle_in_front = self.get_distance_to_obstacle_along_direction(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians, box_polys_list, 'front')
        self.distance_to_obstacle_in_right = self.get_distance_to_obstacle_along_direction(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians, box_polys_list, 'right')
        self.distance_to_obstacle_in_back = self.get_distance_to_obstacle_along_direction(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians, box_polys_list, 'back')
        self.distance_to_obstacle_in_left = self.get_distance_to_obstacle_along_direction(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians, box_polys_list, 'left')

        # print "self.distance_to_obstacle_in_front = " + str(self.distance_to_obstacle_in_front)
        # print "self.distance_to_obstacle_in_right = " + str(self.distance_to_obstacle_in_right)
        # print "self.distance_to_obstacle_in_back = " + str(self.distance_to_obstacle_in_back)
        # print "self.distance_to_obstacle_in_left = " + str(self.distance_to_obstacle_in_left)

        self.obstacle_in_front = False
        self.obstacle_in_right = False
        self.obstacle_in_back = False
        self.obstacle_in_left = False

        self.obstacle_in_front_right = False
        self.obstacle_in_front_front = False
        self.obstacle_in_front_left = False

        obstacle_distance = 20.

        front_car_region_poly = self.get_front_car_region_shapely_poly(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians)
        right_car_region_poly = self.get_right_car_region_shapely_poly(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians)
        back_car_region_poly = self.get_back_car_region_shapely_poly(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians)
        left_car_region_poly = self.get_left_car_region_shapely_poly(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians)


        front_right_car_region_poly = self.get_front_right_car_region_shapely_poly(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians)
        front_front_car_region_poly = self.get_front_center_car_region_shapely_poly(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians)
        front_left_car_region_poly = self.get_front_left_car_region_shapely_poly(self.blockly_vehicle.x, self.blockly_vehicle.y, yaw_radians)


        for box_poly in box_polys_list:

            # print "box_poly = " + str(box_poly)
            # print "point_in_front = " + str(point_in_front)

            # print "front_car_region_poly = " + str(front_car_region_poly)
            
            if not self.obstacle_in_front:

                if front_car_region_poly.intersects(box_poly):

                    self.obstacle_in_front = True

            if not self.obstacle_in_right:

                if right_car_region_poly.intersects(box_poly):

                    self.obstacle_in_right = True

            if not self.obstacle_in_back:

                if back_car_region_poly.intersects(box_poly):

                    self.obstacle_in_back = True

            if not self.obstacle_in_left:

                if left_car_region_poly.intersects(box_poly):

                    self.obstacle_in_left = True

            if not self.obstacle_in_front_right:

                if front_right_car_region_poly.intersects(box_poly):

                    self.obstacle_in_front_right = True

            if not self.obstacle_in_front_front:

                if front_front_car_region_poly.intersects(box_poly):

                    self.obstacle_in_front_front = True

            if not self.obstacle_in_front_left:

                if front_left_car_region_poly.intersects(box_poly):

                    self.obstacle_in_front_left = True

            pass

        # print ""

        # print "self.distance_to_goal = " + str(self.distance_to_goal)

        # print "self.angle_to_goal = " + str(self.angle_to_goal)

        # if self.obstacle_in_front:

        #     print "Obstacle in front"

        # if self.obstacle_in_right:

        #     print "Obstacle in right"

        # if self.obstacle_in_back:

        #     print "Obstacle in back"

        # if self.obstacle_in_left:

        #     print "Obstacle in left"

        # print "self.obstacle_in_front_right = " + str(self.obstacle_in_front_right)
        # print "self.obstacle_in_front_front = " + str(self.obstacle_in_front_front)
        # print "self.obstacle_in_front_left = " + str(self.obstacle_in_front_left)

        return


    def is_obstacle_in_front(self):

        return self.obstacle_in_front

    def is_obstacle_in_right(self):

        return self.obstacle_in_right

    def is_obstacle_in_back(self):

        return self.obstacle_in_back

    def is_obstacle_in_left(self):

        return self.obstacle_in_left

    def is_obstacle_in_front_right(self):

        return self.obstacle_in_front_right

    def is_obstacle_in_front_front(self):

        return self.obstacle_in_front_front

    def is_obstacle_in_front_left(self):

        return self.obstacle_in_front_left


    def set_up_blocky_level_1(self):

        qualisys_goal = bodyclasses.QualisysGoal()

        qualisys_goal.x = 50
        qualisys_goal.y = 0
        qualisys_goal.yaw = 0
        qualisys_goal.id = 6

        self.sml_world.bodies_dict[qualisys_goal.id] = qualisys_goal

        initial_blockly_x = -50
        initial_blockly_y = 0
        initial_blockly_yaw = 0

        self.set_initial_blockly_vehicle_state(initial_blockly_x, initial_blockly_y, initial_blockly_yaw)

        self.reset_blocky_state()

        return

    def set_up_blocky_level_2(self):

        qualisys_goal = bodyclasses.QualisysGoal()

        qualisys_goal.x = 50
        qualisys_goal.y = 70
        qualisys_goal.yaw = 0
        qualisys_goal.id = 6

        self.sml_world.bodies_dict[qualisys_goal.id] = qualisys_goal

        initial_blockly_x = -50
        initial_blockly_y = 0
        initial_blockly_yaw = 0

        self.set_initial_blockly_vehicle_state(initial_blockly_x, initial_blockly_y, initial_blockly_yaw)

        self.reset_blocky_state()

        return

    def set_up_blocky_level_3(self):

        qualisys_box_1 = bodyclasses.QualisysBigBox()

        qualisys_box_1.x = -0.3*32.
        qualisys_box_1.y = -0.2*32.
        qualisys_box_1.yaw = 0
        qualisys_box_1.id = 21

        self.sml_world.bodies_dict[qualisys_box_1.id] = qualisys_box_1

        qualisys_goal = bodyclasses.QualisysGoal()

        qualisys_goal.x = 50
        qualisys_goal.y = 0
        qualisys_goal.yaw = 0
        qualisys_goal.id = 6

        self.sml_world.bodies_dict[qualisys_goal.id] = qualisys_goal

        initial_blockly_x = -50
        initial_blockly_y = 0
        initial_blockly_yaw = 0

        self.set_initial_blockly_vehicle_state(initial_blockly_x, initial_blockly_y, initial_blockly_yaw)

        self.reset_blocky_state()

        return

    def set_up_blocky_level_4(self):

        delta_yaw_degrees_1 = -20
        delta_yaw_degrees_2 = -15

        qualisys_box_1 = bodyclasses.QualisysBigBox()

        qualisys_box_1.x = -0.2*32.
        qualisys_box_1.y = -0.6*32.
        qualisys_box_1.yaw = 0
        qualisys_box_1.id = 21

        self.sml_world.bodies_dict[qualisys_box_1.id] = qualisys_box_1

        qualisys_box_2 = bodyclasses.QualisysBigBox()

        qualisys_box_2.x = -0.2*32.
        qualisys_box_2.y = 0.2*32.
        qualisys_box_2.yaw = 0
        qualisys_box_2.id = 22

        self.sml_world.bodies_dict[qualisys_box_2.id] = qualisys_box_2

        qualisys_box_3 = bodyclasses.QualisysBigBox()

        # qualisys_box_3.x = 0.4*32.
        qualisys_box_3.x = qualisys_box_1.x + math.cos(math.radians(qualisys_box_1.yaw))*0.6*32
        # qualisys_box_3.y = -0.6*32.
        qualisys_box_3.y = qualisys_box_1.y + math.sin(math.radians(qualisys_box_1.yaw))*0.6*32
        qualisys_box_3.yaw = delta_yaw_degrees_1
        qualisys_box_3.id = 23

        self.sml_world.bodies_dict[qualisys_box_3.id] = qualisys_box_3

        qualisys_box_4 = bodyclasses.QualisysBigBox()

        qualisys_box_4.x = qualisys_box_2.x + math.cos(math.radians(qualisys_box_2.yaw))*0.6*32
        qualisys_box_4.y = qualisys_box_2.y + math.sin(math.radians(qualisys_box_2.yaw))*0.6*32
        qualisys_box_4.yaw = delta_yaw_degrees_2
        qualisys_box_4.id = 24

        self.sml_world.bodies_dict[qualisys_box_4.id] = qualisys_box_4

        qualisys_box_5 = bodyclasses.QualisysBigBox()

        # qualisys_box_5.x = 1.0*32.
        qualisys_box_5.x = qualisys_box_3.x + math.cos(math.radians(qualisys_box_3.yaw))*0.6*32
        # qualisys_box_5.y = -0.6*32.
        qualisys_box_5.y = qualisys_box_3.y + math.sin(math.radians(qualisys_box_3.yaw))*0.6*32
        qualisys_box_5.yaw = delta_yaw_degrees_1*2.
        qualisys_box_5.id = 25

        self.sml_world.bodies_dict[qualisys_box_5.id] = qualisys_box_5

        qualisys_box_6 = bodyclasses.QualisysBigBox()

        qualisys_box_6.x = qualisys_box_4.x + math.cos(math.radians(qualisys_box_4.yaw))*0.6*32
        qualisys_box_6.y = qualisys_box_4.y + math.sin(math.radians(qualisys_box_4.yaw))*0.6*32
        qualisys_box_6.yaw = delta_yaw_degrees_2*2.
        qualisys_box_6.id = 26

        self.sml_world.bodies_dict[qualisys_box_6.id] = qualisys_box_6

        qualisys_box_7 = bodyclasses.QualisysBigBox()

        qualisys_box_7.x = qualisys_box_5.x + math.cos(math.radians(qualisys_box_5.yaw))*0.6*32
        qualisys_box_7.y = qualisys_box_5.y + math.sin(math.radians(qualisys_box_5.yaw))*0.6*32
        qualisys_box_7.yaw = delta_yaw_degrees_1*3.
        qualisys_box_7.id = 27

        self.sml_world.bodies_dict[qualisys_box_7.id] = qualisys_box_7

        qualisys_box_8 = bodyclasses.QualisysBigBox()

        qualisys_box_8.x = qualisys_box_6.x + math.cos(math.radians(qualisys_box_6.yaw))*0.6*32
        qualisys_box_8.y = qualisys_box_6.y + math.sin(math.radians(qualisys_box_6.yaw))*0.6*32
        qualisys_box_8.yaw = delta_yaw_degrees_2*3.
        qualisys_box_8.id = 28

        self.sml_world.bodies_dict[qualisys_box_8.id] = qualisys_box_8

        delta_yaw_degrees_1 = 35
        delta_yaw_degrees_2 = 20

        qualisys_box_9 = bodyclasses.QualisysBigBox()

        qualisys_box_9.yaw = qualisys_box_7.yaw + delta_yaw_degrees_1
        qualisys_box_9.x = qualisys_box_7.x + math.cos(math.radians(qualisys_box_7.yaw))*0.6*32 - math.sin(math.radians(qualisys_box_7.yaw))*0.4*32 + math.sin(math.radians(qualisys_box_9.yaw))*0.4*32
        qualisys_box_9.y = qualisys_box_7.y + math.sin(math.radians(qualisys_box_7.yaw))*0.6*32 + math.cos(math.radians(qualisys_box_7.yaw))*0.4*32 - math.cos(math.radians(qualisys_box_9.yaw))*0.4*32
        
        qualisys_box_9.id = 29
        self.sml_world.bodies_dict[qualisys_box_9.id] = qualisys_box_9

        qualisys_box_10 = bodyclasses.QualisysBigBox()

        qualisys_box_10.yaw =  qualisys_box_8.yaw + delta_yaw_degrees_2
        qualisys_box_10.x = qualisys_box_8.x + math.cos(math.radians(qualisys_box_8.yaw))*0.6*32 - math.sin(math.radians(qualisys_box_8.yaw))*0.4*32 + math.sin(math.radians(qualisys_box_10.yaw))*0.4*32
        qualisys_box_10.y = qualisys_box_8.y +  math.sin(math.radians(qualisys_box_8.yaw))*0.6*32 + math.cos(math.radians(qualisys_box_8.yaw))*0.4*32 - math.cos(math.radians(qualisys_box_10.yaw))*0.4*32
        qualisys_box_10.id = 30

        self.sml_world.bodies_dict[qualisys_box_10.id] = qualisys_box_10

        # qualisys_box_11 = bodyclasses.QualisysBigBox()

        # qualisys_box_11.x = qualisys_box_9.x + math.cos(math.radians(qualisys_box_9.yaw))*0.6*32
        # qualisys_box_11.y = qualisys_box_9.y + math.sin(math.radians(qualisys_box_9.yaw))*0.6*32
        # qualisys_box_11.yaw = qualisys_box_9.yaw + delta_yaw_degrees_1
        # qualisys_box_11.id = 31
        # self.sml_world.bodies_dict[qualisys_box_11.id] = qualisys_box_11

        # qualisys_box_12 = bodyclasses.QualisysBigBox()

        # qualisys_box_12.x = qualisys_box_10.x + math.cos(math.radians(qualisys_box_10.yaw))*0.6*32
        # qualisys_box_12.y = qualisys_box_10.y + math.sin(math.radians(qualisys_box_10.yaw))*0.6*32
        # qualisys_box_12.yaw =  qualisys_box_10.yaw + delta_yaw_degrees_2
        # qualisys_box_12.id = 32

        # self.sml_world.bodies_dict[qualisys_box_12.id] = qualisys_box_12




        qualisys_goal = bodyclasses.QualisysGoal()

        qualisys_goal.x = 75
        qualisys_goal.y = -50
        qualisys_goal.yaw = 0
        qualisys_goal.id = 6

        self.sml_world.bodies_dict[qualisys_goal.id] = qualisys_goal

        initial_blockly_x = -25
        initial_blockly_y = 0
        initial_blockly_yaw = 0

        self.set_initial_blockly_vehicle_state(initial_blockly_x, initial_blockly_y, initial_blockly_yaw)

        self.reset_blocky_state()

        return

    def set_up_blocky_level_5(self):

        num_boxes = 20

        cnt = 1

        length_containers = 200.
        containers_amplitude = 20.
        containers_offset = 10


        for i in range(num_boxes):

            frac = float(i)/num_boxes

            x_pos = -length_containers + frac*length_containers*2
            y_pos = containers_offset + containers_amplitude*math.sin( frac*2*math.pi )

            qualisys_box = bodyclasses.QualisysBigBox()
            qualisys_box.x = x_pos
            qualisys_box.y = y_pos
            # qualisys_box.yaw = 360*frac           
            qualisys_box.yaw = 0
            qualisys_box.id = 20 + 2*cnt - 1

            self.sml_world.bodies_dict[qualisys_box.id] = qualisys_box

            x_pos = -length_containers + frac*length_containers*2
            y_pos = -containers_offset + containers_amplitude*math.sin( frac*2*math.pi ) -0.4*32

            qualisys_box = bodyclasses.QualisysBigBox()
            qualisys_box.x = x_pos
            qualisys_box.y = y_pos
            # qualisys_box.yaw = 360*frac           
            qualisys_box.yaw = 0
            qualisys_box.id = 20 + 2*cnt

            self.sml_world.bodies_dict[qualisys_box.id] = qualisys_box

            cnt += 1

        x_pos = -length_containers - 0.6*32
        y_pos = containers_offset

        qualisys_box = bodyclasses.QualisysBigBox()
        qualisys_box.x = x_pos
        qualisys_box.y = y_pos
        # qualisys_box.yaw = 360*frac           
        qualisys_box.yaw = 0
        qualisys_box.id = 20 + 2*cnt - 1

        self.sml_world.bodies_dict[qualisys_box.id] = qualisys_box

        x_pos = -length_containers - 0.6*32
        y_pos = -containers_offset + -0.4*32

        qualisys_box = bodyclasses.QualisysBigBox()
        qualisys_box.x = x_pos
        qualisys_box.y = y_pos
        # qualisys_box.yaw = 360*frac           
        qualisys_box.yaw = 0
        qualisys_box.id = 20 + 2*cnt

        self.sml_world.bodies_dict[qualisys_box.id] = qualisys_box

        cnt += 1



        x_pos = -length_containers + frac*length_containers*2 + 0.6*32
        y_pos = containers_offset + 20*math.sin( frac*2*math.pi )

        qualisys_box = bodyclasses.QualisysBigBox()
        qualisys_box.x = x_pos
        qualisys_box.y = y_pos
        # qualisys_box.yaw = 360*frac           
        qualisys_box.yaw = 0
        qualisys_box.id = 20 + 2*cnt - 1

        self.sml_world.bodies_dict[qualisys_box.id] = qualisys_box

        x_pos = -length_containers + frac*length_containers*2 + 0.6*32
        y_pos = -containers_offset + 20*math.sin( frac*2*math.pi ) -0.4*32

        qualisys_box = bodyclasses.QualisysBigBox()
        qualisys_box.x = x_pos
        qualisys_box.y = y_pos
        # qualisys_box.yaw = 360*frac           
        qualisys_box.yaw = 0
        qualisys_box.id = 20 + 2*cnt

        self.sml_world.bodies_dict[qualisys_box.id] = qualisys_box

        cnt += 1

    

        qualisys_goal = bodyclasses.QualisysGoal()

        qualisys_goal.x = length_containers
        qualisys_goal.y = 0
        qualisys_goal.yaw = 0
        qualisys_goal.id = 6

        self.sml_world.bodies_dict[qualisys_goal.id] = qualisys_goal

        initial_blockly_x = -length_containers
        initial_blockly_y = 0
        initial_blockly_yaw = 0

        self.set_initial_blockly_vehicle_state(initial_blockly_x, initial_blockly_y, initial_blockly_yaw)

        self.reset_blocky_state()

        return
