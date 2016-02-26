import time, math, threading, socket
import BlocklyVehicle
import bodyclasses
import shapely.geometry
import bodyclasses
import smartvehicle
import dummyvehicle

class BlocklyModuleTrafficLight:
    '''
    Class that interfaces with the Blocky Python Code.

    '''

    def __init__(self, sml_world, simulated = True):

        # Get a reference to the sml_world,
        # so that we are able to access the
        # bodies_dict
        self.sml_world = sml_world

        self.simulated = simulated

        self.loop_rate = 10.

        # To add an ever increasing packet
        # number to every packet sent
        self.sent_packet_counter = 0
        # Initialize the sender socket
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

        self.create_blocking_vehicles()
        self.blocking_vehicles = False
        self.remove_blocking_vehicles()

        self.horizontal_barrier = False
        self.vertical_barrier = False

        # self.put_horizontal_barrier()



    def create_blocking_vehicles(self):

        vehicle_id = -100

        for i in range(3):

            dummy_vehicle = dummyvehicle.DummyVehicle(self.sml_world, 10., vehicle_id)

            if i == 0:
                dummy_vehicle.set_trajectory('right')
            elif  i == 1:
                dummy_vehicle.set_trajectory('center')
            else:
                dummy_vehicle.set_trajectory('left')

            dummy_vehicle.set_vehicle_on_trajectory_state(len(dummy_vehicle.traj[0])-20)

            self.sml_world.bodies_dict[vehicle_id] = dummy_vehicle 

            vehicle_id -= 1

        for i in range(3):

            dummy_vehicle = dummyvehicle.DummyVehicle(self.sml_world, 10., vehicle_id)

            if i == 0:
                dummy_vehicle.set_trajectory('right')
                second_traffic_light_offset = 1440+40
            elif  i == 1:
                dummy_vehicle.set_trajectory('center')
                second_traffic_light_offset = 1350+40
            else:
                dummy_vehicle.set_trajectory('left')
                second_traffic_light_offset = 1250+40

            dummy_vehicle.set_vehicle_on_trajectory_state(second_traffic_light_offset)

            self.sml_world.bodies_dict[vehicle_id] = dummy_vehicle 

            vehicle_id -= 1

        self.remove_blocking_vehicles()

        return

    def put_blocking_vehicles(self):

        # vehicle_id = -100

        # for i in range(3):

        #     dummy_vehicle = self.sml_world.bodies_dict[vehicle_id]
        #     # dummy_vehicle.set_vehicle_on_trajectory_state(0)
        #     dummy_vehicle.set_vehicle_on_trajectory_state(len(dummy_vehicle.traj[0])-25)

        #     vehicle_id -= 1

        # for i in range(3):

        #     dummy_vehicle = self.sml_world.bodies_dict[vehicle_id]

        #     if i == 0:
        #         dummy_vehicle.set_trajectory('right')
        #         second_traffic_light_offset = 1470-25
        #     elif  i == 1:
        #         dummy_vehicle.set_trajectory('center')
        #         second_traffic_light_offset = 1380-25
        #     else:
        #         dummy_vehicle.set_trajectory('left')
        #         second_traffic_light_offset = 1280-25

        #     dummy_vehicle.set_vehicle_on_trajectory_state(second_traffic_light_offset)

        #     vehicle_id -= 1

        self.put_horizontal_barrier()
        self.put_vertical_barrier()

        self.blocking_vehicles = True

        return

    def put_horizontal_barrier(self):

        vehicle_id = -100

        for i in range(3):

            dummy_vehicle = self.sml_world.bodies_dict[vehicle_id]
            # dummy_vehicle.set_vehicle_on_trajectory_state(0)
            dummy_vehicle.set_vehicle_on_trajectory_state(len(dummy_vehicle.traj[0])-25)

            vehicle_id -= 1

        self.horizontal_barrier = True

        return

    def put_vertical_barrier(self):

        vehicle_id = -103

        for i in range(3):

            dummy_vehicle = self.sml_world.bodies_dict[vehicle_id]

            if i == 0:
                dummy_vehicle.set_trajectory('right')
                second_traffic_light_offset = 1470-25
            elif  i == 1:
                dummy_vehicle.set_trajectory('center')
                second_traffic_light_offset = 1380-25
            else:
                dummy_vehicle.set_trajectory('left')
                second_traffic_light_offset = 1280-25

            dummy_vehicle.set_vehicle_on_trajectory_state(second_traffic_light_offset)

            vehicle_id -= 1

        self.blocking_vehicles = True

        self.vertical_barrier = True

        return

    def remove_blocking_vehicles(self):

        # vehicle_id = -100

        # for i in range(3):

        #     dummy_vehicle = self.sml_world.bodies_dict[vehicle_id]
        #     dummy_vehicle.x = 1000.

        #     vehicle_id -= 1

        # for i in range(3):

        #     dummy_vehicle = self.sml_world.bodies_dict[vehicle_id]
        #     dummy_vehicle.x = 1000.            

        #     vehicle_id -= 1

        self.remove_horizontal_barrier()
        self.remove_vertical_barrier()

        self.blocking_vehicles = False

        return

    def remove_vertical_barrier(self):

        vehicle_id = -103

        for i in range(3):

            dummy_vehicle = self.sml_world.bodies_dict[vehicle_id]
            dummy_vehicle.x = 1000.            

            vehicle_id -= 1

        self.vertical_barrier = False

        return

    def remove_horizontal_barrier(self):

        vehicle_id = -100

        for i in range(3):

            dummy_vehicle = self.sml_world.bodies_dict[vehicle_id]
            dummy_vehicle.x = 1000.

            vehicle_id -= 1

        self.horizontal_barrier = False

        return

    def place_traffic_vehicles(self, num_traffic_vehicles = 40):

        '''
        This function will create and place the
        smart vehicles in the intersection map.
        '''

        # The desired number of vehicles to place
        num_traffic_vehicles = num_traffic_vehicles

        # The id of the vehicles, starting at -1, and 
        # decreasing for each vehicle placed.
        vehicle_id = -1

        for i in range(num_traffic_vehicles):


            dummy_vehicle = dummyvehicle.DummyVehicle(self.sml_world, 10., vehicle_id)

            if i % 3 == 0:
                dummy_vehicle.set_trajectory('right')
            elif  i % 3 == 1:
                dummy_vehicle.set_trajectory('center')
            else:
                dummy_vehicle.set_trajectory('left')

            dummy_vehicle.set_vehicle_on_trajectory_state(int( (-450*vehicle_id)%2800 ))

            self.sml_world.bodies_dict[vehicle_id] = dummy_vehicle 

            dummy_vehicle.start_control_loop()

            # For each vehicle placed, the vehicle id has to decrease
            vehicle_id -= 1

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

        last_time_removal = time.time()

        while not self.sml_world.close:

            start_loop_time = time.time()

            self.process_incoming_packets()

            end_loop_time = time.time()

            time_elapsed = end_loop_time - start_loop_time

            time_to_sleep = 1./self.loop_rate - time_elapsed

            # if time.time() - last_time_removal > 10.:

            #     # if self.blocking_vehicles:
            #     #     self.remove_blocking_vehicles()
            #     # else:
            #     #     self.put_blocking_vehicles()

            #     if self.horizontal_barrier:

            #         self.remove_horizontal_barrier()
            #         self.put_vertical_barrier()

            #     else:

            #         self.remove_vertical_barrier()
            #         self.put_horizontal_barrier()

            #     last_time_removal = time.time()


            if time_to_sleep > 0:

                time.sleep(time_to_sleep)

            else:

                print "BlocklyModule failed desired rate."

        self.close_sockets()

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

                is_red_light_on = int(tokens[2])

                if is_red_light_on == 1:

                    if not self.horizontal_barrier:

                        self.put_horizontal_barrier()

                else:

                    if self.horizontal_barrier:

                        self.remove_horizontal_barrier()


                is_bar_down = int(tokens[1])

                if is_bar_down == 1:

                    if not self.vertical_barrier:

                        self.put_vertical_barrier()

                else:

                    if self.vertical_barrier:

                        self.remove_vertical_barrier()


        except socket.timeout:

            # If nothing to receive a timeout exception will 
            # be thrown, simply ignore this exception
            return

        return

 
