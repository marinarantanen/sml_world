import time, math, threading, socket

class UnityModule:
    '''
    The UnityModule implementes the mechanisms to correctly
    communicate and interact with a Unity's Simulator.
    Unity's Simulator will be used to implement a driver
    in the loop to work together with the control algorithms.
    '''

    def __init__(self, sml_world, unity_address = 'localhost', desired_rate = 50.):

        # Get a reference to the sml_world,
        # so that we are able to interact with
        # it and its bodies_dict
        self.sml_world = sml_world

        # The ip address of the computer hostin
        # the Unity Simulator
        self.unity_address = unity_address

        # The loop rate of this classes thread loop
        self.loop_rate = 50.

        # To add an ever increasing packet
        # number to every packet sent
        self.sent_packet_counter = 0
        # Initialize the sender socket
        self.start_udp_sender_socket()

        # To keep track of the most recent
        # packet received, and to discart possible
        # our of order packets
        self.received_packet_counter = -1
        # Initialize the receiver socket
        self.start_udp_receiver_socket()

        # Launches the tread that will run the main
        # loop of the SimulatorModule
        t = threading.Thread( target = self.thread_loop, args=([]) )
        t.daemon = True
        t.start()

    def thread_loop(self):
        '''
        This function is executed in a thread, thus running 
        in parallel with the rest of the SML World.
        It consists of a loop, where messages with Unity
        are exchanged.
        '''

        # Time keepers
        start_loop_time = time.time()
        end_loop_time = time.time()

        while not self.sml_world.close:

            start_loop_time = time.time()

            self.send_vehicle_state_packet()    
            self.process_incoming_packets()

            end_loop_time = time.time()

            time_elapsed = end_loop_time - start_loop_time

            time_to_sleep = 1./self.loop_rate - time_elapsed

            if time_to_sleep > 0:

                time.sleep(time_to_sleep)

            else:

                print "UnityModule failed desired rate."

        self.close_sockets()

        return

    def start_udp_sender_socket(self):
        '''
        Initializes the sending socket, that
        will send the vehicle states to the
        Simulator.
        '''

        self.UDP_IP = self.unity_address
        self.UDP_PORT = 34005

        self.udp_sender_socket = socket.socket(socket.AF_INET, # Internet
        socket.SOCK_DGRAM) # UDP

    def start_udp_receiver_socket(self):
        '''
        Initializes the receiving socket, that
        will receive driver in the loop inputs
        (steering)
        '''

        UDP_IP = '' # Allows us to receive from any external PC
        UDP_PORT = 34006

        self.udp_reveiver_socket = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        self.udp_reveiver_socket.bind((UDP_IP, UDP_PORT))
        self.udp_reveiver_socket.settimeout(0.01)

    def close_sockets(self):
        '''
        Before exiting the program, free the sockets.
        '''

        self.udp_reveiver_socket.close()

    def get_vehicle_state_string(self, body_to_send):
        '''
        Given a Vehicle, it will transform its state
        into a string to be sent to Unity through UDP
        '''

        body_string = "id=" + str(body_to_send.id)
        body_string += ";x=" + str(body_to_send.x)
        body_string += ";y=" + str(body_to_send.y)
        body_string += ";yaw=" + str(body_to_send.yaw)
        body_string += ";steering_angle=" + str(body_to_send.commands['steering'])
        body_string += ";v=" + str(math.hypot(body_to_send.x_speed, body_to_send.y_speed))        
        body_string += ";t=" + str(time.time())

        if body_to_send.id == -1:

            body_string += ";desired_lane=" + str(body_to_send.desired_lane)

        return body_string

    def send_vehicle_state_packet(self):        
        '''
        Creates the UDP packet containing
        all the information about the current
        state of all vehicles in the SML World.
        '''

        message_string = str(self.sent_packet_counter)
        message_string += "#"

        bodies_string = ""

        for body_id in self.sml_world.bodies_dict:

            # We don't want a delimiter before
            # the first vehicle state
            if bodies_string:

                bodies_string += "!"

            bodies_string += self.get_vehicle_state_string( self.sml_world.bodies_dict[body_id] ) 

        message_string += bodies_string

        self.udp_sender_socket.sendto(message_string, (self.UDP_IP, self.UDP_PORT) )

        self.sent_packet_counter += 1


    def process_incoming_packets(self):        
        '''
        It will see if there are any incoming 
        packets, and it will process them all.
        It is currently used to obtain the 
        steering from the driver in the loop.
        '''

        buffer_size = 4*1024

        try:

            while True:

                data, addr = self.udp_reveiver_socket.recvfrom(buffer_size) 

                tokens = data.split(';')

                packet_number = int(tokens[2])

                if packet_number <= self.received_packet_counter:

                    # Out of order packet, ignore    
                    print "Received out of order packet"                
                    continue

                self.received_packet_counter = packet_number

                if tokens[0] == '1':

                    self.sml_world.bodies_dict[-1].unity_steering_override = True
                    self.sml_world.bodies_dict[-1].commands['steering'] = float(tokens[1])

                else:

                    self.sml_world.bodies_dict[-1].unity_steering_override = False

        except socket.timeout:

            # If nothing to receive a timeout exception will 
            # be thrown, simply ignore this exception
            return

        