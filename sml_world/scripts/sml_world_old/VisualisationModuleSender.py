import time, threading, socket

class VisualisationModuleSender:
    '''
    Class that sends the current vehicle states to a 
    VisualisationModule, in order for it to draw the vehicles.
    This class starts its own execution thread, that will 
    run a loo at the desired rate.
    '''

    def __init__(self, sml_world, visualisation_address = 'localhost', desired_rate = 50.):

        # Get a reference to the sml_world,
        # so that we are able to access the
        # bodies_dict
        self.sml_world = sml_world

        # The ip address of the computer hosting
        # the Visualisation Module
        self.visualisation_address = visualisation_address

        # The loop rate of this class' thread loop
        self.loop_rate = 50.

        # To add an ever increasing packet
        # number to every packet sent
        self.sent_packet_counter = 0
        # Initialize the sender socket
        self.start_udp_sender_socket()

        # Launches the tread that will run the main
        # loop of the VisualisationSenderModule
        t = threading.Thread( target = self.thread_loop, args=([]) )
        t.daemon = True
        t.start()

    def thread_loop(self):
        '''
        This function is executed in a thread, thus running 
        in parallel with the rest of the SML World.
        It consists of a loop, where messages containing
        the vehicle states are sent to the Visualisation Module.
        '''

        # Time keepers
        start_loop_time = time.time()
        end_loop_time = time.time()

        while not self.sml_world.close:

            start_loop_time = time.time()

            self.send_vehicle_state_packet()

            end_loop_time = time.time()

            time_elapsed = end_loop_time - start_loop_time

            time_to_sleep = 1./self.loop_rate - time_elapsed

            if time_to_sleep > 0:

                time.sleep(time_to_sleep)

            else:

                print "VisualisationModuleSender failed desired rate."

        self.close_sockets()

        return

    def start_udp_sender_socket(self):
        '''
        Initializes the sending socket, that
        will send the vehicle states to the
        VisualisationModule.
        '''

        self.UDP_IP = self.visualisation_address
        self.UDP_PORT = 38008

        self.udp_sender_socket = socket.socket(socket.AF_INET, # Internet
        socket.SOCK_DGRAM) # UDP

    def get_vehicle_state_string(self, body_to_send):
        '''
        Given a Vehicle, it will transform its state
        into a string to be sent to the Visualisation through UDP
        '''

        body_string = "id=" + str(body_to_send.id)
        body_string += ";x=" + str(body_to_send.x)
        body_string += ";y=" + str(body_to_send.y)
        body_string += ";yaw=" + str(body_to_send.yaw)
        #body_string += ";fwd=" + str(body_to_send.fwd_pair_partner)
        #body_string += ";desired_lane=" + str(body_to_send.desired_lane)
        body_string += ";class_name=" + str(body_to_send.__class__.__name__)

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

        body_ids = self.sml_world.bodies_dict.keys()

        for body_id in body_ids:

            # We don't want a delimiter before
            # the first vehicle state
            if bodies_string:

                bodies_string += "!"

            if body_id in self.sml_world.bodies_dict:

                current_body = self.sml_world.bodies_dict[body_id]

            bodies_string += self.get_vehicle_state_string( current_body )

        message_string += bodies_string

        self.udp_sender_socket.sendto(message_string, (self.UDP_IP, self.UDP_PORT) )

        self.sent_packet_counter += 1