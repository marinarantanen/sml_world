import socket
import rospy
from std_msgs.msg import String
from sml_world.msg import VehicleState, WorldState

class VisualisationModuleReceiver:
    '''
    Implements UDP receiving functionalities, to 
    be used on the VisualisationModule.
    These UDP functionalities consist in receiving the
    UDP packets with the vehicle states, and use this 
    information to update the current vehicle states to 
    be drawn by the visualisation module.
    '''

    def __init__(self, visualisation_module):

        # Get a reference to the VisualisationModule, 
        # so that we can change its vehicle states
        self.visualisation_module = visualisation_module

        # To keep track of the most recent
        # packet received, and to discard possible
        # our of order packets
        self.received_packet_counter = -1
        # Initialize the receiver socket
        self.start_udp_receiver_socket()
        self.pub = rospy.Publisher('world_state', WorldState, queue_size=10)
        rospy.init_node('simulator_world_class', anonymous=True)


    def start_udp_receiver_socket(self):
        '''
        Initializes the receiving socket, that
        will receive the states to draw
        '''

        UDP_IP = '' # Allows us to receive from any external PC
        UDP_PORT = 38008

        self.udp_reveiver_socket = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        self.udp_reveiver_socket.bind((UDP_IP, UDP_PORT))
        self.udp_reveiver_socket.settimeout(1./100.)

        return

    def process_incoming_packets(self):        
        '''
        It will see if there are any incoming 
        packets, and it will process them all.
        It is currently used to obtain the 
        states sent from the SML World.
        '''

        buffer_size = 4*1024

        try:

            while True:

                data, addr = self.udp_reveiver_socket.recvfrom(buffer_size)
                # print data
                self.process_states_data(data)

        except socket.timeout:

            # If nothing to receive a timeout exception will 
            # be thrown, simply ignore this exception
            return

    def process_states_data(self, data):
        '''
        Given the vehicle states data string, it will
        parse it and change the current vehicle states
        kept by the ProjectorModule
        '''

        vehicles_dict = dict()

        packet_tokens = data.split('#')

        packet_number = int(packet_tokens[0])

        if packet_number <= self.received_packet_counter:

            # Out of order packet, ignore    
            print "Received out of order packet"  

            return

        if packet_tokens[1] == '':

            print "No vehicles present in the message"

            self.visualisation_module.vehicles_dict = vehicles_dict

            return

        self.received_packet_counter = packet_number

        vehicle_tokens = packet_tokens[1].split('!')

        for vehicle_token in vehicle_tokens:

            state_tokens = vehicle_token.split(';')

            vehicle_id = None
            vehicle_x = None
            vehicle_y = None
            vehicle_yaw = None
            vehicle_fwd = None

            for state_token in state_tokens:
                key_and_value = state_token.split('=')

                key = key_and_value[0]
                value = key_and_value[1]

                if key == 'id':
                    vehicle_id = int(value)

                elif key == 'x':
                    vehicle_x = float(value)

                elif key == 'y':
                    vehicle_y = float(value)

                elif key == 'yaw':
                    vehicle_yaw = float(value)

                elif key == 'fwd':
                    try:
                        vehicle_fwd = int(value)
                    except ValueError:
                        vehicle_fwd = "None"

                elif key == 'desired_lane':
                    vehicle_class_name = value

                elif key == 'class_name':

                    vehicle_class_name = value
                else:

                    # Maybe the vehicle state has more information
                    # that we are not interested in.
                    # Simply ignore it.
                    pass
                    

            if (vehicle_id == None or vehicle_x == None or vehicle_y == None or vehicle_yaw == None):

                # If we are missing some of the required state information
                # we must issue a warning
                print "VisualisationModuleReceiver.py: Vehicle parsing went wrong!"

            vehicle = dict()
            vehicle['id'] = vehicle_id
            vehicle['x'] = vehicle_x
            vehicle['y'] = vehicle_y
            vehicle['yaw'] = vehicle_yaw
            vehicle['fwd'] = vehicle_fwd
            vehicle['class_name'] = vehicle_class_name

            vehicles_dict[vehicle_id] = vehicle
        
        world_state = WorldState()
        ws = []
        for v_id in vehicles_dict:
            vs = VehicleState()
            vs.vehicle_id = v_id
            vs.class_name = vehicles_dict[v_id]['class_name']
            vs.x = vehicles_dict[v_id]['x']
            vs.y = vehicles_dict[v_id]['y']
            vs.yaw = vehicles_dict[v_id]['yaw']
            ws.append(vs)
        print ws
        world_state.vehicle_states = ws
        self.pub.publish(world_state)
        self.visualisation_module.vehicles_dict = vehicles_dict

        return