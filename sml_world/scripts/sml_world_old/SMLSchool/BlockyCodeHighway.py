import socket
import time

# UDP_IP_SML_WORLD = 'localhost'
UDP_IP_SML_WORLD = '130.237.50.246'
# UDP_IP_SML_WORLD = '192.168.1.149'

bool_is_car_in_right = False
bool_is_car_in_rear_right = False
bool_is_car_in_rear_middle = False
bool_is_car_in_rear_left = False
bool_is_car_in_left = False
bool_is_car_in_front_left = False
bool_is_car_in_front_middle = False
bool_is_car_in_front_right = False
string_current_car_lane = "right"

string_desired_car_lane = "right"
string_desired_car_speed = "normal"

int_speed = 0
int_steering = 0

execution_sleeping_time = 0.05

udp_receiver_socket = []
udp_sender_socket = []

sent_packet_counter = 0
received_packet_counter = 0

def start_udp_sender_socket():
    '''
    Initializes the sending socket, that
    will send the vehicle states to the
    Simulator.
    '''

    global udp_sender_socket

    udp_sender_socket = socket.socket(socket.AF_INET, # Internet
    socket.SOCK_DGRAM) # UDP

def start_udp_receiver_socket():
    
    UDP_IP = '' # Allows us to receive from any external PC
    UDP_PORT = 34514

    global udp_receiver_socket
    udp_receiver_socket = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    udp_receiver_socket.bind((UDP_IP, UDP_PORT))
    udp_receiver_socket.settimeout(0.01)

    return

def close_udp_receiver_socket():

    global udp_receiver_socket
    udp_receiver_socket.close()

    return

def read_latest_packet():

	print_str =  "read_latest_packet()"

	buffer_size = 1024

	try:

		while True:

			global udp_receiver_socket

			data, addr = udp_receiver_socket.recvfrom(buffer_size) 

			tokens = data.split(';')

			packet_number = int(tokens[0])

			global received_packet_counter

			if packet_number <= received_packet_counter:

				# Out of order packet, ignore    
				print "BlockyCode received out of order packet: " + str(packet_number)  
				continue

			received_packet_counter = packet_number

			print print_str	+ " data = " + data

			global bool_is_car_in_right
			if tokens[1] == 'True':
				bool_is_car_in_right = True
			else:
				bool_is_car_in_right = False

			global bool_is_car_in_rear_right
			if tokens[2] == 'True':
				bool_is_car_in_rear_right = True
			else:
				bool_is_car_in_rear_right = False

			global bool_is_car_in_rear_middle
			if tokens[3] == 'True':
				bool_is_car_in_rear_middle = True
			else:
				bool_is_car_in_rear_middle = False

			global bool_is_car_in_rear_left
			if tokens[4] == 'True':
				bool_is_car_in_rear_left = True
			else:
				bool_is_car_in_rear_left = False

			global bool_is_car_in_left
			if tokens[5] == 'True':
				bool_is_car_in_left = True
			else:
				bool_is_car_in_left = False

			global bool_is_car_in_front_left
			if tokens[6] == 'True':
				bool_is_car_in_front_left = True
			else:
				bool_is_car_in_front_left = False

			global bool_is_car_in_front_middle
			if tokens[7] == 'True':
				bool_is_car_in_front_middle = True
			else:
				bool_is_car_in_front_middle = False

			global bool_is_car_in_front_right
			if tokens[8] == 'True':
				bool_is_car_in_front_right = True
			else:
				bool_is_car_in_front_right = False

			global string_current_car_lane
			if tokens[9] == 'left' or tokens[9] == 'middle' or tokens[9] == 'right':
				string_current_car_lane = tokens[9]
			else:
				raise NameError("Unexpected lane name")


			



			# print "bool_is_obstacle_in_front_left = " + str(bool_is_obstacle_in_front_left)

	except socket.timeout:

		# print print_str
		# If nothing to receive a timeout exception will 
		# be thrown, simply ignore this exception
		return

	print print_str	

	send_packet()

	return

def send_packet():

	# print "send_packet()"

	global sent_packet_counter

	message_string = str(sent_packet_counter)
	message_string += ";"


	global string_desired_car_lane

	message_string += string_desired_car_lane
	message_string += ";"

	global string_desired_car_speed

	message_string += string_desired_car_speed

	global UDP_IP_SML_WORLD
	UDP_PORT = 34513

	global udp_sender_socket

	print "Sent data = " + str(message_string)

	udp_sender_socket.sendto(message_string, (UDP_IP_SML_WORLD, UDP_PORT) )

	sent_packet_counter += 1
	
	return

def is_car_in_right():
	read_latest_packet()
	return bool_is_car_in_right

def is_car_in_rear_right():
	read_latest_packet()
	return bool_is_car_in_rear_right

def is_car_in_rear_middle():
	read_latest_packet()
	return bool_is_car_in_rear_middle

def is_car_in_rear_left():
	read_latest_packet()
	return bool_is_car_in_rear_left

def is_car_in_left():
	read_latest_packet()
	return bool_is_car_in_left

def is_car_in_front_left():
	read_latest_packet()
	return bool_is_car_in_front_left

def is_car_in_front_middle():
	read_latest_packet()
	return bool_is_car_in_front_middle

def is_car_in_front_right():
	read_latest_packet()
	return bool_is_car_in_front_right

def current_car_lane():
	read_latest_packet()

	if string_current_car_lane == 'left':
		return "Left"
	elif string_current_car_lane == 'middle':
		return "Middle"
	elif string_current_car_lane == 'right':
		return "Right"
	else:
		raise NameError("Unknown")

def set_car_speed(desired_speed_argument):

	if desired_speed_argument == 'fast' or desired_speed_argument == 'normal' or desired_speed_argument == 'slow' or desired_speed_argument == 'stop':

		global string_desired_car_speed
		string_desired_car_speed = desired_speed_argument

	else:

		raise NameError("Unexpected speed")

	send_packet()

	global execution_sleeping_time
	time.sleep(execution_sleeping_time)

	return

def set_car_lane(desired_lane_argument):

	if desired_lane_argument == 'left' or desired_lane_argument == 'middle' or desired_lane_argument == 'right':

		global string_desired_car_lane
		string_desired_car_lane = desired_lane_argument

	else:

		raise NameError("Unexpected lane")

	send_packet()

	global execution_sleeping_time
	time.sleep(execution_sleeping_time)

	return


def blockly_wait(seconds):

	start_time = time.time()

	elapsed_time = time.time() - start_time

	while elapsed_time < seconds:
	
		send_packet()

		read_latest_packet()

		global execution_sleeping_time
		time.sleep(execution_sleeping_time)

		elapsed_time = time.time() - start_time

	return




	

start_udp_receiver_socket()
start_udp_sender_socket()

print "Program Started"



'''
Code starts here
'''




while True:
  set_car_speed('fast')
  if is_car_in_front_middle():
    if (current_car_lane()) == 'Left':
      if is_car_in_front_right():
        set_car_speed('stop')
        blockly_wait(4.0)
      set_car_lane('middle')
      blockly_wait(4.0)
    elif (current_car_lane()) == 'Right':
      if is_car_in_front_left():
        set_car_speed('stop')
        blockly_wait(4.0)
      set_car_lane('middle')
      blockly_wait(4.0)
    elif (current_car_lane()) == 'Middle':
      if is_car_in_left():
        set_car_lane('right')
        blockly_wait(4.0)
      elif is_car_in_right():
        set_car_lane('left')
        blockly_wait(4.0)
      else:
        set_car_lane('left')
        blockly_wait(4.0)



'''
Code ends here
'''


close_udp_receiver_socket()