import socket
import time
import serial

serial_comms = serial.Serial('COM5', 9600, timeout = .01)
time.sleep(1)



# UDP_IP_SML_WORLD = '192.168.1.149'
UDP_IP_SML_WORLD = '130.237.50.246'

global_bool_red_light = False
global_bool_yellow_light = False
global_bool_green_light = False

global_dict_lights = dict()
global_dict_lights[1] = False
global_dict_lights[2] = False
global_dict_lights[3] = False
global_dict_lights[4] = False
global_dict_lights[5] = False
global_dict_lights[6] = False
global_dict_lights[7] = False
global_dict_lights[8] = False
global_dict_lights[9] = False
global_dict_lights[10] = False
global_dict_lights[11] = False
global_dict_lights[12] = False
global_dict_lights[13] = False

global_bar_angle = 0

global_bool_bar_down = False

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


	
	

	# global global_bool_bar_down
	global global_bar_angle
	# if global_bool_bar_down:
	# if global_bar_angle == 10:
	# if global_bar_angle == 90:
	if global_bar_angle == 0:
		message_string += str(1)
	else:
		message_string += str(0)
	message_string += ";"

	# global global_bool_red_light
	global global_dict_lights

	# if global_bool_red_light:
	if global_dict_lights[8]:
		message_string += str(1)
	else:
		message_string += str(0)

	global UDP_IP_SML_WORLD
	UDP_PORT = 34513

	global udp_sender_socket

	print "Sent data = " + str(message_string)

	udp_sender_socket.sendto(message_string, (UDP_IP_SML_WORLD, UDP_PORT) )

	sent_packet_counter += 1

	send_serial_data()
	
	return

def send_serial_data():

	# string = "7;8;9;10;11;12;" + str(i) + "K"

	string = ''

	global global_dict_lights

	for i in range(6):

		if global_dict_lights[7+i]:
			string+='1'
		else:
			string+='0'
		string+=';'

	global global_bar_angle

	string+=str(global_bar_angle)

	string+='K'	

	serial_comms.write(string)

	print "serial data = " + string

	serial_comms.flush()
	time.sleep(execution_sleeping_time)


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


def set_red_light(lit):

	global global_bool_red_light

	if lit:
		global_bool_red_light = True
	else:
		global_bool_red_light = False

	send_packet()	
	time.sleep(execution_sleeping_time)

	return

def set_yellow_light(lit):

	global global_bool_yellow_light

	if lit:
		global_bool_yellow_light = True
	else:
		global_bool_yellow_light = False

	send_packet()	
	time.sleep(execution_sleeping_time)

	return

def set_green_light(lit):

	global global_bool_green_light

	if lit:
		global_bool_green_light = True
	else:
		global_bool_green_light = False

	send_packet()	
	time.sleep(execution_sleeping_time)

	return

def set_bar_down(down):

	global global_bool_bar_down

	if down:
		global_bool_bar_down = True
	else:
		global_bool_bar_down = False

	send_packet()	
	time.sleep(execution_sleeping_time)

	return

def set_bar_angle(angle):

	global global_bar_angle

	global_bar_angle = angle

	send_packet()	
	time.sleep(execution_sleeping_time)

	return

def turn_light_on(light_index):

	global global_dict_lights

	global_dict_lights[light_index] = True
	
	send_packet()	
	time.sleep(execution_sleeping_time)

	return

def turn_light_off(light_index):

	global global_dict_lights

	global_dict_lights[light_index] = False
	
	send_packet()	
	time.sleep(execution_sleeping_time)

	return
	

start_udp_receiver_socket()
start_udp_sender_socket()

print "Program Started"



'''
Code starts here
'''












while True:
 
  
  
  set_bar_angle(0)
  blockly_wait(3.0)
  turn_light_off(8)
  turn_light_on(9)
  blockly_wait(3.0)
  turn_light_off(9)
  turn_light_on(10)
  blockly_wait(5.0)
  turn_light_off(10)
  turn_light_on(9)
  blockly_wait(3.0)
  turn_light_off(9)
  turn_light_on(8)
  blockly_wait(4.0)
  set_bar_angle(90)
  blockly_wait(8.0)
'''
Code ends here
'''


close_udp_receiver_socket()
serial_comms.close()