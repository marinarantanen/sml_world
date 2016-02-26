import socket
import time

SML_WORLD_UDP_IP = '130.237.50.246'
# SML_WORLD_UDP_IP = '192.168.1.149'

bool_is_obstacle_in_front = False
bool_is_obstacle_in_right = False
bool_is_obstacle_in_back = False
bool_is_obstacle_in_left = False
distance_to_goal = 100
angle_to_goal = 0
distance_to_obstacle_in_front = 50
distance_to_obstacle_in_right = 50
distance_to_obstacle_in_back = 50
distance_to_obstacle_in_left = 50
bool_is_obstacle_in_front_right = False
bool_is_obstacle_in_front_front = False
bool_is_obstacle_in_front_left = False

int_speed = 0
int_steering = 0

execution_sleeping_time = 0.01

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

# Scenario 1 Solution:

# while (get_distance_to_goal()) > 10:
#   move_car_forward()
# stop_car()

# Scenario 2 Solution:

# while (get_distance_to_goal()) > 10:
#   if (get_angle_to_goal()) == 0:
#     steer_car_straight()
#   elif (get_angle_to_goal()) < 0:
#     steer_car_right()
#   else:
#     steer_car_left()
#   move_car_forward()
# stop_car()

# Scenario 3 Solution:

# while (get_distance_to_goal()) > 10:
#   if is_obstacle_in_front():
#     steer_car_left()
#   else:
#     if (get_angle_to_goal()) < 0:
#       steer_car_right()
#     elif (get_angle_to_goal()) > 0:
#       steer_car_left()
#     else:
#       steer_car_straight()
#   move_car_forward()
# stop_car()

# Maze Solution:

# while (get_distance_to_goal()) > 10:
#   if is_obstacle_in_front_left():
#     steer_car_right()
#   elif is_obstacle_in_front_right():
#     steer_car_left()
#   else:
#     steer_car_straight()
#   move_car_forward()


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

			# print print_str	+ " data = " + data

			global bool_is_obstacle_in_front
			if tokens[1] == 'True':
				bool_is_obstacle_in_front = True
			else:
				bool_is_obstacle_in_front = False

			global bool_is_obstacle_in_right
			if tokens[2] == 'True':
				bool_is_obstacle_in_right = True
			else:
				bool_is_obstacle_in_right = False

			global bool_is_obstacle_in_back
			if tokens[3] == 'True':
				bool_is_obstacle_in_back = True
			else:
				bool_is_obstacle_in_back = False

			global bool_is_obstacle_in_left
			if tokens[4] == 'True':
				bool_is_obstacle_in_left = True
			else:
				bool_is_obstacle_in_left = False

			global distance_to_goal
			distance_to_goal = int(tokens[5])

			global angle_to_goal
			angle_to_goal = int(tokens[6])

			global distance_to_obstacle_in_front
			distance_to_obstacle_in_front = int(tokens[7])

			global distance_to_obstacle_in_right
			distance_to_obstacle_in_right = int(tokens[8])

			global distance_to_obstacle_in_back
			distance_to_obstacle_in_back = int(tokens[9])

			global distance_to_obstacle_in_left
			distance_to_obstacle_in_left = int(tokens[10])

			global bool_is_obstacle_in_front_right
			if tokens[11] == 'True':
				bool_is_obstacle_in_front_right = True
			else:
				bool_is_obstacle_in_front_right = False

			global bool_is_obstacle_in_front_front
			if tokens[12] == 'True':
				bool_is_obstacle_in_front_front = True
			else:
				bool_is_obstacle_in_front_front = False

			global bool_is_obstacle_in_front_left
			if tokens[13] == 'True':
				bool_is_obstacle_in_front_left = True
			else:
				bool_is_obstacle_in_front_left = False



			# print "bool_is_obstacle_in_front_left = " + str(bool_is_obstacle_in_front_left)

	except socket.timeout:

		# print print_str
		# If nothing to receive a timeout exception will 
		# be thrown, simply ignore this exception
		return

	# print print_str	

	return

def send_packet():

	# print "send_packet()"

	global sent_packet_counter

	message_string = str(sent_packet_counter)
	message_string += ";"


	global int_speed

	message_string += str(int_speed)
	message_string += ";"

	global int_steering

	message_string += str(int_steering)

	global SML_WORLD_UDP_IP
	UDP_PORT = 34513

	global udp_sender_socket

	udp_sender_socket.sendto(message_string, (SML_WORLD_UDP_IP, UDP_PORT) )

	print "Sent data = " + message_string

	sent_packet_counter += 1
	
	return

def get_distance_to_goal():

	read_latest_packet()

	return distance_to_goal

def get_angle_to_goal():

	read_latest_packet()
	
	# print angle_to_goal

	return angle_to_goal

def is_obstacle_in_front():

	read_latest_packet()

	return bool_is_obstacle_in_front

def is_obstacle_in_right():

	read_latest_packet()

	return bool_is_obstacle_in_right

def is_obstacle_in_left():

	read_latest_packet()

	return bool_is_obstacle_in_left

def is_obstacle_in_back():

	read_latest_packet()

	return bool_is_obstacle_in_back

def is_obstacle_in_front_right():

	read_latest_packet()

	return bool_is_obstacle_in_front_right

def is_obstacle_in_front_front():

	read_latest_packet()

	return bool_is_obstacle_in_front_front

def is_obstacle_in_front_left():

	read_latest_packet()

	return bool_is_obstacle_in_front_left

def get_distance_to_obstacle_in_front():

	read_latest_packet()

	return distance_to_obstacle_in_front

def get_distance_to_obstacle_in_right():

	read_latest_packet()

	return distance_to_obstacle_in_right

def get_distance_to_obstacle_in_back():

	read_latest_packet()

	return distance_to_obstacle_in_back

def get_distance_to_obstacle_in_left():

	read_latest_packet()

	return distance_to_obstacle_in_left

def move_car_forward():

  global int_speed
  int_speed = 1

  send_packet()

  global execution_sleeping_time
  time.sleep(execution_sleeping_time)

  return

def move_car_back():

  global int_speed
  int_speed = -1

  send_packet()

  global execution_sleeping_time
  time.sleep(execution_sleeping_time)

  return

def stop_car():

  global int_speed
  int_speed = 0

  send_packet()

  global execution_sleeping_time
  time.sleep(execution_sleeping_time)

  return

def steer_car_left():

  global int_steering
  int_steering = 1

  send_packet()

  global execution_sleeping_time
  time.sleep(execution_sleeping_time)

  return

def steer_car_straight():

  global int_steering
  int_steering = 0

  send_packet()

  global execution_sleeping_time
  time.sleep(execution_sleeping_time)

  return

def steer_car_right():

  global int_steering
  int_steering = -1

  send_packet()

  global execution_sleeping_time
  time.sleep(execution_sleeping_time)

  return


start_udp_receiver_socket()
start_udp_sender_socket()

print "Program Started"



'''
Code starts here
'''















time.sleep(20)






move_car_forward()

while not (get_distance_to_goal()) < 10:
  if (is_obstacle_in_front_left()) and ((is_obstacle_in_front_front()) and (is_obstacle_in_front_right())):
    stop_car()
  else:
    if (is_obstacle_in_front_left()) and (is_obstacle_in_front_front()):
      steer_car_right()
    elif (is_obstacle_in_front_right()) and (is_obstacle_in_front_front()):
      steer_car_left()
    else:
      if not ((is_obstacle_in_front_left()) or ((is_obstacle_in_front_front()) or (is_obstacle_in_front_right()))):
        if (get_angle_to_goal()) < 0:
          steer_car_right()
        elif (get_angle_to_goal()) > 0:
          steer_car_left()
        else:
          steer_car_straight()
      else:
        steer_car_straight()
    move_car_forward()
stop_car()






# move_car_forward()

# while not (get_distance_to_goal()) < 10:
#   if (is_obstacle_in_front()):
#     steer_car_right()
#   else:
#     if (get_angle_to_goal()) < 0:
#       steer_car_right()
#     elif (get_angle_to_goal()) > 0:
#       steer_car_left()
#     else:
#       steer_car_straight()
#     move_car_forward()
# stop_car()





'''
Code ends here
'''


close_udp_receiver_socket()