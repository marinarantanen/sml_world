import xml.etree.ElementTree as ET
import xml.dom.minidom
import socket
from xml.dom import minidom
import time

import bodyclasses, smartvehicle

BUFSIZE = 2**15


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def get_message_type(request_string):
	"Roughly process an incoming message"
	"Output otions guide:"
	"If message is invalid return None"
	"If message is valid return message_type"

	xml_root = []

	try:

		xml_root = ET.fromstring(request_string)

	except (ET.ParseError):

		#print ET.ParseError
		return None

	if xml_root.tag == None or xml_root.tag != "message":

		return None

	message_type = xml_root.get("type")

	return message_type


def process_controlled_body_info_message(request_string,log_flag = False):
	"Process an incoming controlled_body_info message"
	
	xml_root = []

	try:

		xml_root = ET.fromstring(request_string)

		if log_flag:
			f = open('Logs/controlled_body_info_message.xml','w')
			f.write(prettify(root))
			f.close()

		controlled_bodies_ids = []

		for current_body in xml_root.findall('body'):

			controlled_bodies_ids.append( int( current_body.get("id") ) )

		return controlled_bodies_ids

	except:

		print "Error in xmlcomms.process_controlled_body_info_message()"

		return None

def process_unity_simulator_message_string(unity_simulator_message_string):

	root = []

	original_message = unity_simulator_message_string

	try:
		root = ET.fromstring(unity_simulator_message_string.encode('utf-16-be'))



	except:

		print "Failed to receive message: " + original_message
		print "Will assume that body is in origin 0,0,0"
		return [808, 0, 0, 0]

		try:
			print "Failed to receive in utf-16-be mode, trying in the standard way"
			root = ET.fromstring(unity_simulator_message_string)
		except:

			print "Failed again, here is the message: " + original_message


	# root = ET.Element('message')

	unity_simulator_car_id = int( root.find("body").get("id") )
	unity_simulator_car_x = float( root.find("body").get("x") )
	unity_simulator_car_y = float( root.find("body").get("y") )
	unity_simulator_car_yaw = float( root.find("body").get("yaw") )

	return [unity_simulator_car_id, unity_simulator_car_x, unity_simulator_car_y, unity_simulator_car_yaw]

def process_image_request_xml_string(request_string,log_flag = False):

	print "request_string = " + str(request_string) 

	xml_root = ET.fromstring(request_string)

	if log_flag:
		f = open('Logs/image_request.xml','w')
		f.write(prettify(xml_root))
		f.close()

	# xml_message = xml_root.find('message')

	really_send_string = xml_root.get('really_send')

	really_send = True

	if really_send_string == "True" or really_send_string == "true":

		really_send = True

	else:

		really_send = False


	xml_socket_info = xml_root.find('socket')

	if xml_socket_info == None:

		print "Error in xml string"
		return

	ip_string = xml_socket_info.get("ip")
	port_number = int( xml_socket_info.get("port") )

	return [really_send, ip_string, port_number]

def process_construction_command_xml_string(request_string,log_flag = False):

	print "construction_command = " + str(request_string) 

	xml_root = ET.fromstring(request_string)

	if log_flag:
		f = open('Logs/construction_command.xml','w')
		f.write(prettify(xml_root))
		f.close()

	processed_orders = []

	for current_order in xml_root.findall('order'):

		processed_order = dict()
		processed_order['id'] = int( current_order.get("id") )
		processed_order['yaw'] = float( current_order.get("yaw") )

		processed_orders.append(processed_order)

	return processed_orders

def get_construction_finished_xml_string(completed_orders, log_flag = False):

	root = ET.Element('message')

	root.set("type", "construction_finished")

	for completed_order in completed_orders:

		current_id = completed_order['id']

		ET.SubElement(root, "body", id = str(current_id))

	solution_root = ET.SubElement(root, "solution")

	if log_flag:

		f = open('Logs/construction_finished.xml','w')
		f.write(prettify(root))
		f.close()


	xml_string = ET.tostring(root)

	return xml_string

def process_rrt_trajectory_request_xml_string(request_string,log_flag = False):

	xml_root = ET.fromstring(request_string)

	if log_flag:
		f = open('Logs/rrt_trajectory_request.xml','w')
		f.write(prettify(xml_root))
		f.close()

	xml_init_state_info = xml_root.find('initial_state')

	if xml_init_state_info == None:

		print "process_rrt_trajectory_request_xml_string: Error in xml string"
		return None

	start_x = float( xml_init_state_info.get("x") )
	start_y =  float( xml_init_state_info.get("y") )
	start_theta_0 = float( xml_init_state_info.get("theta_0") )
	start_theta_1 =  float( xml_init_state_info.get("theta_1") )
	body_id =  int( xml_init_state_info.get("body_id") )

	start_pose = [start_x, start_y, start_theta_0, start_theta_1]

	xml_goal_info = xml_root.find('goal_position')

	if xml_goal_info == None:

		print "process_rrt_trajectory_request_xml_string: Error in xml string"
		return None

	goal_x = float( xml_goal_info.get("x") )
	goal_y =  float( xml_goal_info.get("y") )

	goal_position = [goal_x, goal_y]

	return [start_pose, goal_position, body_id]

def process_matlab_trajectory_message(request_string,log_flag = False):
	"Process an incoming controlled_body_info message"
	
	xml_root = []

	message_type = get_message_type(request_string)

	if message_type == None:

		return None

	if message_type == "body_trajectory_command":

		print "It's a trajectory"

	
	xml_root = ET.fromstring(request_string)

	body_id = xml_root.get("body_id")

	if log_flag:
		f = open('Logs/matlab_trajectory.xml','w')
		f.write(prettify(xml_root))
		f.close()

	return body_id

def get_controlled_body_info_message(body_id,log_flag = False):
	"Process an incoming controlled_body_info message"
	
	root = ET.Element('message')

	root.set("type", "task_completed")

	ET.SubElement(root, "body", id = str(body_id) )

	if log_flag:
		f = open('Logs/controlled_body_info.xml','w')
		f.write(prettify(root))
		f.close()

	xml_string = ET.tostring(root)

	return xml_string

def get_simulated_body_id_message(body_id,log_flag = False):
	"Process an incoming controlled_body_info message"
	
	root = ET.Element('message')

	root.set("type", "new_simulated_id")

	ET.SubElement(root, "body", id = str(body_id) )

	if log_flag:
		f = open('Logs/simulated_body_id.xml','w')
		f.write(prettify(root))
		f.close()

	xml_string = ET.tostring(root)

	return xml_string	

def get_rrt_solution_message(rrt_nodes, solution_ids, body_id, log_flag = False):
	"Process an incoming controlled_body_info message"
	
	root = ET.Element('message')

	root.set("type", "rrt_solution")

	tree_root = ET.SubElement(root, "tree", body_id = str( int(body_id) ) )

	# for tree_node in rrt_nodes:
	for it in range( len( rrt_nodes ) ):

		tree_node = rrt_nodes[it]
		ET.SubElement(tree_root, "tree_node", id = str(it), x = str(tree_node[0]), y = str(tree_node[1]), parent_id = str(tree_node[2]) )

	solution_root = ET.SubElement(root, "solution")

	# for tree_node in rrt_nodes:
	for it in range( len( solution_ids ) ):

		solution_id = solution_ids[it]
		ET.SubElement(solution_root, "node", id = str(solution_id), order = str(it) )


	if log_flag:
		f = open('Logs/rrt_solution.xml','w')
		f.write(prettify(root))
		f.close()


	xml_string = ET.tostring(root)

	return xml_string

def process_task_completed_message(request_string,log_flag = False):
	"Process an incoming controlled_body_info message"
	
	xml_root = []

	try:

		xml_root = ET.fromstring(request_string)

		if log_flag:
			f = open('Logs/task_completed.xml','w')
			f.write(prettify(xml_root))
			f.close()

		completed_bodies_ids = []

		for current_body in xml_root.findall('body'):

			completed_bodies_ids.append( int( current_body.get("id") ) )

		if len(completed_bodies_ids) != 1:

			print "Big Error, only one vehicle should have finished"

		return completed_bodies_ids

	except:

		print "Error in xmlcomms.process_task_completed_message()"

		return None

def make_road_info_xml_message_string(osm_node_list, osm_way_list, osm_lanelet_list,log_flag = False):

	# print "make_road_info_xml_message_string"

	root = ET.Element('message')

	root.set("type", "road_info")

	for node in osm_node_list:

		if node.ignore:
			# print "ignored node in xml message"
			continue

		# ET.SubElement(root, "node", id = str(node.id), pixel_x = str(node.pixel_x), pixel_y = str(node.pixel_y) )
		current_element = ET.SubElement(root, "node", id = str(node.id), x = str(node.x), y = str(node.y) )

		if node.destination != None:

			# print "Adding a special tag destination: " + node.destination
			current_element.set("destination", node.destination)

	for way in osm_way_list:

		current_way_et = ET.SubElement(root, "way", id = str(way.id) )

		if way.line_type != None:

			# print "Setting special line type in road_info_message_string"
			# print "way.line_type = " + str(way.line_type)
			current_way_et.set("line_type", way.line_type)

		for node_id in way.node_ids:

			ET.SubElement(current_way_et, "nd", id = str(node_id) )

	for lanelet in osm_lanelet_list:

		current_lanelet_et = ET.SubElement(root, "lanelet", id = str(lanelet.id) )
		current_lanelet_et.set('left_way', str(lanelet.left_osm_way.id) )
		current_lanelet_et.set('right_way', str(lanelet.right_osm_way.id) )

	xml_string = ET.tostring(root)
	
	return xml_string

def get_trajectory_reply_message_string(start_id, end_id, osm_lanelet_list, osm_way_list, osm_node_list, truck_vehicle = True, log_flag = False):

	root = ET.Element('message')

	[traj_x, traj_y] = get_trajectory_from_node_ids(start_id, end_id, osm_lanelet_list, osm_way_list, osm_node_list, truck_vehicle)

	root.set("type", "trajectory_reply")
	root.set("id", "5")

	for i in range( len( traj_x ) ):

		current_node = ET.SubElement(root, 'point')
		current_node.set("x", str( traj_x[i] ) )
		current_node.set("y", str( traj_y[i] ) )
		current_node.set("time", str(i) )

	if log_flag:
		f = open('Logs/trajectory_reply.xml','w')
		f.write(prettify(root))
		f.close()
	
	return ET.tostring(root)

def get_lanelet_obstruction_message_string(lanelet_collision_ids, log_flag = False):

	root = ET.Element('message')

	root.set("type", "lanelet_obstruction")

	for lanelet_id in lanelet_collision_ids:

		current_lanelet = ET.SubElement(root, 'lanelet')
		current_lanelet.set("id", str(lanelet_id) )

	if log_flag:
		f = open('Logs/lanelet_obstruction.xml','w')
		f.write(prettify(root))
		f.close()
	
	return ET.tostring(root)

def get_lanelet_polygon_obstruction_message_string( new_polygons_lanelets , log_flag = False):

	root = ET.Element('message')

	root.set("type", "lanelet_obstruction_polygons")

	for polygons_lanelets in new_polygons_lanelets:

		current_polygon_lanelet = ET.SubElement(root, 'lanelet_polygon')
		current_polygon_lanelet.set("id", str(polygons_lanelets['id']) )		
		current_polygon_lanelet.set("obstructed", str(polygons_lanelets['obstructed']) )

		if polygons_lanelets['obstructed']:

			current_polygon_points = ET.SubElement(current_polygon_lanelet, 'polygon_points')

			for order, current_point_coordinates in enumerate( polygons_lanelets['points'] ):

				current_polygon_point = ET.SubElement(current_polygon_points, 'points')

				current_polygon_point.set("x", str( current_point_coordinates[0] ) )
				current_polygon_point.set("y", str( current_point_coordinates[1] ) )
				current_polygon_point.set("seq", str( order ) )

	if log_flag:
		f = open('Logs/lanelet_polygon.xml','w')
		f.write(prettify(root))
		f.close()

	return ET.tostring(root)

def get_image_properties_reply_message_string(center_pixel_x, center_pixel_y, pixel_per_meter,log_flag = False):

	root = ET.Element('message')

	root.set("type", "image_properties_reply")

	image_info = ET.SubElement(root, 'image_info')
	image_info.set("center_pixel_x", str( center_pixel_x ) )
	image_info.set("center_pixel_y", str( center_pixel_y ) )
	image_info.set("pixel_per_meter", str(pixel_per_meter) )

	if log_flag:
		f = open('Logs/image_properties.xml','w')
		f.write(prettify(root))
		f.close()
	
	return ET.tostring(root)

def process_trajectory_request_string(trajectory_request_message_string, osm_node_list, osm_way_list, body_request = False,log_flag = False):

	root = ET.fromstring(trajectory_request_message_string)

	if log_flag:
		f = open('Logs/trajectory_request.xml','w')
		f.write(prettify(root))
		f.close()


	start_id = 0

	if body_request:

		start_id = int( root.find("body").get("id") )

	else:

		start_id = int( root.find("start").get("id") )

	end_id = int( root.find("end").get("id") )

	return [start_id, end_id]

def process_obstacle_message_string(obstacle_request_message_string,log_flag = False):

	root = ET.fromstring(obstacle_request_message_string)

	if log_flag:
		f = open('Logs/obstacle.xml', 'w')
		f.write(prettify(root))
		f.close()

	obstacle_info = dict()
	obstacle_info['reply_type'] = 'obstacle_command'
	obstacle_info['id'] = int(root.find("vehicle").get("id"))
	obstacle_info['x'] = float(root.find("vehicle").get("x"))
	obstacle_info['y'] = float(root.find("vehicle").get("y"))
	obstacle_info['type'] = root.find("vehicle").get("type")
	obstacle_info['radius'] = float(root.find("vehicle").get("radius"))
	obstacle_info['obstacle_type'] = root.find("vehicle").get("obstacle_type")

	return obstacle_info

def process_highlight_message_string(highlight_request_message_string,log_flag = False):

	root = ET.fromstring(highlight_request_message_string)

	if log_flag:
		f = open('Logs/highlight.xml','w')
		f.write(prettify(root))
		f.close()

	vehicle_id = int( root.find("vehicle").get("id") )

	highlight = int( root.find("vehicle").get("highlight") )

	if highlight!=0 and highlight!=1:

		raise NameError('In process_highlight_message_string(): Highlight field should be a boolean 0 or 1')

	return [vehicle_id, highlight]

def process_body_id_message_string(body_id_message,log_flag = False):

	root = ET.fromstring(body_id_message)

	if log_flag:
		f = open('Logs/body_id_message.xml','w')
		f.write(prettify(root))
		f.close()
	
	vehicle_id = int( root.find("body").get("id") )

	return vehicle_id

def process_initial_state_simulated_vehicle_message_string(initial_state_message, log_flag = False):

	root = ET.fromstring(initial_state_message)

	if log_flag:
		f = open('Logs/initial_state_simulated_vehicle_message.xml','w')
		f.write(prettify(root))
		f.close()
	
	initial_state = dict()

	initial_state['x'] = float( root.find("initial_state").get("x") )
	initial_state['y'] = float( root.find("initial_state").get("y") )
	initial_state['yaw'] = float( root.find("initial_state").get("yaw") )

	return initial_state

def process_vehicle_commands_message_string(commands_string, log_flag = False):

	root = ET.fromstring(commands_string)

	if log_flag:
		f = open('Logs/vehicle_commands_message.xml','w')
		f.write(prettify(root))
		f.close()
	
	commands = dict()

	commands['throttle'] = float( root.find("commands").get("throttle"))
	commands['steering'] = float( root.find("commands").get("steering"))

	return commands

def get_controllable_vehicle_readings_reply_message_string(bodies_list, bodies_readings,log_flag = False):

	readings = []

	for body in bodies_list:

		if body.has_key('controllable'):

			if body['controllable'] == True:

				for body_reading in bodies_readings:

					if body_reading['id'] == body['id']:

						readings.append( body_reading )


	root = ET.Element('message')

	root.set("type", "vehicle_readings_reply")

	for body_reading in readings:

		body_info = ET.SubElement(root, 'vehicle_readings')
		body_info.set("id", str( body_reading['id'] ))

		for other_body in body_reading['readings']:

			other_body_info = ET.SubElement(body_info, 'other_body_info')
			other_body_info.set("x", str( other_body[0] ))
			other_body_info.set("y", str( other_body[1] ))

	if log_flag:
		f = open('Logs/controllable_vehicles_readings.xml','w')
		f.write(prettify(root))
		f.close()
	
	return ET.tostring(root)

def get_vehicle_states_reply_message_string_old(bodies_list,log_flag = False):

	root = ET.Element('message')

	root.set("type", "vehicle_states_reply")

	if bodies_list is not 'off':
		for body in bodies_list:
			if body != None:
				body_info = ET.SubElement(root, 'body_info')
				for tag in body.keys():
					body_info.set(tag, str( body[tag]))

	if log_flag:
		f = open('Logs/vehicles_states.xml','w')
		f.write(prettify(root))
		f.close()

	return ET.tostring(root)

def get_vehicle_states_reply_message_string(bodies_array,log_flag = False):

	# print "get_vehicle_states_reply_message_string()"

	root = ET.Element('message')

	root.set("type", "vehicle_states_reply")

	current_body_ids = bodies_array.keys()
	# Making a copy of the keys, to avoid dictionary size change error

	for body_id in current_body_ids:

		if body_id in bodies_array:
		# Making sure the body is still there to avoid dictionary access error

			current_body = bodies_array[body_id]

			body_info = ET.SubElement(root, 'body_info')


			body_info.set( 'id', str( body_id ) )
			# body_info.set( 'id', str( bodies_array[body_id].id ) )
			# body_info.set( 'id', str( -2 ) )

			body_info.set( 'x', str( current_body.x/32. ) )
			body_info.set( 'y', str( current_body.y/32. ) )
			body_info.set( 'yaw', str( current_body.yaw ) )
			
			
			if isinstance(current_body, bodyclasses.UnitySimulatorCar):

				body_info.set( 'body_type' , 'simulator_vehicle' )

			elif isinstance(current_body, bodyclasses.DummyVehicle):

				body_info.set( 'body_type' , 'truck' )		

			elif isinstance(current_body, bodyclasses.Person):

				body_info.set( 'body_type' , 'person' )		

			elif isinstance(current_body, bodyclasses.ConstructionVehicle):

				body_info.set( 'body_type' , 'construction' )

			elif isinstance(current_body, bodyclasses.BusVehicle):

				body_info.set( 'body_type' , 'bus' )	

			elif isinstance(current_body, smartvehicle.SmartVehicle):

				body_info.set( 'body_type' , 'smart_vehicle' )	

				if current_body.cat != None:
					body_info.set( 'cat' , str( current_body.cat ))
				else:
					body_info.set( 'cat', '0' )

			else:

				body_info.set( 'body_type' , 'truck' )		
		

			if current_body.id == 5:

				body_info.set( 'obstacle' , 'True' )
				body_info.set( 'obstacle_type' , 'rrt' )
				body_info.set( 'width' , str( 0.4 ) )
				body_info.set( 'height' , str( 0.3 ) )

			else:

				body_info.set( 'obstacle' , 'False' )
			
			if not isinstance(current_body, bodyclasses.UnitySimulatorCar) and not isinstance(current_body, bodyclasses.Person) :
				
				if current_body.sensor_readings:
					body_info.set( 'collision' , 'True' )
				else:
					body_info.set( 'collision' , 'False' )

			body_info.set( 'controllable' , 'False' )
			

			body_info.set( 'trailer' , 'False' )
			body_info.set( 'trailer_id' , '0' )


			body_info.set( 'pitch' , '0.0' )
			body_info.set( 'roll' , '0.0' )
			body_info.set( 'z' , '0.0' )

			body_info.set( 'signal_strength', '-1.0')

	if log_flag:
		f = open('Logs/vehicles_states.xml','w')
		f.write(prettify(root))
		f.close()

	return ET.tostring(root)

def get_collision_warning_message_string(colliding_bodies,log_flag = False):

	root = ET.Element('message')

	root.set("type", "collision_warning")

	for body in colliding_bodies:

		body_info = ET.SubElement(root, 'body_info')
		body_info.set("id", str( body ) )

	if log_flag:
		f = open('Logs/colission_warning.xml','w')
		f.write(prettify(root))
		f.close()

	message_string = ET.tostring(root) + '\n'

	return message_string

def create_server(osm_lanelet_list, osm_node_list, osm_way_list):

	#create an INET, STREAMing socket
	serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	#bind the socket to a public host,
	# and a well-known port

	print "Creating a PUBLIC socket"
	serversocket.bind((socket.gethostname(), 6557))
	# print "Creating a PRIVATE INTERNAL socket"
	# serversocket.bind(('localhost', 5555))
	
	print "Waiting for a connection"
	serversocket.settimeout(10)
	#become a server socket
	serversocket.listen(1)
	#accept connections from outside
	(clientsocket, address) = serversocket.accept()

	# print "Pause one second"
	# time.sleep() 

	print "Sending road info"
	# Get road info string and send it
	road_info_message_string = make_road_info_xml_message_string(osm_node_list, osm_way_list)
	road_info_message_string = road_info_message_string + "\n"
	# road_info_message_string = "lalalalalalalala \n"
	sent = clientsocket.send(road_info_message_string)
	if sent == 0:
		raise RuntimeError("socket connection broken")

	print "Waiting for trajectory request"
	# Wait for trajectory request and process it
	clientsocket.setblocking(1)
	reply_string = clientsocket.recv(BUFSIZE);
	print "Received trajectory reply: " + reply_string
	[start_id, end_id] = process_trajectory_request_string(reply_string, osm_node_list, osm_way_list)

	print "Sending trajectory reply"
	# Get trajectory string and send it
	trajectory_reply_message_string = get_trajectory_reply_message_string(start_id, end_id, osm_lanelet_list, osm_way_list, osm_node_list)
	print "trajectory_reply_message_string: "
	print trajectory_reply_message_string
	clientsocket.send(trajectory_reply_message_string)

	return 0

def send_initial_WILL_IT_BE_MISSED_xml(osm_node_list, osm_way_list):


	root = ET.Element('message')

	root.set("type", "road_info")

	for node in osm_node_list:

		ET.SubElement(root, "node", id = str(node.id), pixel_x = str(node.pixel_x), pixel_y = str(node.pixel_y) )


	for way in osm_way_list:

		current_way_et = ET.SubElement(root, "way", id = str(way.id) )

		for node_id in way.node_ids:

			ET.SubElement(current_way_et, "nd", id = str(node_id) )


	tree = ET.ElementTree(root)
	tree.write("send_initial_xml.xml")

	# print "tree:"
	xml_string = ET.tostring(root)
	print xml_string
	print prettify(root)



	print "Creating the socket"
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	host = "130.237.50.246"
	port = 5555

	print "Connecting the socket"
	s.connect( (host, port) )

	print "Sending information over the socket"
	s.send( xml_string )
