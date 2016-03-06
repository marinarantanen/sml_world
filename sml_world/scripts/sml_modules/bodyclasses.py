
class Body(object):
	# This is the base class for all the objects in the simulation (trucks, obstacles etc)
	def __init__(self):
		# Unique identifier for every body
		self.id = 0
		self.x = 0.
		self.y = 0.
		self.z = 0.
		self.yaw = 0.
		self.pitch = 0.
		self.roll = 0.
		# Specifies the type of body (truck, excavator, bus, person, etc...)
		self.type = ''
		# Specifies if the body is simulated, or real (obtained from Qualisys)
		self.simulated = True


class QualisysBody(Body):
	def __init__(self):
		Body.__init__(self)
		self.qualisys_time_stamp = 0.
		self.simulated = False

class QualisysBigBox(Body):
	def __init__(self):
		Body.__init__(self)
		self.qualisys_time_stamp = 0.
		self.simulated = False

class QualisysSmallBox(Body):
	def __init__(self):
		Body.__init__(self)
		self.qualisys_time_stamp = 0.
		self.simulated = False

class QualisysGoal(Body):
	def __init__(self):
		Body.__init__(self)
		self.qualisys_time_stamp = 0.
		self.simulated = False


class Moving(Body):
	# Child class of Body, it will be used for every body that is moving (trucks, buses, persons, etc...)
	def __init__(self):
		Body.__init__(self)
		self.x_speed = 0.
		self.y_speed = 0.
		self.z_speed = 0.
		self.yaw_speed = 0.
		self.pitch_speed = 0.
		self.roll_speed = 0.
		self.width = 0.
		self.length = 0.
		self.height = 0.


class UnitySimulatorCar(Moving):
	def __init__(self):
		Moving.__init__(self)


class Static(Body):
	# Child class of Body, it will be used for every body that is static (antenna, obstacle, etc...)
	def __init__(self):
		Body.__init__(self)


class Person(Static):
	# Child class of static, to be used with persons
	def __init__(self):
		Static.__init__(self)
		self.bus_stop_destination_string = None


class Antenna(Static):
	# Child class of static, to be used with antennas
	def __init__(self):
		Static.__init__(self)
		self.coverage_radius = 0.


class Obstacle(Static):
	# Child class of Static, it will be used for every obstacle
	def __init__(self):
		Static.__init__(self)


class RectangularObstacle(Obstacle):
	# Child class of Obstacle, it will be used for every rectangular obstacle
	def __init__(self):
		Obstacle.__init__(self)
		self.width = 0.
		self.height = 0.


class CircularObstacle(Obstacle):
	# Child class of Obstacle, it will be used for every circular obstacle
	def __init__(self):
		Obstacle.__init__(self)
		self.radius = 0.


class Controllable(Moving):
	# Child class of Moving, it is used for every body that can be controlled (issued commands)
	def __init__(self):
		Moving.__init__(self)
		# Sensor readings can be used by the controller in order to avoid collisions
		self.sensor_readings = []
		# A controller will be responsible for setting the commands to the values that will result in a desired behaviour
		self.commands = []


class WheeledVehicle(Controllable):
	# Child class of Controllable, it is used for every wheeled vehicle with an axis length
	def __init__(self):
		Controllable.__init__(self)
		self.axis_length = 0.
		self.axles_distance = 2.8 # By default lets put it with a car's axle distance


class ConstructionVehicle(WheeledVehicle):
	# Child class of WheeledVehicle, it will be used for the Construction Vehicles
	def __init__(self):
		WheeledVehicle.__init__(self)


# SmartVehicle and smartbus is now located in the files smartvehicle.py and smartbus.py respectively

class DummyVehicle(WheeledVehicle):
	# Child class of WheeledVehicle, it will be used for the Dummy Vehicles
	def __init__(self):
		WheeledVehicle.__init__(self)
		self.axles_distance = 2.8


class BusVehicle(WheeledVehicle):
	# Child class of WheeledVehicle, it will be used for the Bus Vehicles
	def __init__(self):
		WheeledVehicle.__init__(self)
		self.axles_distance = 6.0
		self.people_capacity = 20
		self.people_on_board = 0
		self.distance_to_next_stop = 0.
		self.current_bus_stop = None
		self.next_bus_stop = None
		self.ids_on_board = []


class Quadrotor(Controllable):
	# Child class of Controllable, it will be used for quads
	def __init__(self):
		Controllable.__init__(self)


class NonControllable(Moving):
	# Child class of Moving, it is used for bodies that don't have commands
	def __init__(self):
		Moving.__init__(self)


class Trailer(NonControllable):
	# Child class of NonControllable, it is used for the trailers
	def __init__(self):
		NonControllable.__init__(self)
		self.trailer_axle_length = 0.
