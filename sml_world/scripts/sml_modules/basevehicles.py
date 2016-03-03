"""Base classes for simulated vehicles."""
import time
import random

import rospy
from std_msgs.msg import String
from sml_world.msg import VehicleState
# from sml_world.srv import PublishCom

from sml_modules.bodyclasses import WheeledVehicle


class BaseVehicle(WheeledVehicle):
    """Base class for all vehicles."""

    def __init__(self, namespace, vehicle_id, simulation_rate):
        """Initialize class BaseVehicle."""
        rospy.Subscriber(namespace + '/sensor_readings', String,
                         self.process_sensor_readings)
        rospy.Subscriber(namespace + '/receivable_com', String,
                         self.process_receivable_com)

        self.pub_state = rospy.Publisher(namespace + "/current_vehicle_state",
                                         VehicleState, queue_size=10)

        # rospy.wait_for_service(namespace + '/publish_com')
        # self.publish_com = rospy.ServiceProxy(namespace + '/publish_com',
        #                                       PublishCom)

        self.vehicle_id = vehicle_id
        self.simulation_rate = simulation_rate
        self.sensors = []

        self.simulation_loop()

    def simulation_loop(self):
        """The simulation loop of the car."""
        rate = rospy.Rate(self.simulation_rate)
        while not rospy.is_shutdown():
            self.simulation_step()
            if rate.remaining() < rospy.Duration(0):
                rospy.logwarn("Simulation rate of vehicle " +
                              "#%i " % self.vehicle_id +
                              "could not be achieved.")
                rate.last_time = rospy.get_rostime()
            else:
                rate.sleep()

    def simulation_step(self):
        """Simulate one timestep of the car."""
        # Find closest trajectory point.

        # Set reference point 5 indices ahead of the closest trajecctory point
        # to imporove lateral controller performance.
        # Get reference state.

        # Drive at the cruise velocity.
        time.sleep(random.random() / 2.)

    def process_sensor_readings(self, data):
        """Process all sensor readings."""
        print data.data

    def process_receivable_com(self, data):
        """Process the receivable communication."""
        print data.data
