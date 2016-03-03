"""Base classes for simulated vehicles."""

import rospy
from std_msgs.msg import String
from sml_world.msg import VehicleState
from sml_world.srv import PublishCom

from sml_modules.bodyclasses import WheeledVehicle


class BaseVehicle(WheeledVehicle):
    """Base class for all vehicles."""

    def __init__(self, namespace):
        """Initialize class BaseVehicle."""
        rospy.Subscriber(namespace + '/sensor_readings', String)
        rospy.Subscriber(namespace + '/receivable_com', String)

        self.pub_state = rospy.Publisher(namespace + "/current_vehicle_state",
                                         VehicleState)

        rospy.wait_for_service(namespace + '/publish_com')
        self.publish_com = rospy.ServiceProxy(namespace + '/publish_com',
                                              PublishCom)

    def process_sensor_readings(self, data):
        """Process all sensor readings."""
        print data.data

    def process_receivable_com(self, data):
        """Process the receivable communication."""
        print data.data
