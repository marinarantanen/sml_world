"""Sensor module."""

import numpy as np

import rospy
from sml_world.msg import WorldState
from sml_world.msg import Pose2DPolar, RadarReadings

from std_msgs.msg import String


class BaseSensor(object):
    """Base class for sensors."""

    def __init__(self, vehicle_id):
        """Initialize class BaseSensor."""
        self.vehicle_id = int(vehicle_id)
        self.vehicle_states = np.asarray([[], [], [], [], []])
        rospy.Subscriber('/world_state', WorldState, self.update_state)

    def update_state(self, ws):
        """Callback function for topic 'world_state'."""
        self.vehicle_states = np.asarray([[], [], [], [], []])
        for vs in ws.vehicle_states:
            self.vehicle_states = np.concatenate(
                                        (self.vehicle_states,
                                         [[vs.vehicle_id], [vs.x], [vs.y],
                                          [vs.yaw], [vs.v]]),
                                        axis=1)


class Radar(BaseSensor):
    """Radar sensor class."""

    def __init__(self, vehicle_id, name, sens_range, sens_angle):
        """Initialize Radar sensor class."""
        super(Radar, self).__init__(vehicle_id)
        self.name = name
        self.sens_range = float(sens_range)
        self.sens_angle = float(sens_angle) * np.pi/180.
        self.pub_readings = rospy.Publisher(self.name, RadarReadings,
                                            queue_size=10)

    def publish_readings(self):
        """Publish the sensor readings."""
        self.pub_readings.publish("These are the sensor readings!")


class Velodyne(BaseSensor):
    """Radar sensor class."""

    def __init__(self, vehicle_id, name, sens_range):
        """Initialize Velodyne sensor class."""
        super(Radar, self).__init__(vehicle_id)
        self.name = name
        self.sens_range = sens_range
        self.pub_readings = rospy.Publisher(self.name, String,
                                            queue_size=10)

    def publish_readings(self):
        """Publish the sensor readings."""
        self.pub_readings.publish("These are the sensor readings!")


class Omniscient(BaseSensor):
    """Omnicient sensor class."""

    def __init__(self, vehicle_id, name):
        """Initialize Oniscient sensor class."""
        super(Omniscient, self).__init__(vehicle_id)
        self.name = name
        self.pub_readings = rospy.Publisher(self.name, String,
                                            queue_size=10)

    def publish_readings(self):
        """Publish the sensor readings."""
        self.pub_readings.publish("These are the sensor readings!")
