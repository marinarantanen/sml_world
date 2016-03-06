"""Sensor module."""

import numpy

import rospy
from sml_world.msg import WorldState

from std_msgs.msg import String


class BaseSensor(object):
    """Base class for sensors."""

    def __init__(self):
        """Initialize class BaseSensor."""
        self.vehicle_poses = numpy.asarray([[], [], []])
        rospy.Subscriber('/world_state', WorldState, self.update_state)

    def update_state(self, ws):
        """Callback function for topic 'world_state'."""
        self.vehicle_poses = numpy.asarray([[], [], []])
        for vs in ws.vehicle_states:
            numpy.concatenate((self.vehicle_poses, [[vs.x], [vs.y], [vs.yaw]]),
                              axis=1)


class Radar(BaseSensor):
    """Radar sensor class."""

    def __init__(self, name):
        """Initialize class."""
        super(Radar, self).__init__()
        self.name = name
        self.pub_readings = rospy.Publisher(self.name, String,
                                            queue_size=10)

    def publish_readings(self):
        """Publish the sensor readings."""
        self.pub_readings.publish("These are the sensor readings!")
