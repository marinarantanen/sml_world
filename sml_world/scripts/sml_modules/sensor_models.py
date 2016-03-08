"""
Module containing all available sensor classes.

Created on Mar 6, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import numpy as np

import rospy
from sml_world.msg import WorldState
from sml_world.msg import Pose2DPolar, RadarReadings

from std_msgs.msg import String


class BaseSensor(object):
    """
    Base class for sensors.

    New sensors inherit from this class which implements the subsciption
    to the world state and the processing of this subscription.
    """

    def __init__(self, vehicle_id):
        """
        Initialize class BaseSensor.

        @param vehicle_id: I{(int)} ID of the vehicle this sensor belongs to.
        """
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
        """
        Initialize Radar sensor class.

        @param vehicle_id: I{(int)} ID of the vehicle this sensor belongs to.
        @param name: I{(str)} Name of the sensor under which it will publish
                     its readings.
        @param sens_range: I{(float)} Range of the randar sensor.
        @param sens_angle: I{(float)} Opening angle of the radar sensor.
        """
        super(Radar, self).__init__(vehicle_id)
        self.name = name
        self.sens_range = float(sens_range)
        self.sens_angle = float(sens_angle) * np.pi/180.
        self.pub_readings = rospy.Publisher(self.name, RadarReadings,
                                            queue_size=10)

    def publish_readings(self):
        """Publish the sensor readings."""
        # Only go through this routine if vehicle is in state array.
        if (not np.any(self.vehicle_states[0] == self.vehicle_id) or
            not self.vehicle_states.shape[1] > 1):
            return
        # Get the current vehicle state.
        ind = self.vehicle_states[0].tolist().index(self.vehicle_id)
        self_state = self.vehicle_states[:, ind]
        vehicle_states = np.delete(self.vehicle_states, np.s_[ind], 1)
        rel_state = (vehicle_states[(1, 2, 3), :].transpose() -
                     self_state[[1, 2, 3]]).transpose()
        # Calculate rotation matrix
        yaw = self_state[3]
        rot = np.asmatrix([[np.cos(yaw), np.sin(yaw)],
                           [-np.sin(yaw), np.cos(yaw)]])
        # Transform current vehicle states.
        rel_pos = rot * rel_state[(0, 1), :]
        # Find states in range.
        to_polar = np.vectorize(lambda x, y: (np.hypot(x, y),
                                              np.arctan2(y, x)))
        rel_rho, rel_theta = to_polar(rel_pos[0, :], rel_pos[1, :])
        pol_state = np.concatenate((np.array(rel_rho),
                                    np.array(rel_theta),
                                    [rel_state[2, :]]),
                                   axis=0)
        pol_state = pol_state[:, np.all(
                                  (pol_state[0, :] <= self.sens_range,
                                   np.abs(pol_state[1, :]) <= self.sens_angle),
                                  axis=0)]
        # Send out message with relative poses.
        radar_readings = []
        for v in pol_state.transpose():
            radar_readings.append(Pose2DPolar(v[0], v[1], v[2]))
        self.pub_readings.publish(RadarReadings(radar_readings))


class Velodyne(BaseSensor):
    """Radar sensor class."""

    def __init__(self, vehicle_id, name, sens_range):
        """
        Initialize Velodyne sensor class.

        @param vehicle_id: I{(int)} ID of the vehicle this sensor belongs to.
        @param name: I{(str)} Name of the sensor under which it will publish
                     its readings.
        @param sens_range: I{(float)} Range of the velodyne sensor.
        """
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
        """
        Initialize Oniscient sensor class.

        @param vehicle_id: I{(int)} ID of the vehicle this sensor belongs to.
        @param name: I{(str)} Name of the sensor under which it will publish
                     its readings.
        """
        super(Omniscient, self).__init__(vehicle_id)
        self.name = name
        self.pub_readings = rospy.Publisher(self.name, String,
                                            queue_size=10)

    def publish_readings(self):
        """Publish the sensor readings."""
        self.pub_readings.publish("These are the sensor readings!")
