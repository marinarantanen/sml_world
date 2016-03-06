#!/usr/bin/env python
"""
Vehicle node of the SML-World.

Created on Mar 3, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import sys

import rospy

from sml_modules.sensor_models import Radar


def sensor(sensor_type):
    """Initialize ROS-node 'sensor' and register subs and pubs."""
    rospy.init_node('sensor')
    if sensor_type == 'Radar':
        sensor = Radar(rospy.get_name())
    else:
        raise Exception("Unknown sensor typ %s." % sensor_type)
    rate = rospy.Rate(40)  # 40hz
    while not rospy.is_shutdown():
        sensor.publish_readings()
        if rate.remaining() < rospy.Duration(0):
            sens_name = sensor.__class__.__name__
            rospy.logwarn("Sensor %s was not able to work at " % sens_name +
                          "the desired rate.")
            rate.last_time = rospy.get_rostime()
        else:
            rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        sensor_type = sys.argv[1]
    else:
        msg = "Usage: rosrun sml_world sensor.py <sensor_type>"
        raise Exception(msg)
    sensor(sensor_type)
