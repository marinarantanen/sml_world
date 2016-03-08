#!/usr/bin/env python
"""
Sensor node of the SML-World.

Created on Mar 6, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import sys

import rospy

from sml_modules.sensor_models import Radar


def sensor(vehicle_id, sensor_type, opt_argv):
    """
    Initialize ROS-node 'sensor' and register subs and pubs.

    @param sensor_type: I{(str)} Name of the sensor class that should be
                        started up.
    @param opt_argv: I{(list)} List of optional parameters that should be
                     passed to the sensor initialization.
    """
    rospy.init_node('sensor')
    if sensor_type == 'Radar':
        sensor = Radar(vehicle_id, rospy.get_name(), *opt_argv)
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
    # Filter sys.argv to remove automatically added arguments
    sys.argv = [arg for arg in sys.argv if str(arg).find(':=') < 0]
    if len(sys.argv) > 2:
        vehicle_id = sys.argv[1]
        sensor_type = sys.argv[2]
    else:
        msg = ("Usage: rosrun sml_world sensor.py <vehicle_id> " +
               "<sensor_type> [<opt_params>].")
        raise Exception(msg)
    sensor(vehicle_id, sensor_type, sys.argv[3:])
