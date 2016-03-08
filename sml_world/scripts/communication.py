#!/usr/bin/env python
"""
Communication node of the SML-World.

Created on Mar 8, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import sys

import rospy

from sml_modules.com_models import Wifi


def communication(vehicle_id, com_type, opt_argv):
    """Initialize ROS-node 'communication' and register subs and pubs."""
    rospy.init_node('communication')
    if com_type == 'Wifi':
        Wifi(vehicle_id, rospy.get_name(), *opt_argv)
    else:
        raise Exception("Unknown communication type %s." % com_type)
    """
    rate = rospy.Rate(40)  # 40 hz
    while not rospy.is_shutdown():
        if rate.remaining() < rospy.Duration(0):
            sens_name = com.__class__.__name__
            rospy.logwarn("Communication %s was not able to " % sens_name +
                          "work at the desired rate.")
            rate.last_time = rospy.get_rostime()
        else:
            rate.sleep()
    """
    rospy.spin()


if __name__ == '__main__':
    # Filter sys.argv to remove automatically added arguments
    sys.argv = [arg for arg in sys.argv if str(arg).find(':=') < 0]
    if len(sys.argv) > 2:
        vehicle_id = int(sys.argv[1])
        com_type = sys.argv[2]
    else:
        msg = ("Usage: rosrun sml_world communication.py <vehicle_id> " +
               "<com_type> [<opt_params>].")
        raise Exception(msg)
    communication(vehicle_id, com_type, sys.argv[3:])
