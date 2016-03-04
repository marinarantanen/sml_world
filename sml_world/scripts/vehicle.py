#!/usr/bin/env python
"""
Vehicle node of the SML-World.

Created on Mar 3, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import sys

import rospy

from sml_modules.basevehicles import BaseVehicle


def vehicle(vehicle_id):
    """
    Initialize ROS-node 'vehicle' and register subs, pubs and services.

    @param vehicle_id: I{(int)} ID of the vehicle that is created.
    """
    rospy.init_node('vehicle', log_level=rospy.WARN)
    BaseVehicle(rospy.get_namespace(), vehicle_id, 20)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        vehicle_id = sys.argv[1]
    else:
        msg = "Usage: rosrun sml_world vehicle.py <vehicle_id>"
        raise Exception(msg)
    vehicle(int(vehicle_id))
