#!/usr/bin/env python
"""
Vehicle node of the SML-World.

Created on Mar 3, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import sys

import rospy


def vehicle(vehicle_id):
    """
    Initialize ROS-node 'vehicle' and register subs, pubs and services.

    @param vehicle_id
    """
    rospy.init_node('vehicle', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        vehicle_id = sys.argv[1]
    else:
        raise Exception("vehicle_id as argument expected.")
    vehicle(vehicle_id)
