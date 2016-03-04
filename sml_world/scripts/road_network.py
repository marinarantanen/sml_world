#!/usr/bin/env python
# license removed for brevity
"""
RoadModule node of the SML-World.

Created on Feb 29, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import sys
import os
import math

import rospy
from sml_world.msg import Pose2D
from sml_world.srv import GetMapLocation, GetMapLocationResponse
from sml_world.srv import GetTrajectory, GetTrajectoryResponse

from sml_modules.road_module.RoadModule import RoadModule


class RoadModuleExtend(RoadModule):
    """
    Extension of the class RoadModule.

    This extension is done to be able to add more function parameters to the
    handle_get_map_location method.
    """

    def __init__(self, file_location):
        """
        Initialize the class RoadModule_new.

        @param file_location: I{str} Relative location of the .xml file
                              containing the road network information.
        """
        base_path = os.path.dirname(__file__)
        self.map_location = file_location
        super(RoadModuleExtend, self).__init__(base_path, file_location)

    def handle_get_map_location(self, req):
        """
        Handle the map location request.

        @param req: I{GetMapLocation} Request of the service that provides the
                    map location to client.
        """
        return GetMapLocationResponse(self.base_path, self.map_location)

    def handle_get_trajectory(self, req):
        """
        Handle the get trajectory request.

        @param req: I{GetTrajectory} Request of the service that provides a
                    trajectory to follow.
        """
        if req.loop:
            print "get_closed_path_from_node_id"
            tx, ty = self.get_closed_path_from_node_id(req.start_id)
        else:
            print "get_path_between_node_ids"
            tx, ty = self.get_path_between_node_ids(req.start_id,
                                                    req.end_id)
        trajectory = []
        if tx:
            # txp[i] = tx[i+1]
            txp = tx[1:]+[tx[0]]
            # typ[i] = ty[i+1]
            typ = ty[1:]+[ty[0]]
        else:
            txp = []
            typ = []
        for x, y, xp, yp in zip(tx, ty, txp, typ):
            dx = xp - x
            dy = yp - y
            yaw = math.atan2(dy, dx)
            # Make sure yaw is never negative.
            # yaw 0..2pi
            if yaw < 0.:
                yaw += 2*math.pi
                print "->", yaw
            trajectory.append(Pose2D(x, y, yaw))
        return GetTrajectoryResponse(trajectory)


def road_network(file_location):
    """Initialize ROS-node 'road_network' and start the services."""
    road_module = RoadModuleExtend(file_location)

    rospy.init_node('road_network')
    rospy.Service('get_map_location', GetMapLocation,
                  road_module.handle_get_map_location)
    rospy.Service('get_trajectory', GetTrajectory,
                  road_module.handle_get_trajectory)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        file_location = sys.argv[1]
    else:
        msg = "Usage: rosrun sml_world road_network.py <file_location>"
        raise Exception(msg)
    road_network(file_location)
