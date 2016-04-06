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
from sml_world.srv import GetNearestNodeId, GetNearestNodeIdResponse

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

        @param file_location: I{(str)} Relative location of the .xml file
                              containing the road network information.
        """
        base_path = os.path.dirname(__file__)
        self.map_location = file_location
        super(RoadModuleExtend, self).__init__(base_path, file_location)
        self.xy_to_node = {(n_osm.x, n_osm.y): n_id for n_osm, n_id
                           in zip(self.osm_node_dict.values(),
                                  self.osm_node_dict.keys())}

    def handle_get_map_location(self, req):
        """
        Handle the map location request.

        @param req: I{(GetMapLocation)} Request of the service that provides
                    the map location to client.
        """
        return GetMapLocationResponse(self.base_path, self.map_location)

    def handle_get_trajectory(self, req):
        """
        Handle the get trajectory request.

        @param req: I{(GetTrajectory)} Request of the service that provides a
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
            trajectory.append(Pose2D(x, y, yaw))
        return GetTrajectoryResponse(trajectory)

    def handle_get_nearest_nodeid(self, req):
        """
        Get the node ID that is nearest to a vehicles location.

        @param req: I{(GetNearestNodeId)} Request of the service that provides
                    the ID of the node nearest to the sent coordinates.
        """
        min_dist = float('inf')
        nearest_xy = None
        for x, y in self.xy_to_node.keys():
            min_dist_tmp = math.sqrt(math.pow(x-req.x, 2) +
                                     math.pow(y-req.y, 2))
            if min_dist > min_dist_tmp:
                nearest_xy = (x, y)
                min_dist = min_dist_tmp
        return GetNearestNodeIdResponse(self.xy_to_node[nearest_xy])


def road_network(file_location):
    """
    Initialize ROS-node 'road_network' and start the services.

    @param file_location: I{(str)} Location of the road network description
                          that should be loaded.
    """
    road_module = RoadModuleExtend(file_location)

    rospy.init_node('road_network')
    rospy.Service('/get_map_location', GetMapLocation,
                  road_module.handle_get_map_location)
    rospy.Service('/get_trajectory', GetTrajectory,
                  road_module.handle_get_trajectory)
    rospy.Service('/get_nearest_nodeid', GetNearestNodeId,
                  road_module.handle_get_nearest_nodeid)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        file_location = sys.argv[1]
    else:
        msg = "Usage: rosrun sml_world road_network.py <file_location>"
        raise Exception(msg)
    road_network(file_location)
