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

import rospy
from sml_world.srv import GetMapLocation, GetMapLocationResponse

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
        self.map_location = base_path + file_location
        super(RoadModuleExtend, self).__init__(base_path, file_location)

    def handle_get_map_location(self, req):
        """
        Handle the map location request.

        @param req: I{GetMapLocation} Request of the service that provides the
                    map location to client.
        """
        return GetMapLocationResponse(self.map_location)


def road_network(file_location):
    """Initialize ROS-node 'road_network' and start the services."""
    road_module = RoadModuleExtend(file_location)

    rospy.init_node('road_network', anonymous=True)
    rospy.Service('get_map_location', GetMapLocation,
                  road_module.handle_get_map_location)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        file_location = sys.argv[1]
    else:
        raise Exception("file_location as argument expected.")
    road_network(file_location)
