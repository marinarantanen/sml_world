#!/usr/bin/env python
# license removed for brevity
"""
Visualization node of the SML-World.

Created on Feb 23, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import rospy
from sml_world.msg import WorldState
from sml_world.srv import GetMapLocation

from sml_modules.visualization_module import Visualization


def update_state(ws, vis_module):
    """
    Callback function for topic 'world_state'.

    @param ws: I{(WorldState)} ROS-message of the world state.
    @param vis_module: I{(VisualisationModule)} The initialized visualization
                       module used to show the current state of the simulation.

    @todo: Integrate ROS-messages for the world state.
    """
    ws_dict = {}
    for vs in ws.vehicle_states:
        ws_dict[vs.vehicle_id] = {}
        ws_dict[vs.vehicle_id]['id'] = vs.vehicle_id
        ws_dict[vs.vehicle_id]['class_name'] = vs.class_name
        ws_dict[vs.vehicle_id]['x'] = vs.x
        ws_dict[vs.vehicle_id]['y'] = vs.y
        ws_dict[vs.vehicle_id]['yaw'] = vs.yaw
    vis_module.loop_iteration(ws_dict)


def visualizer(vis_module):
    """
    Initialize ROS-node 'visualizer' and start subscriber to 'world_state'.

    @param vis_module: I{(VisualisationModule)} The initialized visualization
                       module used to show the current state of the simulation.
    """
    rospy.init_node('visualizer')
    rospy.Subscriber('world_state', WorldState, update_state, vis_module)
    rospy.loginfo("ROS-node 'visualizer' start spinning.")
    rospy.spin()


if __name__ == '__main__':
    # Request the map location.
    rospy.wait_for_service('get_map_location')
    try:
        get_map = rospy.ServiceProxy('get_map_location', GetMapLocation)
        resp = get_map()
        base_path = resp.base_path
        map_location = resp.map_location
    except rospy.ServiceException, e:
        raise "Service call failed: %s" % e
    # Initialize the visualization module
    vis_module = Visualization(base_path, map_location, 800, 600, 5, True)
    print "vis_module_started."
    vis_module.loop_iteration({})
    visualizer(vis_module)
