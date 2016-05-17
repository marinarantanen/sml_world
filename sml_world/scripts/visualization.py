#!/usr/bin/env python
# license removed for brevity
"""
Visualization node of the SML-World.

Created on Feb 23, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import rospy
#import roslaunch

import mocap

import sys
import signal

from sml_world.msg import WorldState
from sml_world.srv import GetMapLocation

from sml_modules.visualization_module import Visualization

INITIAL_TIME = 700

def qualisys_info():
    """
    Qualisys data listener.
    @param qs_body:
    """
    stop = False

    def signal_handler(signal, frame):
        stop = True
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    Qs = mocap.Mocap(info=1)
    bodies = Qs.find_available_bodies(printinfo=1)

    #pick the first valid body
    id_body = Qs.get_id_from_name("Iris2")

    body = mocap.Body(Qs, id_body)

    while not stop:
        pose = body.getPose()
        #rospy.loginfo(pose)
        #rospy.logwarn(pose)

#    qs_dict = {}
#    qs_dict[qs_body]['x']
#    qs_dict[qs_body]['y']
#    qs_dict[qs_body]['z']
#    qs_dict[qs_body]['yaw']


def qualisys_info():
    """
    Qualisys data listener.
    @param qs_body:
    """
    stop = False

    def signal_handler(signal, frame):
        stop = True
        print "What?"
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    Qs = mocap.Mocap(info=1)
    bodies = Qs.find_available_bodies(printinfo=1)

    #pick the first valid body
    id_body = Qs.get_id_from_name("Iris2")

    body = mocap.Body(Qs, id_body)

    while not stop:
        pose = body.getPose()
        #rospy.loginfo(pose)
        #rospy.logwarn(pose)

#    qs_dict = {}
#    qs_dict[qs_body]['x']
#    qs_dict[qs_body]['y']
#    qs_dict[qs_body]['z']
#    qs_dict[qs_body]['yaw']


def update_state(ws, vis_module):
    """
    Callback function for topic 'world_state'.

    @param ws: I{(WorldState)} ROS-message of the world state.
    @param vis_module: I{(VisualisationModule)} The initialized visualization
                       module used to show the current state of the simulation.

    @todo: Integrate ROS-messages for the world state.
    """
    ws_dict = {}
    ws_dict['vehicles'] = {}
    for vs in ws.vehicle_states:
        ws_dict['vehicles'][vs.vehicle_id] = {}
        ws_dict['vehicles'][vs.vehicle_id]['id'] = vs.vehicle_id
        ws_dict['vehicles'][vs.vehicle_id]['class_name'] = vs.class_name
        ws_dict['vehicles'][vs.vehicle_id]['x'] = vs.x
        ws_dict['vehicles'][vs.vehicle_id]['y'] = vs.y
        ws_dict['vehicles'][vs.vehicle_id]['yaw'] = vs.yaw
    ws_dict['bus_stop_ids'] = ws.bus_stop_ids
    ws_dict['bus_stop_demands'] = ws.bus_stop_demands
    ws_dict['time'] = ws.time
    vis_module.loop_iteration(ws_dict)

    for td in ws.traffic_demand:
        ws_dict[td.bus_demand]['bus_demand'] = td.bus_demand


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
    rospy.wait_for_service('/get_map_location')
    try:
        get_map = rospy.ServiceProxy('/get_map_location', GetMapLocation)
        resp = get_map()
        base_path = resp.base_path
        map_location = resp.map_location
    except rospy.ServiceException, e:
        raise "Service call failed: %s" % e
    # Initialize the visualization module
<<<<<<< HEAD
    vis_module = Visualization(base_path, map_location, 800, 600, 5, True)
#    vis_module = Visualization(base_path, map_location, 1820, 1380, 5, True)
=======
    vis_module = Visualization(base_path, map_location, 1024, 768, 5, True)
>>>>>>> eeae3cb5dc6d5f0c77dec60e8b06225f1d25d921
    print "vis_module_started."
    #First loop with empty values
    vis_module.loop_iteration({'vehicles' : {}, 'bus_stop_ids' : [], 'bus_stop_demands' : [], 'time': INITIAL_TIME})
    visualizer(vis_module)
    qualisys_info()
