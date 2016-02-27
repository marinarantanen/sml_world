#!/usr/bin/env python
# license removed for brevity
"""
Visualization mode of the SML-World.

Created on Feb 23, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import os
import json

import rospy
from std_msgs.msg import String

from sml_modules.visualization_module import Visualization
from sml_modules.road_module import RoadModule

def update_state(data, vis_module):
    """
    Callback function for the topic 'world_state' is called every time a new
    message is published.
    
    @param data: I{String} Json string that represents the world state.
    @param vis_module: I{VisualisationModule} The initialized visualization
                       module used to show the current state of the simulation.
    """
    #print json.loads(data.data)
    world_state = {}
    ws = json.loads(data.data)
    for v in ws:
        world_state[int(v)] = ws[v]
    vis_module.loop_iteration(world_state)
    
def visualizer(vis_module):
    """
    Function to initialize the new ROS-node 'visualizer' and start the
    subscriber for the topic 'world_state'.
    
    @param vis_module: I{VisualisationModule} The initialized visualization
                       module used to show the current state of the simulation.
    """
    rospy.init_node('visualizer', anonymous=True)
    rospy.Subscriber('world_state', String, update_state, vis_module)
    rospy.spin()

if __name__ == '__main__':
    # Initialize road
    base_path = os.path.dirname(__file__)
    file_location = "/resources/scenarios/HighwaySML"
    road_module = RoadModule.RoadModule(base_path, file_location)
    # Initialize the visualization module
    vis_module = Visualization(base_path, file_location, 800, 600, 5, True)
    print "vis_module_started."
    vis_module.loop_iteration({})
    visualizer(vis_module)

