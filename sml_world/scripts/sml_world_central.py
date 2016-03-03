#!/usr/bin/env python
# license removed for brevity
"""
Central node of the SML-World.

Created on Mar 1, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import rospy
from sml_world.msg import VehicleState, WorldState


def update_vehicle_state(vs, vs_dict):
    """
    Write received vehicle state into world state.

    @param vs: I{VehicleState} Vehicle state that needs to be updated in the
               world state.
    @param vs_dict: I{dict} Dict of all vehicle states that is updated.
    """
    print "vehicle_id:", vs.vehicle_id
    vs_dict[vs.vehicle_id] = vs


def sml_world():
    """Inizialize ROS-node 'sml_world' and start subs, pubs and srvs."""
    world_state = WorldState()
    vs_dict = {}  # Saves all vehicle states in a dict with vehicle_id as key
    rospy.init_node('sml_world')
    rospy.Subscriber('current_vehicle_state', VehicleState,
                     update_vehicle_state, vs_dict)
    pub_ws = rospy.Publisher('world_state', WorldState, queue_size=10)
    rate = rospy.Rate(60)  # 60hz
    while not rospy.is_shutdown():
        world_state.vehicle_states = vs_dict.values()
        pub_ws.publish(world_state)
        rate.sleep()


if __name__ == '__main__':
    sml_world()
