#!/usr/bin/env python
# license removed for brevity
"""
Central node of the SML-World.

Created on Mar 1, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""
import Queue

import rospy
from roslaunch.scriptapi import ROSLaunch
from roslaunch.core import Node
from sml_world.msg import VehicleState, WorldState
from sml_world.srv import SpawnVehicle, SpawnVehicleResponse
from sml_world.srv import SetLoop
#from sml_world.srv import SetDestination
from sml_world.srv import SetBool


class ROSLaunchExtended(ROSLaunch):
    """Extended version of the ROSLaunch class."""

    def __init__(self):
        """Initialize the ROSLaunchExtended class."""
        super(ROSLaunchExtended, self).__init__()
        rospy.Service('spawn_vehicle', SpawnVehicle, self.handle_spawn_vehicle)
        # Working with a launch queue is necessary, because self.launch()
        # only works in the main thread.  Service calls spawn sidethreads.
        self.launch_queue = Queue.Queue()
        self.launched_processes = []
        self.start()

    def handle_spawn_vehicle(self, req):
        """
        Spawn new vehicle.

        This includes starting a new vehicle node and (if a looping node is
        set) calling its /set_loop service, as well as its /toggle_simulation
        service.

        @param req: I{(SpawnVehicle)} Request message of the service that
                    spawns a new vehicle.
        """
        namespace = "vehicle_" + str(req.vehicle_id)
        args = "%i %s %f %f %f %f" % (req.vehicle_id, req.class_name,
                                      req.x, req.y, req.yaw, req.v)
        node = Node('sml_world', 'vehicle.py',
                    namespace=namespace, args=args,
                    name=req.class_name.lower())
        self.launch_queue.put(node)
        loop_service = '/' + namespace + '/set_loop'
        rospy.wait_for_service(loop_service)

        if bool(req.node_id):
            try:
                set_loop = rospy.ServiceProxy(loop_service, SetLoop)
                set_loop(req.node_id)
            except rospy.ServiceException, e:
                raise "Service call failed: %s" % e
        toggle_sim_serv = '/' + namespace + '/toggle_simulation'
        rospy.wait_for_service(toggle_sim_serv)
        try:
            toggle_sim = rospy.ServiceProxy(toggle_sim_serv, SetBool)
            toggle_sim(req.toggle_sim)
        except rospy.ServiceException, e:
            raise "Service call failed: %s" % e
        msg = ("Vehicle #%i was successfully spawned." % req.vehicle_id)
        return SpawnVehicleResponse(True, msg)

    def spawn_vehicle(self):
        """Spawn vehicle node."""
        node = self.launch_queue.get()
        self.launched_processes.append(self.launch(node))


def update_vehicle_state(vs, vs_dict):
    """
    Write received vehicle state into world state.

    @param vs: I{(VehicleState)} Vehicle state that needs to be updated in the
               world state.
    @param vs_dict: I{(dict)} Dict of all vehicle states that is updated.
    """
    vs_dict[vs.vehicle_id] = vs


def sml_world_central():
    """Inizialize ROS-node 'sml_world' and start subs, pubs and srvs."""
    world_state = WorldState()
    vs_dict = {}  # Saves all vehicle states in a dict with vehicle_id as key
    rospy.init_node('sml_world', log_level=rospy.WARN)
    rospy.Subscriber('current_vehicle_state', VehicleState,
                     update_vehicle_state, vs_dict)

    launcher = ROSLaunchExtended()

    pub_ws = rospy.Publisher('world_state', WorldState, queue_size=10)
    rate = rospy.Rate(60)  # 60hz
    while not rospy.is_shutdown():
        while not launcher.launch_queue.empty():
            launcher.spawn_vehicle()
            #launcher.spawn_vehicleAtoB()
        world_state.vehicle_states = vs_dict.values()
        pub_ws.publish(world_state)
        if rate.remaining() < rospy.Duration(0):
            rospy.logwarn("SML-World central could not keep up with the " +
                          "update rate aimed for.")
            rate.last_time = rospy.get_rostime()
        else:
            rate.sleep()


if __name__ == '__main__':
    sml_world_central()
