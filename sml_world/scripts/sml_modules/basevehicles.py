"""Base classes for simulated vehicles."""
import time
import random
import numpy

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from sml_world.msg import VehicleState
from sml_world.srv import SetVehicleState, SetVehicleStateResponse
from sml_world.srv import SetSpeed, SetSpeedResponse
from sml_world.srv import SetLoop, SetLoopResponse
from sml_world.srv import SetDestination, SetDestinationResponse
from sml_world.srv import GetTrajectory
# from sml_world.srv import PublishCom


from sml_modules.bodyclasses import WheeledVehicle


class BaseVehicle(WheeledVehicle):
    """Base class for all vehicles."""

    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=0.):
        """Initialize class BaseVehicle."""
        rospy.Subscriber(namespace + '/sensor_readings', String,
                         self.process_sensor_readings)
        rospy.Subscriber(namespace + '/receivable_com', String,
                         self.process_receivable_com)

        self.pub_state = rospy.Publisher(namespace + '/current_vehicle_state',
                                         VehicleState, queue_size=10)

        rospy.Service(namespace + '/set_state', SetVehicleState,
                      self.handle_set_state)
        rospy.Service(namespace + 'set_speed', SetSpeed,
                      self.handle_set_speed)
        rospy.Service(namespace + 'set_loop', SetLoop, self.handle_set_loop)
        rospy.Service(namespace + 'set_destination', SetDestination,
                      self.handle_set_destination)
        rospy.Service(namespace + 'start_simulation', Trigger,
                      self.handle_start_simulation)
        # rospy.wait_for_service(namespace + '/publish_com')
        # self.publish_com = rospy.ServiceProxy(namespace + '/publish_com',
        #                                       PublishCom)

        self.vehicle_id = vehicle_id
        self.class_name = self.__class__
        self.simulation_rate = simulation_rate

        # Set parameters of base vehicle to default values.
        self.simulate = False
        self.sensors = []
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.np_trajectory = []
        self.commands = []

    def simulation_loop(self):
        """The simulation loop of the car."""
        rate = rospy.Rate(self.simulation_rate)
        while not rospy.is_shutdown():
            self.simulation_step()
            if rate.remaining() < rospy.Duration(0):
                rospy.logwarn("Simulation rate of vehicle " +
                              "#%i " % self.vehicle_id +
                              "could not be achieved.")
                rate.last_time = rospy.get_rostime()
            else:
                rate.sleep()

    def simulation_step(self):
        """Simulate one timestep of the car."""
        # Find closest trajectory point.

        # Set reference point 5 indices ahead of the closest trajecctory point
        # to imporove lateral controller performance.
        # Get reference state.

        # Drive at the cruise velocity.
        time.sleep(random.random() / 2.)

    def process_sensor_readings(self, data):
        """Process all sensor readings."""
        print data.data

    def process_receivable_com(self, data):
        """Process the receivable communication."""
        print data.data

    def handle_set_state(self, req):
        """
        Handle the set state request.

        @param req: I{(SetState)} Request of the service that sets the vehicle
                    state.
        """
        self.x = req.x
        self.y = req.y
        self.yaw = req.yaw
        self.v = req.v
        msg = "State of vehicle #%i successfully set." % self.vehicle_id
        return SetVehicleStateResponse(True, msg)

    def handle_set_speed(self, req):
        """
        Handle the set speed request.

        @param req: I{(SetSpeed)} Request of the service that sets the vehicles
                    cruising speed.
        """
        self.v = req.speed
        msg = "Speed of vehicle #%i successfully set." % self.vehicle_id
        return SetSpeedResponse(True, msg)

    def handle_set_loop(self, req):
        """
        Handle the set closed loop request.

        @param req: I{(SetLoop)} Request of the service that sets the vehicles
                    closed loop trajectory.
        """
        rospy.wait_for_service('get_tranjectory')
        try:
            get_traj = rospy.ServiceProxy('get_tranjectory', GetTrajectory)
            trajectory = get_traj(True, req.node_id, 0).trajectory
        except rospy.ServiceException, e:
            raise "Service call failed: %s" % e
        self.numpy_trajectory = to_numpy_trajectory(trajectory)
        msg = ("Closed loop trajectory of vehicle #%i " % self.vehicle_id +
               "successfully set.")
        return SetLoopResponse(True, msg)

    def handle_set_destination(self, req):
        """
        Handle the set destination request.

        @param req: I{(SetDestination)} Request of the service that sets the
                    vehicles trajectory to a specific destination.
        """
        rospy.wait_for_service('get_tranjectory')
        try:
            get_traj = rospy.ServiceProxy('get_tranjectory', GetTrajectory)
            current_node = None
            trajectory = get_traj(False, current_node, req.dest_id).trajectory
        except rospy.ServiceException, e:
            raise "Service call failed: %s" % e
        self.numpy_trajectory = to_numpy_trajectory(trajectory)
        msg = ("Trajectory to destination of vehicle #%i " % self.vehicle_id +
               "successfully set.")
        return SetDestinationResponse(True, msg)

    def handle_start_simulation(self, req):
        """
        Handle the start simulation request.

        @param req: I{(Trigger)} Request to start the vehicle simulation.
        """
        return TriggerResponse()


def to_numpy_trajectory(self, trajectory):
    """Transform Pose2D[] message to numpy array."""
    tx = []
    ty = []
    tyaw = []
    for pose in trajectory:
        tx.append(pose.x)
        ty.append(pose.y)
        tyaw.append(pose.yaw)
    return numpy.asarray([tx, ty, tyaw])
