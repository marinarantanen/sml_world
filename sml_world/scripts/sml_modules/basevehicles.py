"""Base classes for simulated vehicles."""
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
        self.axles_distance = 1.9
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
        # Find closest trajectory point, then set reference point 5 indices
        # ahead of the closest trajecctory point to imporove lateral controller
        # performance.  Use this trajectory pose as reference pose.
        closest_ind = self.find_closest_trajectory_pose() + 5
        traj_len = len(self.np_trajectory[0])
        ref_ind = closest_ind % traj_len
        ref_state = self.numpy_trajectory[:][ref_ind]
        # set controll commands.
        self.set_control_commands(ref_state)
        # update vehicle state.
        self.update_vehicle_state()
        # publish vehicle state.

    def find_closest_trajectory_pose(self):
        """
        Find closest point to the current vehicle position in the trajectory.

        @return: The index of the closest trajectory point to the current
                 vehicle position.
        """
        np_state = numpy.array([[self.x], [self.y]])
        temp_distance = numpy.sum(
                          (self.numpy_trajectory[0:2, :] - np_state) ** 2,
                          axis=0)
        best_idx = numpy.argmin(temp_distance)
        return best_idx

    def set_control_commands(self, ref_state):
        """Set the control commands, depending on the vehicles controler."""
        self.commands['speed'] = self.v
        dx = ref_state[0] - self.x
        dy = ref_state[1] - self.y
        dx_v = numpy.cos(self.yaw) * dx + numpy.sin(self.yaw) * dy
        dy_v = -numpy.sin(self.yaw) * dx + numpy.cos(self.yaw) * dy
        dyaw_v = ref_state[2] - self.yaw

        steering_command = dy_v + dyaw_v * 1/dx_v
        # Compare with max steering angle
        if steering_command > 0.5:
            steering_command = 0.5
        elif steering_command < -0.5:
            steering_command = -0.5
        self.commands['steering_angle'] = steering_command

    def update_vehicle_state(self):
        """Update the vehicle state."""
        sim_timestep = 1. / self.simulation_rate
        # Decompose v into x and y component.
        vx = numpy.cos(self.yaw) * self.v
        vy = numpy.sin(self.yaw) * self.v
        # Update vehicles position
        self.x += vx * sim_timestep
        self.y += vy * sim_timestep
        self.yaw += ((self.v / self.axles_distance) *
                     numpy.tan(self.commands['steering_angle']) *
                     sim_timestep)

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


def to_numpy_trajectory(trajectory):
    """Transform Pose2D[] message to numpy array."""
    tx = []
    ty = []
    tyaw = []
    for pose in trajectory:
        tx.append(pose.x)
        ty.append(pose.y)
        tyaw.append(pose.yaw)
    return numpy.asarray([tx, ty, tyaw])
