"""
Module containing classes for all simulated vehicles.

Created on Mar 5, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import numpy
import threading

import rospy
from roslaunch.scriptapi import ROSLaunch
from roslaunch.core import Node
import sml_world.msg as msgs
from sml_world.msg import VehicleState, TrafficDemand
from sml_world.srv import SetBool, SetBoolResponse
from sml_world.srv import SetVehicleState, SetVehicleStateResponse
from sml_world.srv import SetSpeed, SetSpeedResponse
from sml_world.srv import SetLoop, SetLoopResponse
from sml_world.srv import SetDestination, SetDestinationResponse
from sml_world.srv import SetDemand, SetDemandResponse
from sml_world.srv import GetTrajectory
# from sml_world.srv import PublishCom

from sml_modules.bodyclasses import WheeledVehicle


class BaseVehicle(WheeledVehicle):
    """
    Base class for all vehicles.

    Other vehicles inherit from this class and can use/overwrite its
    functions.  It provides the following basic services:
        - /set_state: Set the vehicle state.
        - /set_speed_kph: Set the vehicles speed in kilometers per hour.
        - /set_loop: Set a closed loop trajectory from a certain node.
        - /set_destination: Set a trajectory to a certain destination node.
        - /toggle_simulation: Toggle the simulation of the vehicle on/off.

    The launch_sensor function can be called from child classes to launch
    the sensor nodes that are listed in their class variable self.sensors.
    """

    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=0.):
        """
        Initialize class BaseVehicle.

        @param namespace: I{(string)} Namespace in which the vehicle node is
                          started.
        @param vehicle_id: I{(int)} ID of the vehicle that is created.
        @param simulation_rate: I{(int)} Rate at which the vehicle is
                                simulated (hz)
        @param x: I{(float)} x-coordinate at which the vehicle starts.
        @param y: I{(float)} y-coordinate at which the vehicle starts.
        @param yaw: I{(float)} Initial yaw of the vehicle.
        @param v: I{(float)} Initial velocity of the vehicle.
        """
        self.launcher = ROSLaunch()
        self.launcher.start()

        self.namespace = namespace

        self.vehicle_id = int(vehicle_id)
        self.class_name = self.__class__.__name__
        self.simulation_rate = simulation_rate

        # Set parameters of base vehicle to default values.
        self.simulate = False
        self.sensors = []
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.cruising_speed = v
        self.axles_distance = 1.9
        self.np_trajectory = []
        self.commands = {}

        # Start the simulation loop in a separate thread.
        sim_thread = threading.Thread(target=self.simulation_loop)
        sim_thread.daemon = True
        sim_thread.start()

        # Register all services, pubs and subs last to prevent attempts to use
        # the services before the initialization of the vehicle is finished.
        self.pub_state = rospy.Publisher('/current_vehicle_state',
                                         VehicleState, queue_size=10)
        self.pub_demand = rospy.Publisher('/current_demand', TrafficDemand,
                                         queue_size=10)
        rospy.Service(self.namespace + 'set_state', SetVehicleState,
                      self.handle_set_state)
        rospy.Service(self.namespace + 'set_speed_kph', SetSpeed,
                      self.handle_set_speed_kph)
        rospy.Service(self.namespace + 'set_loop', SetLoop,
                      self.handle_set_loop)
        rospy.Service(self.namespace + 'set_destination', SetDestination,
                      self.handle_set_destination)
        rospy.Service(self.namespace + 'toggle_simulation', SetBool,
                      self.handle_toggle_simulation)

        rospy.Service(self.namespace + 'set_demand', SetDemand,
                      self.handle_set_demand)
        # rospy.wait_for_service(self.namespace + '/publish_com')
        # self.publish_com = rospy.ServiceProxy(self.namespace + 'publish_com',
        #                                       PublishCom)

    def simulation_loop(self):
        """The simulation loop of the car."""
        rate = rospy.Rate(self.simulation_rate)
        while not rospy.is_shutdown():
            # print self.x, self.y, self.yaw, self.v
            # Simulate only if the simulate flat is set.
            if self.simulate:
                self.simulation_step()
            # Check if simulatio rate could be achieved or not.
            if rate.remaining() < rospy.Duration(0):
                rospy.logwarn("Simulation rate of vehicle " +
                              "#%i " % self.vehicle_id +
                              "could not be achieved.")
                rate.last_time = rospy.get_rostime()
            else:
                rate.sleep()

    def simulation_step(self):
        """Simulate one timestep of the car."""
        # Find closest trajectory point, then set reference point some indices
        # ahead of the closest trajecctory point to imporove lateral controller
        # performance.  Use this trajectory pose as reference pose.
        closest_ind = (self.find_closest_trajectory_pose() +
                       numpy.round(self.v / 4))
        traj_len = len(self.np_trajectory[0])
        ref_ind = closest_ind % traj_len
        ref_state = self.np_trajectory[:, ref_ind]
        # set controll commands.
        self.set_control_commands(ref_state)
        # update vehicle state.
        self.update_vehicle_state()
        # publish vehicle state.
        vehicle_state = VehicleState(self.vehicle_id, self.class_name,
                                     self.x, self.y, self.yaw, self.v)
        self.pub_state.publish(vehicle_state)

    def find_closest_trajectory_pose(self):
        """
        Find closest point to the current vehicle position in the trajectory.

        @return: The index of the closest trajectory point to the current
                 vehicle position.
        """
        np_state = numpy.array([[self.x], [self.y]])
        temp_distance = numpy.sum(
                          (self.np_trajectory[0:2, :] - np_state) ** 2,
                          axis=0)
        best_idx = numpy.argmin(temp_distance)
        return best_idx

    def set_control_commands(self, ref_state):
        """
        Set the control commands, depending on the vehicles controler.

        @param ref_state: I{(numpy array)} Reference state [x, y, yaw] that
                          the vehicle tries to reach.
        """
        self.commands['speed'] = self.cruising_speed
        dx = ref_state[0] - self.x
        dy = ref_state[1] - self.y
        dx_v = numpy.cos(self.yaw) * dx + numpy.sin(self.yaw) * dy
        dy_v = -numpy.sin(self.yaw) * dx + numpy.cos(self.yaw) * dy
        dyaw_v = ref_state[2] - self.yaw
        # Correct yaw difference. dyaw_v 0..pi
        while dyaw_v > numpy.pi:
            dyaw_v -= 2*numpy.pi
        while dyaw_v < -numpy.pi:
            dyaw_v += 2*numpy.pi
        # Calculate steering command from dy_v, dx_v and dyaw_v
        steering_command = dy_v + dyaw_v * 1.5 / (1 + dx_v)
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
        self.v = self.commands['speed']
        vx = numpy.cos(self.yaw) * self.v
        vy = numpy.sin(self.yaw) * self.v
        # Update vehicles position
        self.x += vx * sim_timestep
        self.y += vy * sim_timestep
        self.yaw += ((self.v / self.axles_distance) *
                     numpy.tan(self.commands['steering_angle']) *
                     sim_timestep)
        # Make sure self.yaw is never negative.
        # self.yaw 0..2pi
        if self.yaw > 2*numpy.pi:
            self.yaw = 0.
        elif self.yaw < 0.:
            self.yaw += 2*numpy.pi

    def launch_sensors(self):
        """Launch and register the sensors used by the vehicle."""
        # Go through sensor list.
        for sensor in self.sensors:
            # Launch sensor node.
            sensor_name = sensor.partition(' ')[0]
            subpub_name = sensor_name.lower()+'_readings'
            args = str(self.vehicle_id)+' '+sensor
            node = Node('sml_world', 'sensor.py', namespace=self.namespace,
                        args=args, name=subpub_name)
            self.launcher.launch(node)
            # Register subscriptions for each of them.
            rospy.Subscriber(self.namespace + subpub_name,
                             getattr(msgs, sensor_name+'Readings'),
                             getattr(self, 'process_'+subpub_name))
        pass

    def handle_set_demand(self, req):
        """
        Handle set demand.

        @param req: I{(SetDemand)} Request of the service that sets demand.
        """
        self.bus_demand = req.bus_demand
        msg = "Demand #%i successfully set." % self.bus_demand
        return SetDemandResponse(True, msg)

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

    def handle_set_speed_kph(self, req):
        """
        Handle the set speed request.

        @param req: I{(SetSpeed)} Request of the service that sets the vehicles
                    cruising speed in kmh.
        """
        self.cruising_speed = req.speed / 3.6
        msg = "Speed of vehicle #%i successfully set." % self.vehicle_id
        return SetSpeedResponse(True, msg)

    def handle_set_loop(self, req):
        """
        Handle the set closed loop request.

        @param req: I{(SetLoop)} Request of the service that sets the vehicles
                    closed loop trajectory.
        """
        rospy.wait_for_service('/get_trajectory')
        try:
            get_traj = rospy.ServiceProxy('/get_trajectory', GetTrajectory)
            trajectory = get_traj(True, req.node_id, 0).trajectory
        except rospy.ServiceException, e:
            raise "Service call failed: %s" % e
        self.np_trajectory = to_numpy_trajectory(trajectory)
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
        self.np_trajectory = to_numpy_trajectory(trajectory)
        msg = ("Trajectory to destination of vehicle #%i " % self.vehicle_id +
               "successfully set.")
        return SetDestinationResponse(True, msg)

    def handle_toggle_simulation(self, req):
        """
        Handle the toggle simulation request.

        @param req: I{(SetBool)} Enable/Disable the vehicle simulation.
        """
        self.simulate = req.data
        if self.simulate:
            msg = "Vehicle #%i will now be simulated." % self.vehicle_id
        else:
            msg = "Vehicle #%i will stop to be simulated." % self.vehicle_id
        return SetBoolResponse(True, msg)


def to_numpy_trajectory(trajectory):
    """
    Transform Pose2D[] message to numpy array.

    @param trajectory: I{(Pose2D[])} ROS message of vehicle trajectory.

    @return: I{(numpy array)} Numpy array representation of the trajectory.
             [[x], [y], [yaw]]
    """
    tx = []
    ty = []
    tyaw = []
    for pose in trajectory:
        tx.append(pose.x)
        ty.append(pose.y)
        tyaw.append(pose.yaw)
    return numpy.asarray([tx, ty, tyaw])


class DummyVehicle(BaseVehicle):
    """Class for the dummy vehicle."""

    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=0.):
        """Initialize class DummyVehicle."""
        super(DummyVehicle, self).__init__(namespace, vehicle_id,
                                           simulation_rate, x, y, yaw, v)
        self.sensors = ['Radar 35 10']
        self.radar_readings = numpy.asarray([[], [], []])
        self.launch_sensors()

    def set_control_commands(self, ref_state):
        """
        Set the control commands, depending on the vehicles controler.

        @param ref_state: I{(numpy array)} Reference state [x, y, yaw] that
                          the vehicle tries to reach.
        """
        super(DummyVehicle, self).set_control_commands(ref_state)
        safety_distance = 10.
        full_stop_distance = 6.
        # Analyze radar readings.
        if not numpy.any(self.radar_readings[0, :]):
            return
        min_dist = numpy.min(self.radar_readings[0, :])
        # Set speed.
        if min_dist < full_stop_distance:
            desired_speed = 0.
        elif min_dist < safety_distance:
            desired_speed = self.cruising_speed * min_dist / safety_distance
        else:
            desired_speed = self.cruising_speed
        self.commands['speed'] = desired_speed

    def process_radar_readings(self, rr):
        """
        Put all sensor readings into a numpy array.

        @param rr: I{(RadarReadings)} Radar readings message that needs to
                   be put into the class variable radar_readings.
        """
        # Write sensor readings in an ndarray
        self.radar_readings = numpy.asarray([[], [], []])
        for r in rr.registered_vehicles:
            self.radar_readings = numpy.concatenate(
                                    (self.radar_readings,
                                     [[r.rho], [r.theta], [r.yaw]]),
                                    axis=1)


class Bus(BaseVehicle):
    """Class for the automated bus."""

    def __init__(self, namespace, vehicle_id, simulation_rate,
                 x=0., y=0., yaw=0., v=0.):
        """Initialize class Bus."""
        super(Bus, self).__init__(namespace, vehicle_id,
                                  simulation_rate, x, y, yaw, v)
        self.sensors = ['Radar 35 10']
        self.radar_readings = numpy.asarray([[], [], []])
        self.launch_sensors()

    def set_control_commands(self, ref_state):
        """
        Set the control commands, depending on the vehicle's controller.

        @param ref_state: I{(numpy array)} Reference state [x, y, yaw] that
                          the vehicle tries to reach.
        """
        super(Bus, self).set_control_commands(ref_state)
        safety_distance = 15.
        full_stop_distance = 6.
        # Analyze radar readings.
        if not numpy.any(self.radar_readings[0, :]):
            return
        min_dist = numpy.min(self.radar_readings[0, :])
        # Set speed.
        if min_dist < full_stop_distance:
            desired_speed = 0.
        elif min_dist < safety_distance:
            desired_speed = self.cruising_speed * min_dist / safety_distance
        else:
            desired_speed = self.cruising_speed
        self.commands['speed'] = desired_speed

    def process_radar_readings(self, rr):
        """
        Put all sensor readings into a numpy array.

        @param rr: I{(RadarReadings)} Radar readings message that needs to
                   be put into the class variable radar_readings.
        """
        # Write sensor readings in an ndarray
        self.radar_readings = numpy.asarray([[], [], []])
        for r in rr.registered_vehicles:
            self.radar_readings = numpy.concatenate(
                                    (self.radar_readings,
                                     [[r.rho], [r.theta], [r.yaw]]),
                                    axis=1)
