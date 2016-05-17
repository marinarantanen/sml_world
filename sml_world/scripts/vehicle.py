#!/usr/bin/env python
"""
Vehicle node of the SML-World.

Created on Mar 3, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import sys

import rospy

<<<<<<< HEAD
from sml_modules.vehicle_models import BaseVehicle, DummyVehicle, Bus
=======
from sml_modules.vehicle_models import BaseVehicle, DummyVehicle, WifiVehicle
from sml_modules.bus_vehicle_model import BusVehicle
>>>>>>> eeae3cb5dc6d5f0c77dec60e8b06225f1d25d921


def vehicle(vehicle_id, vehicle_class, x=0., y=0., yaw=0., speed_in_ms=0.):
    """
    Initialize ROS-node 'vehicle' and register subs, pubs and services.

    @param vehicle_id: I{(int)} ID of the vehicle that is created.
    @param vehicle_class: I{(str)} Name of the vehicle class that should be
                          created.
    @param x: I{(float)} x-coordinate at which the vehicle should be spawned.
    @param y: I{(float)} y-coordinate at which the vehicle should be spawned.
    @param yaw: I{(float)} Initial yaw with which the vehicle should be
                spawned.
    @param speed_in_ms: I{(float)} Initial speed whith which the vehicle
                        should be spawned.
    """
    rospy.init_node('vehicle', log_level=rospy.WARN)
    if vehicle_class == BaseVehicle.__name__:
        BaseVehicle(rospy.get_namespace(), vehicle_id, 20,
                    x, y, yaw, speed_in_ms)
    elif vehicle_class == DummyVehicle.__name__:
        DummyVehicle(rospy.get_namespace(), vehicle_id, 20,
                     x, y, yaw, speed_in_ms)
<<<<<<< HEAD
    elif vehicle_class == Bus.__name__:
        Bus(rospy.get_namespace(), vehicle_id, 20, x, y, yaw, speed_in_ms)
=======
    elif vehicle_class == WifiVehicle.__name__:
        WifiVehicle(rospy.get_namespace(), vehicle_id, 20,
                    x, y, yaw, speed_in_ms)
    elif vehicle_class == BusVehicle.__name__:
        BusVehicle(rospy.get_namespace(), vehicle_id, 20,
                    x, y, yaw, speed_in_ms)
>>>>>>> eeae3cb5dc6d5f0c77dec60e8b06225f1d25d921
    else:
        raise Exception("ERROR: Unknown vehicle class '%s'." % vehicle_class)
    rospy.spin()


if __name__ == '__main__':
    # Filter sys.argv to remove automatically added arguments
    sys.argv = [arg for arg in sys.argv if str(arg).find(':=') < 0]
    args = {}
    if len(sys.argv) > 2:
        args['vehicle_id'] = int(sys.argv[1])
        args['vehicle_class'] = str(sys.argv[2])
    else:
        msg = ("Usage: rosrun sml_world vehicle.py " +
               "<vehicle_id> <vehicle_class> <x> <y> <yaw> <speed_in_ms>")
        raise Exception(msg)
    if len(sys.argv) > 3:
        args['x'] = float(sys.argv[3])
    if len(sys.argv) > 4:
        args['y'] = float(sys.argv[4])
    if len(sys.argv) > 5:
        args['yaw'] = float(sys.argv[5])
    if len(sys.argv) > 6:
        args['speed_in_ms'] = float(sys.argv[6])
    vehicle(**args)
