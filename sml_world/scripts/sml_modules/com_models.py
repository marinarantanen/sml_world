"""
Module containing all available communication classes.

Created on Mar 8, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""
import math

import rospy
import sml_world.msg as msgs
import sml_world.srv as srvs


class BaseCom(object):
    """Base class for communication."""

    def __init__(self, vehicle_id):
        """Initialize class BaseCom."""
        self.vehicle_id = int(vehicle_id)
        self.class_name = self.__class__.__name__
        self.pos = (None, None)
        # Register subscribers, publisher and service
        pubsub_msg = getattr(msgs, self.class_name+'Com')
        glob_pubsub_name = '/all_'+self.class_name.lower()+'_com'
        cb_func_name = 'filter_all_'+self.class_name.lower()+'_com'
        rospy.Subscriber(glob_pubsub_name, pubsub_msg,
                         getattr(self, cb_func_name))
        self.pub_glob = rospy.Publisher(glob_pubsub_name, pubsub_msg,
                                        queue_size=10)
        rospy.Subscriber('/world_state', msgs.WorldState,
                         self.update_vehicle_state)
        send_service_name = 'send_'+self.class_name.lower()+'_com'
        rospy.Service(send_service_name,
                      getattr(srvs, 'Send'+self.class_name+'Com'),
                      getattr(self, 'handle_'+send_service_name))

    def update_vehicle_state(self, ws):
        """Callback function for topic 'world-state'."""
        self.pos = (None, None)
        for vs in ws.vehicle_states:
            if vs.vehicle_id == self.vehicle_id:
                self.pos = (vs.x, vs.y)


class Wifi(BaseCom):
    """Wifi communication class."""

    def __init__(self, vehicle_id, name, com_range):
        """Initialize Wifi communication class."""
        super(Wifi, self).__init__(vehicle_id)
        self.name = name
        self.com_range = float(com_range)
        self.pub_com = rospy.Publisher(name.lower()+'_com', msgs.WifiCom,
                                       queue_size=10)

    def filter_all_wifi_com(self, wc):
        """Filter Wifi communication to what the vehicle can receive."""
        if wc.vehicle_id == self.vehicle_id:
            return
        if (not self.pos[0] or not self.pos[1]):
            return
        dx = self.pos[0] - wc.x
        dy = self.pos[1] - wc.y
        dist = math.hypot(dx, dy)
        if dist <= self.com_range:
            self.pub_com.publish(wc)

    def handle_send_wifi_com(self, req):
        """Handle sending msg over wifi request."""
        self.pub_glob.publish(self.vehicle_id, self.pos[0], self.pos[1],
                              req.message)
        return srvs.SendWifiComResponse(True, "Message sucessfully sent out.")
