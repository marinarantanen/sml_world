#!/usr/bin/env python
# license removed for brevity
"""
Simulator of the SML-World.

For now all this node does is publishing the states of one vehicle.

Created on Feb 23, 2016

@author: U{David Spulak<spulak@kth.se>}
@organization: KTH
"""
import rospy
from std_msgs.msg import String

def pub_state(states):
    pub = rospy.Publisher('world_state', String, queue_size=10)
    rospy.init_node('simulator', anonymous=True)
    rate = rospy.Rate(60) # 60hz
    while not rospy.is_shutdown():
        for line in lines:
            pub.publish(line)
            rate.sleep()

if __name__ == '__main__':
    try:
        lines = []
        with open('/home/david/vehicle_state2.txt', 'r') as f:
            for line in f:
                lines.append(line)
        pub_state(lines)

    except rospy.ROSInterruptException:
        pass
