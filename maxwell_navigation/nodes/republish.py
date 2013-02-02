#!/usr/bin/env python

import roslib; roslib.load_manifest('maxwell_navigation')
import rospy

from sensor_msgs.msg import LaserScan

class republish:

    def __init__(self):
        rospy.init_node("short_scan")
        rospy.Subscriber("base_scan", LaserScan, self.laserCb)
        self.scanPub = rospy.Publisher('scan', LaserScan)
        rospy.spin()

    def laserCb(self, msg):
        n_ranges = list()
        for i in range(len(msg.ranges)):
            if msg.ranges[i] == 0.0:
                n_ranges.append(5.2)
            else:
                n_ranges.append(msg.ranges[i])
        msg.ranges = n_ranges
        self.scanPub.publish(msg)

if __name__ == "__main__":
    a = republish()

