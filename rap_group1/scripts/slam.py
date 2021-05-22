#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class SlamHandler:
    def __init__(self):
        rospy.loginfo("Initializing Slam")
        self.current_pose = ""
        self.scan_subscriber = rospy.Subscriber('/summit_xl/scan_combined', LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        self.current_pose = msg.pose.pose
