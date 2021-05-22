#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdometryHandler:
    def __init__(self):
        rospy.loginfo("Initializing Odometry")
        self.current_pose = ""
        #self.odom_subscriber = rospy.Subscriber('/summit_xl/robotnik_base_control/odom', Odometry, self.odometry_callback)
        self.odom_subscriber = rospy.Subscriber('/summit_xl/amcl_pose', PoseWithCovarianceStamped, self.odometry_callback)

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose
