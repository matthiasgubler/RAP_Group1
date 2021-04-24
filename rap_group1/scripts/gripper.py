#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class Gripper:
    def __init__(self):
        rospy.loginfo("Initializing Gripper")
        self.OPEN = 0.0
        self.CLOSE = 0.65
        self.gripper_publisher = rospy.Publisher('/summit_xl/gripper_left_controller/command', Float64, queue_size=1)

    def open(self):
        rospy.loginfo("Opening Gripper")
        self.gripper_publisher.publish(self.OPEN)

    def close(self):
        rospy.loginfo("Closing Gripper")
        self.gripper_publisher.publish(self.CLOSE)