#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Walker:
    def __init__(self):
        rospy.loginfo("Initializing Walker")
        self.velocityPublisher = rospy.Publisher('/summit_xl/cmd_vel', Twist, queue_size=1)

    def stop(self):
        rospy.loginfo("Stopping Movements")
        stop_movements = Twist()
        self.gripperPublisher.publish(stop_movements)

    def spin(self):
        rospy.loginfo("Start spinning")
        spin = Twist()
        spin.z = 0.5
        self.gripperPublisher.publish(spin)