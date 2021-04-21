#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Marker:
    def __init__(self, marker_detected_callback):
        rospy.loginfo("Initializing Walker")
        self.marker_subscriber = rospy.Subscriber('/visualization_marker', Marker, marker_detected_callback, queue_size=1)

    def stop(self):
        rospy.loginfo("Stopping Movements")
        stop_movements = Twist()
        self.gripperPublisher.publish(stop_movements)

    def spin(self):
        rospy.loginfo("Start spinning")
        spin = Twist()
        spin.z = 0.5
        self.gripperPublisher.publish(spin)