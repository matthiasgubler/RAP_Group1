#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

class MarkerDetection:
    def __init__(self, marker_detected_callback):
        rospy.loginfo("Initializing Marker")
        self.marker_subscriber = rospy.Subscriber('/visualization_marker', Marker, marker_detected_callback, queue_size=1)
