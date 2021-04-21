#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
from moveit_python.geometry import rotate_pose_msg_by_euler_angles, translate_pose_msg
import math
from std_msgs.msg import Float64
from set_niryo_moveit import niryo_moveit
from ar_track_alvar_msgs.msg import AlvarMarkers
from moveit_python.geometry import rotate_pose_msg_by_euler_angles, translate_pose_msg
from gripper import Gripper
from walker import Walker
from marker import Marker

class Robot:
    def __init__(self):
        rospy.loginfo("Initializing Robot")
        self.perform_task = True
        self.detected_markers = {}
        self.TERMINAL_MARKER_ID = 17

        rospy.loginfo("Initializing Arm and Gripper")
        ## XL Summit
        self.gripper = Gripper()
        self.walker = Walker()
        self.marker = Marker(self.marker_detected_callback)

        rospy.loginfo("Initializing TF2")
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(10.0)

        self.marker_sub = rospy.Subscriber('/visualization_marker', Marker, self.callback_marker)


    def savePosition(self):
        rospy.loginfo("Saving Position")

    def startSpinning(self):
        rospy.loginfo("Start spinning")
        self.walker.spin()

    def markerDetection(self):
        rospy.loginfo("Start marker detection")

    def marker_detected_callback(self, data):
        rospy.loginfo("Marker detected")
        if self.TERMINAL_MARKER_ID == data.id:
            rospy.loginfo("Terminal Marker [%s] detected, ending performance", data.id)
            self.perform_task = False
        else:
            rospy.loginfo("Marker [%s] detected", data.id)
            if data.id in self.detected_markers:
                rospy.loginfo("Marker already recoginzed -> nothing to do")
            else:
                rospy.loginfo("New marker")
                self.detected_markers.add(data.id)
                ##TODO Approach Marker

    def saveMap(self):
        rospy.loginfo("Saving map")

    def returnToStartPosition(self):
        rospy.loginfo("Returning to start position")

