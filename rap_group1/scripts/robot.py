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
from marker import MarkerDetection

class Robot:
    def __init__(self):
        rospy.loginfo("Initializing Robot")
        self.detected_markers = set()
        self.TERMINAL_MARKER_ID = 17
        self.interrupt = False

        rospy.loginfo("Initializing Arm and Gripper")
        ## XL Summit
        self.gripper = Gripper()
        self.walker = Walker()
        self.marker = MarkerDetection(self.marker_detected_callback)

        rospy.loginfo("Initializing TF2")
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(10.0)

    def save_position(self):
        rospy.loginfo("Saving Position")
        #TODO Todo save slam position

    def start_spinning(self):
        if not self.interrupt:
            self.walker.spin()

    def stop(self):
        rospy.loginfo("Robot stop")
        self.interrupt = True
        self.walker.stop()

    def marker_detected(self, data):
        rospy.loginfo("Start marker detection")
        self.stop()
        rospy.sleep(1)
        #TODO Save last position
        #TODO Approach Stuff
        #TODO Grasping Logic
        #TODO Return to last position
        self.interrupt = False

    def marker_detected_callback(self, data):
        rospy.loginfo("Marker detected")
        if self.TERMINAL_MARKER_ID == data.id:
            rospy.loginfo("Terminal Marker [%s] detected, ending performance", data.id)
            self.end_task()
        else:
            rospy.loginfo("Marker [%s] detected", data.id)
            if data.id in self.detected_markers:
                rospy.loginfo("Marker already recoginzed -> nothing to do")
            else:
                rospy.loginfo("New marker")
                self.detected_markers.add(data.id)
                self.marker_detected(data)

    def save_map(self):
        rospy.loginfo("Saving map")
        #TODO Todo save slam map

    def return_to_startposition(self):
        rospy.loginfo("Returning to start position")
        #TODO Todo return to startposition

    def end_task(self):
        rospy.loginfo("Ending Task")
        self.walker.stop()
        self.save_map()
        self.return_to_startposition()
        rospy.signal_shutdown("task ended successfully")

