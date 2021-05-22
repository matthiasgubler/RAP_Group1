#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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
from odometry import OdometryHandler
from marker import MarkerDetection
from movebase import MoveBase
#from moveit_msgs.msg import SaveMap

class Robot:
    def __init__(self):
        rospy.loginfo("Initializing Robot")
        self.detected_markers = set()
        self.TERMINAL_MARKER_ID = 17
        self.interrupt = False
        self.saved_position = ""

        rospy.loginfo("Initializing Arm and Gripper")
        ## XL Summit
        self.gripper = Gripper()
        self.walker = Walker()
        self.odometry = OdometryHandler()
        #self.actionserver = MoveAction("move_base")
        self.movebase = MoveBase()
        self.marker = MarkerDetection(self.marker_detected_callback)

        rospy.loginfo("Initializing TF2")
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(10.0)

    def save_position(self):
        rospy.loginfo("Saving Position")
        self.saved_position = self.odometry.current_pose
        rospy.loginfo("Saved Pose is %s", self.saved_position)

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
        self.save_position()
        
        # Doesn't Work --> Just to play around (Line 64 - 74)
        transform = self.tf_buffer.lookup_transform('summit_xl_odom', data.header.frame_id, rospy.Time(0))
        pose_transformed = tf2_geometry_msgs.do_transform_pose(data, transform)
        rospy.loginfo("Marker Pose: %s", data.pose)
        rospy.loginfo("Current Pose: %s", self.saved_position)
        rospy.loginfo("Transformed Pose: %s", pose_transformed)
        rospy.loginfo("Transform: %s", transform)
        
        # TODO Approach Stuff
        pose_transformed.pose.orientation.z = self.saved_position.orientation.z
        pose_transformed.pose.orientation.w = self.saved_position.orientation.w
        self.movebase.set_goal(pose_transformed.pose)
        rospy.sleep(1)

        # TODO Grasping Logic
        rospy.loginfo("Open Gripper")
        self.gripper.gripper_client_open()
        rospy.sleep(1)
        #set arm goal
        #/summit_xl/arm_pos_based_pos_traj_controller/follow_joint_trajectory/goal
        #Type: control_msgs/FollowJointTrajectoryActionGoal
        rospy.loginfo("Close Gripper")
        self.gripper.gripper_client_close()
        rospy.sleep(1)
        rospy.loginfo("Open Gripper")
        self.gripper.gripper_client_open()
        rospy.sleep(1)

        # TODO Return to last position
        rospy.loginfo("Move to saved Position")
        self.movebase.set_goal(self.saved_position)
        
        rospy.sleep(1)
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
        # TODO Todo save slam map
        # Was done in MAP Lab with (cli): rosrun map_server map_saver -f ~/map
        #/summit_xl/move_group/save_map
        #Node: /summit_xl/move_group
        #Type: moveit_msgs/SaveMap
        #Args: filename
        #self.map_saver = rospy.Publisher('/summit_xl/move_group/save_map', SaveMap, queue_size=1)
        rospy.sleep(1)
        #self.map_saver.publish("saved_map.yaml")

    def return_to_saved_position(self):
        rospy.loginfo("Returning to saved position: %s", self.saved_position)
        self.stop()
        # TODO Todo return to startposition

    def end_task(self):
        rospy.loginfo("Ending Task")
        self.walker.stop()
        self.save_map()
        self.return_to_saved_position()
        rospy.signal_shutdown("task ended successfully")

