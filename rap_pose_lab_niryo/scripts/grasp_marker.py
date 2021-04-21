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


class Tag_nav():
    '''Class for reacting to aruco_tags.'''

    def __init__(self):
        rospy.init_node('lab5_nav', anonymous=True)
        rospy.loginfo("Initialized Lab5 Node")
        self.move_arm = niryo_moveit("arm")
        self.gripper_pub1 = rospy.Publisher('/gripper_left_controller/command', Float64, queue_size=1)
        self.gripper_pub2 = rospy.Publisher('/gripper_right_controller/command', Float64, queue_size=1)
        # ADD YOUR CODE HERE (publishers, subscribers, tf_buffers...)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(10.0)

        self.grasp_from_marker = rospy.Publisher('/grasp_pose_from_marker', PoseStamped, queue_size=1)
        self.marker_sub = rospy.Subscriber('/visualization_marker', Marker, self.callback_marker)

    def callback_marker(self, data):

        rospy.loginfo("Marker msg was: \n%s", data)
        # ADD YOUR CODE HERE
        transform = self.tf_buffer.lookup_transform('base_link', data.header.frame_id, rospy.Time(0))

        rospy.loginfo("Transform: \n%s", transform)
        pose_transformed = tf2_geometry_msgs.do_transform_pose(data, transform)

        rospy.loginfo("pose_transformed: \n%s", pose_transformed)
        grasp_pose = rotate_pose_msg_by_euler_angles(pose_transformed.pose, math.pi, math.pi, math.pi / 2)

        rospy.loginfo("grasp_pose: \n%s", grasp_pose)
        grasp_pose.position.z = grasp_pose.position.z + 0.07
        grasp_pose.position.x = grasp_pose.position.x - 0.01

        # log and publish
        rospy.loginfo("Transformed pose is: \n %s", grasp_pose)
        self.publish_grasp_pose(grasp_pose)
        # open gripper
        self.gripper_client(-0.5)
        # send pose goal to arm
        self.move_arm.move_to(grasp_pose)
        # close gripper
        self.gripper_client(0.1)



        # ADD YOUR CODE HERE to MOVE arm somewhere else and drop object

        terminal_pose = Pose()
        terminal_pose.position.x = -0.2
        terminal_pose.position.y = 0
        terminal_pose.position.z = 0.7
        terminal_pose.orientation.x = 0.001
        terminal_pose.orientation.y = 0.705
        terminal_pose.orientation.z = 0.000
        terminal_pose.orientation.w = 0.709

        self.move_arm.move_to(terminal_pose)
        self.gripper_client(-0.5)

    def publish_grasp_pose(self, grasp_pose):
        # ADD YOUR CODE HERE
        stamped_pose = PoseStamped()
        stamped_pose.pose = grasp_pose
        stamped_pose.header.frame_id = 'base_link'

        rospy.loginfo("publish_grasp_pose called with: %s", stamped_pose)
        self.grasp_from_marker.publish(stamped_pose)

    def gripper_client(self, value):
        rospy.loginfo("gripper_client called with: %s", value)
        rospy.sleep(0.5)
        self.gripper_pub1.publish(-value)
        self.gripper_pub2.publish(value)


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        ar_nav = Tag_nav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Aruco_tag test finished.")
