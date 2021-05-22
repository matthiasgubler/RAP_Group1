#!/usr/bin/env python
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveBase:
    def __init__(self):
        rospy.loginfo("Initializing MoveBase")
        #self.move_base_goal_publisher = rospy.Publisher('/summit_xl/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.move_base = actionlib.SimpleActionClient("summit_xl/move_base", MoveBaseAction)
        self.move_base.wait_for_server()

    def set_goal(self, pose):
        rospy.loginfo("Setting move_base goal to pose: %s", pose)

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "summit_xl_map"
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        goal.target_pose.pose = pose
        rospy.loginfo("goal: %s", goal)
        #self.move_base_goal_publisher.publish(goal)
        self.move_base.send_goal(goal)
        wait = self.move_base.wait_for_result()
        if not wait:
            rospy.loginfo("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.move_base.get_result()
