#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_python.geometry import rotate_pose_msg_by_euler_angles, translate_pose_msg
from geometry_msgs.msg import PoseStamped

class niryo_moveit:
  def __init__(self, group_name):
    self.group_name = group_name
    self.group = moveit_commander.MoveGroupCommander(self.group_name, robot_description="/robot_description", ns="")
    self.group.set_planner_id("SPARStwo")
    self.group.set_max_velocity_scaling_factor(0.5)
    self.group.set_goal_orientation_tolerance(0.01)
    self.group.set_planning_time(5)
    self.group.allow_replanning(True)
    self.planning = moveit_commander.PlanningSceneInterface("ground_link")
    rospy.sleep(1)

  def move_to(self, pose_goal):
    self.group.set_start_state_to_current_state()
    self.group.set_goal_tolerance(0.01)
    self.group.set_pose_target(pose_goal)
    print("planning ...")
    plan = self.group.plan()
    rospy.sleep(1)
    cont_plan_move = 0
    while ((len(plan.joint_trajectory.points) == 0) and (cont_plan_move < 10)):
        plan = self.group.plan()
        rospy.sleep(1)
        cont_plan_move += 1
    if (len(plan.joint_trajectory.points) != 0):
        inp = raw_input("Have a look at the planned motion. Do you want to proceed? y/n: ")
        if (inp == 'y'):
          result = self.group.execute(plan, wait=True)
          rospy.sleep(1)

