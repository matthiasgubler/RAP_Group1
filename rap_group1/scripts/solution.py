#!/usr/bin/env python
import rospy
from robot import Robot

class ChallengeTask:

    def __init__(self):
        rospy.init_node('rap_group1', anonymous=True)
        rospy.loginfo("Initializing ChallengeTask")
        self.performTask = True
        self.robot = Robot()

    def start(self):

        rospy.loginfo("Starting ChallengeTask")

        rospy.on_shutdown(self.cleanup)

        self.robot.save_position()
        self.robot.start_spinning()
        while not rospy.is_shutdown():
            self.robot.start_spinning()

        rospy.spin()

    def cleanup(self):
        self.robot.stop()


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        task = ChallengeTask()
        task.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("Aruco_tag test finished.")
