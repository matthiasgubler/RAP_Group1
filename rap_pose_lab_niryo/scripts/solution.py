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

        self.robot.savePosition() #????
        self.robot.startSpinning()

        while self.robot.perform_task:
            self.robot.markerDetection()

        self.robot.saveMap()
        self.robot.returnToStartPosition()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        task = ChallengeTask()
        task.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Aruco_tag test finished.")
