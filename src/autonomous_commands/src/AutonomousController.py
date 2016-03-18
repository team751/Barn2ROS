#!/usr/bin/env python
import rospy
import sys


import actionlib

from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from autonomous_commands.msg import *

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

class AutonomousController():
    def __init__(self):
        rospy.init_node('AutonomousController', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        self.drive_to_defense = actionlib.SimpleActionClient("drive_to_defense", DriveToDefenseAction)
        self.drive_for_time = actionlib.SimpleActionClient("drive_for_time", DriveForTimeAction)
        self.drive_to_low_goal = actionlib.SimpleActionClient("drive_to_low_goal", DriveToLowGoalAction)

        self.drive_to_defense.wait_for_server(rospy.Duration(5))

        dtgoal = DriveToDefenseGoal()
        dtgoal.defense = 5

        rospy.loginfo("Drive to defense")

        self.drive_to_defense.send_goal(dtgoal)

        success = self.drive_to_defense.wait_for_result(rospy.Duration(60)) 

        if success:
            rospy.loginfo("Drive forwards")
            self.drive_for_time.wait_for_server(rospy.Duration(5))

            dfgoal = DriveForTimeGoal()
            dfgoal.cmd = Twist()
            dfgoal.cmd.linear.x = 1.0
            dfgoal.time = 3000
            self.drive_for_time.send_goal(dfgoal)
            success = self.drive_for_time.wait_for_result(rospy.Duration(60))
            
            self.drive_to_low_goal.wait_for_server(rospy.Duration(5))

            rospy.loginfo("Drive to low goal")
            dlggoal = DriveToLowGoalGoal()
            dlggoal.goal = 1
            self.drive_to_low_goal.send_goal(dlggoal)
            success = self.drive_to_low_goal.wait_for_result(rospy.Duration(60))

            if success:
                rospy.loginfo("Drive forwards into low goal")
                dfgoal.cmd.linear.x = 3.0
                dfgoal.time = 1000
                self.drive_for_time.send_goal(dfgoal)
        else:
          rospy.logerr("Drive to goal failed")

    def shutdown(self):
        rospy.loginfo("Stop")


if __name__ == '__main__':
    try:
        AutonomousController()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
