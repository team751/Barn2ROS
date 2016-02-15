#!/usr/bin/env python
import rospy

import actionlib
import time

import actionlib
import autonomous_commands.msg
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

class DriveToDefense():
    # create messages that are used to publish feedback/result
    _move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, autonomous_commands.msg.DriveToDefenseAction, execute_cb=self.execute, auto_start = False)
        self._as.start()

    def execute(self, goal):

        self._move_base.wait_for_server(rospy.Duration(5))

        movebase_goal = MoveBaseGoal()
        movebase_goal.target_pose.header.frame_id = 'map'
        movebase_goal.target_pose.header.stamp = rospy.Time.now()

        movebase_goal.target_pose.pose = Pose(Point(-3.5 + (goal.defense - 1) * (5.4/4), 1.5, 0.000), Quaternion(0.000, 0.000, 0.7071, 0.7071))        

        self._move_base.wait_for_server(rospy.Duration(5))

        self._move_base.send_goal(movebase_goal)

        success = self._move_base.wait_for_result(rospy.Duration(60)) 

        if not success:
            self._move_base.cancel_goal()
            self._as.set_failed()
        else:
            state = self._move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                self._as.set_succeeded()

if __name__ == '__main__':
  rospy.init_node('drive_to_defense')
  DriveToDefense(rospy.get_name())
  rospy.spin()