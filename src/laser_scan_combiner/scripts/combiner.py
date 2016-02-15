#!/usr/bin/env python
import rospy
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from laser_geometry import laser_geometry

class GoToPose():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)

        #what to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        
        #tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        #allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))


        #we'll send a goal to the robot to tell it to move to a pose that's near the docking station
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        if (int(sys.argv[1]) == 0):
            goal.target_pose.pose = Pose(Point(0, -1, 0.000), Quaternion(0.000, 0.000, 0.698034415549, 0.716064211303))
        else:
            goal.target_pose.pose = Pose(Point(-3.5 + (int(sys.argv[1]) - 1) * (5.6/4), 1.5, 0.000), Quaternion(0.000, 0.000, 0.698034415549, 0.716064211303))

        #customize the following Point() values so they are appropriate for your location
        

        self.move_base.wait_for_server(rospy.Duration(5))

        rospy.loginfo("send goal")

        #start moving
        self.move_base.send_goal(goal)

        #allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 


        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach the desired pose")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, reached the desired pose")



    def shutdown(self):
        rospy.loginfo("Stop")


if __name__ == '__main__':
    try:
        GoToPose()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
