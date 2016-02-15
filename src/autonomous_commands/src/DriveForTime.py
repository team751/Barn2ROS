#! /usr/bin/env python

import roslib
import rospy

import actionlib
import time

from geometry_msgs.msg import Twist
import autonomous_commands.msg

publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

class DriveForwardForTime(object):
  # create messages that are used to publish feedback/result
  _feedback = autonomous_commands.msg.DriveForTimeFeedback()
  _result   = autonomous_commands.msg.DriveForTimeResult()
  _startTime = None

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, autonomous_commands.msg.DriveForTimeAction, execute_cb=self.execute, auto_start = False)
    self._as.start()
    
  def execute(self, goal):
    self._startTime = int(round(time.time() * 1000))

    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown() and int(round(time.time() * 1000)) - self._startTime < goal.time:
      self._feedback.time = int(round(time.time() * 1000)) - self._startTime
      publisher.publish(goal.cmd)
      self._as.publish_feedback(self._feedback)
      rate.sleep()

    # Stop on completion
    publisher.publish(Twist())

    self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('drive_forward_for_time')
  DriveForwardForTime(rospy.get_name())
  rospy.spin()
