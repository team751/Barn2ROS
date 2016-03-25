#! /usr/bin/env python

import roslib
import rospy

import actionlib
import time

import tf

import math

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point
import autonomous_commands.msg

publisher = rospy.Publisher('/robot/ddc/cmd_vel', Twist, queue_size=10)
listener = None


class AlignWithBall(object):
  _currentX = 0.0
  _currentY = 0.0
  _currentAngle = 0.0
  _dataReceived = False

  # create messages that are used to publish feedback/result
  _feedback = autonomous_commands.msg.AlignWithBallFeedback()
  _result   = autonomous_commands.msg.AlignWithBallResult()

  def __init__(self, name):
    rospy.Subscriber("circle_detect", Point, self.ballPointReceived);
    rospy.Subscriber("/angle", Float32, self.robotAngleReceived);

    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, autonomous_commands.msg.AlignWithBallAction, execute_cb=self.execute, auto_start = False)
    self._as.start()
    
  def diff(robotTheta, ballTheta):
    return ((robotTheta - ballTheta + math.pi) % (2 * math.pi)) - math.pi
  
  def getRobotTheta():
    return self._currentAngle
    
  def execute(self, goal):
    if (rospy.Time.now() - self._lastMessage > rospy.Duration(5)):
      return

    print  self._currentX
    rotation = math.atan(self._currentY / self._currentX)
    desiredRotation = 0.0
    x = self._currentX
    y = self._currentY
    goodCount = 0

    sign = rotation > 0

    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown() and not self._as.is_preempt_requested():
      try:
          actRot = getRobotTheta()

          if desiredRotation == 0:
            desiredRotation = (actRot + rotation) % math.pi

          twist = Twist()

          if abs((actRot - desiredRotation)) < .01:
            goodCount = goodCount + 1
          else:
            goodCount = 0

          if goodCount >= 20:
            break

          # offset = (actRot - desiredRotation)
          offset = diff(actRot, rotation)
          
          print "OFFSET: " + str(offset)

          if math.fabs(offset) * 0.1 > .1:
            twist.angular.z = .1
          else:
            twist.angular.z = offset * 0.1

          publisher.publish(twist)


          print actRot
          print "DESIRED ROTATION: " + str(desiredRotation)
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "ERR"
      self._as.publish_feedback(self._feedback)
      rate.sleep()

    targetY = 0.0

    while not rospy.is_shutdown() and not self._as.is_preempt_requested():
      now = rospy.Time.now()
      listener.waitForTransform("/xv11_front", "/odom", now, rospy.Duration(4.0))
      (trans,rot) = listener.lookupTransform('/xv11_front', '/odom', now)

      dist = math.sqrt(trans[0] * trans[0] + trans[1] * trans[1])

      if targetY == 0:
        targetY = dist + (math.sqrt(y*y+x*x)) - .15

      twist = Twist()

      if abs(targetY - dist) < .1:
        break

      speed = targetY - dist

      if speed > 1:
        speed = 1
      elif speed < -1:
        speed = -1

      twist.linear.x = speed

      publisher.publish(twist)

      self._as.publish_feedback(self._feedback)
      rate.sleep()

      print str(dist) + " : " + str(targetY)
    # Stop on completion
    publisher.publish(Twist())

    self._as.set_succeeded(self._result)

  def robotAngleReceived(self, data):
    self._currentAngle = data.data
    print self._currentAngle
    
  def ballPointReceived(self, data):
    self._currentX = data.x
    self._currentY = data.y
    self._lastMessage = rospy.Time.now()
    self._dataReceived = True

    # print data.x
    
      
if __name__ == '__main__':
  rospy.init_node('align_with_ball')
  listener = tf.TransformListener()
  AlignWithBall(rospy.get_name())
  rospy.spin()
