#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist

current_millis = lambda: int(round(time.time() * 1000))

x = 0
y = 0
quickturn = False

oldWheel = 0
quickStopAccumulator = 0
neg_inertia_accumulator = 0

wheelNonLinearity = .1

lastMsg = current_millis()

def joystickUpdated(data):
  global x, y, lastMsg
  x = data.linear.x
  y = data.linear.y
  lastMsg = current_millis()
  
def calculateMotorOutput(throttle, wheel):
  global oldWheel, neg_inertia_accumulator, quickStopAccumulator
  
  # Calculate the change in steering value from the last loop to this one
  # Positive is a change toward the right
  neg_inertia = wheel - oldWheel
  oldWheel = wheel
  
  # Scale down front/back by 0.7
  throttle *= 0.7;
  
  # Apply a sin function that's scaled to make it feel better
  # Each one of these calls can be visualized with the graph of the
  # function, available at
  # http://www.wolframalpha.com/input/?i=sin%28pi%2F2+*+0.8+*+x%29+%2F+sin%28pi%2F2+*+0.8%29
  wheel = math.sin((math.pi / 2.0) * wheelNonLinearity * wheel) / math.sin((math.pi / 2.0) * wheelNonLinearity);
  wheel = math.sin((math.pi / 2.0) * wheelNonLinearity * wheel) / math.sin((math.pi / 2.0) * wheelNonLinearity);
  wheel = math.sin((math.pi / 2.0) * wheelNonLinearity * wheel) / math.sin((math.pi / 2.0) * wheelNonLinearity);
  
  overpower = 0
  sensitivity = 0
  angular_power = 0
  linear_power = 0
  neg_inertia_scalar = 0
  
  # If the current rotate command and the change in rotate command have the same sign
  if (wheel * neg_inertia > 0):
    neg_inertia_scalar = 2.5
  else:
    if (abs(wheel) > 0.65):
      neg_inertia_scalar = 5
    else:
      neg_inertia_scalar = 3
  
  sensitivity = 1.1
  
  if abs(throttle) > .1:
    sensitivity = 1 - (1 - sensitivity) / abs(throttle)
    
  neg_inertia_power = neg_inertia * neg_inertia_scalar
  neg_inertia_accumulator += neg_inertia_power
  wheel = wheel + neg_inertia_accumulator
  if neg_inertia_accumulator > 1:
    neg_inertia_accumulator -= 1;
  elif (neg_inertia_accumulator < -1):
    neg_inertia_accumulator += 1
  else:
    neg_inertia_accumulator = 0
    
  linear_power = throttle
  
  if quickturn:
    # Scale down the rotation
    wheel *= 0.7
    
    if abs(linear_power) < 0.2:
      alpha = .1
      quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limit(wheel) * 5
    overPower = 1.0
    angular_power = wheel
  else:
    overPower = 0.0
    angular_power = abs(throttle) * wheel * sensitivity - quickStopAccumulator
    
    if quickStopAccumulator > 1:
      quickStopAccumulator -= 1
    elif quickStopAccumulator < -1:
      quickStopAccumulator += 1
    else:
      quickStopAccumulator = 0.0
  
  twist = Twist()
  twist.linear.x = linear_power * 2
  twist.angular.z = angular_power

  return twist

def cheesydrive():
  global x, y
  rospy.init_node('talker', anonymous=True)
  
  velPub = rospy.Publisher(rospy.get_param('vel_topic', '/husky_velocity_controller/cmd_vel'), Twist, queue_size=10)
  
  joystickSub = rospy.Subscriber(rospy.get_param('joystick_topic', "/driver/drivestick"), Twist, joystickUpdated)
    
  rate = rospy.Rate(10) # 10hz
  
  while not rospy.is_shutdown():
    if current_millis() - lastMsg > 500:
      x = 0
      y = 0
    
    twist = calculateMotorOutput(y, x)
    velPub.publish(twist)
  
    rate.sleep()

if __name__ == '__main__':
    try:
        cheesydrive()
    except rospy.ROSInterruptException:
        pass
