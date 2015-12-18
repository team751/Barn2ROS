#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

current_millis = lambda: int(round(time.time() * 1000))

wheelRadius = 3
length = 23

lastMsg = current_millis()

lVelPub = rospy.Publisher("/lvel", Float32, queue_size=10)
rVelPub = rospy.Publisher("/rvel", Float32, queue_size=10)

def callback(data):
    global lastMsg
    lastMsg = current_millis()

    left = (data.linear.x - (length / 2.0) * data.angular.z) / wheelRadius
    right = (data.linear.x + (length / 2.0) * data.angular.z) / wheelRadius
    lVelPub.publish(left)
    rVelPub.publish(right)

def listener():
    rospy.init_node('drivetrain_converter', anonymous=True)

    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist, callback)

    rate = rospy.Rate(3) # 10hz
    while not rospy.is_shutdown():
        if current_millis() - lastMsg > 500:
	    print "Bad" + str(current_millis() - lastMsg)
	    lVelPub.publish(0)
            rVelPub.publish(0)
        rate.sleep()  
 
if __name__ == '__main__':
    listener()
