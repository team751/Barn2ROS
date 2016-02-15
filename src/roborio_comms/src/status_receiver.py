#!/usr/bin/env python
import rospy
import socket
import threading
from threading import Thread
from threading import Event
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

ipAddress = rospy.get_param('~network/ip', "0.0.0.0")
port = rospy.get_param('~network/port', 6000)
imuTopic = rospy.get_param('~imu_topic', "/imu")
odomFrame = rospy.get_param('~odom_frame', "/odom")
lwheelTopic = rospy.get_param('~lwheel_topic', "/lwheel")
rwheelTopic = rospy.get_param('~rhweel_topic', "/rwheel")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ipAddress, port))
    
def listener():
    rospy.init_node('status_receiver', anonymous=True)
    imuPub = rospy.Publisher(imuTopic, Imu, queue_size=10, latch=True)
    lWheelPub = rospy.Publisher(lwheelTopic, Int16, queue_size=10, latch=True)
    rWheelPub = rospy.Publisher(rwheelTopic, Int16, queue_size=10, latch=True)
    
    while True:
        data, addr = sock.recvfrom(1024)
        arr = data[1:-1].split(",")
        
        # encoder
        lWheelMsg = Int16 (int(arr[0]))
        rWheelMsg = Int16 (int(arr[1]))
        
        # IMU
        imuMsg = Imu()
        
        imuMsg.header.stamp = rospy.Time.now()
        imuMsg.header.frame_id = odomFrame
                
        imuMsg.angular_velocity.x = float(arr[2])
        imuMsg.angular_velocity.y = float(arr[3])
        imuMsg.angular_velocity.z = float(arr[4])
        
        imuMsg.linear_acceleration.x = float(arr[5])
        imuMsg.linear_acceleration.y = float(arr[6])
        imuMsg.linear_acceleration.z = float(arr[7])
        
        imuMsg.orientation_covariance[0] = -1
        
        imuMsg.angular_velocity_covariance = [-0.01,0, 0,
                                              0, -0.01, 0,
                                              0, 0, -0.01]
                                              
        imuMsg.linear_acceleration_covariance = [-0.01,0, 0,
                                                 0, -0.01, 0,
                                                 0, 0, -0.01]
        
        # publish messages
        lWheelPub.publish(lWheelMsg)
        rWheelPub.publish(rWheelMsg)
        imuPub.publish(imuMsg)

if __name__ == '__main__':
    listener()
