#!/usr/bin/env python
import rospy
import socket
import threading
from threading import Thread
from threading import Event
from std_msgs.msg import String

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 0))
    
def listener():
    rospy.init_node('status_receiver', anonymous=True)
    pub = rospy.Publisher('/robot/status', String, queue_size=10)
    
    while True:
        data, addr = sock.recvfrom(1024)
        pub.publish(data)

if __name__ == '__main__':
    listener()
