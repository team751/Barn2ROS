#!/usr/bin/env python
import rospy
import socket
import threading
from threading import Thread
from threading import Event
from std_msgs.msg import Float32

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

controllers = ["/lvel", "/rvel"]
output = [None] * len(controllers)

def xstr(s):
    if s is None:
        return 'null'
    return str(s)

def xround(x, n):
    if x is None:
        return None
    return round(x, n)

class SenderThread(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

    def run(self):
        while not self.stopped.wait(.000001):
            sock.sendto('[' + ','.join(xstr(xround(o, 4)) for o in output) + ']', ("192.168.2.1", 9999))

def callback(data):
    output[controllers.index(data._connection_header["topic"])] = data.data
    
def listener():
    rospy.init_node('motor_sender', anonymous=True)

    for controller in controllers:
        rospy.Subscriber(controller, Float32, callback)

    stopFlag = Event()
    thread = SenderThread(stopFlag)
    thread.daemon = True
    thread.start()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
