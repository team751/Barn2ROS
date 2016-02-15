#!/usr/bin/env python
import rospy
import socket
import threading
from threading import Thread
from threading import Event
from std_msgs.msg import Float32

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rospy.init_node('motor_sender', anonymous=True)

controllers = rospy.get_param('~controller_topics')
ipAddress = rospy.get_param('~network/ip', '127.0.0.1')
port = rospy.get_param('~network/port', 9999)
waitTime = rospy.get_param('~send_wait_time', .000001)
roundTo = rospy.get_param('~round_to', 4)
messagePrefix = rospy.get_param('~message_prefix', "[")
messageSuffix = rospy.get_param('~message_suffix', "]")
messageDivider = rospy.get_param('~message_divider', ",")

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
        while not self.stopped.wait(waitTime):
            message = messagePrefix + messageDivider.join(xstr(xround(o, roundTo)) for o in output) + messageSuffix
            sock.sendto(message, (ipAddress, port))

def callback(data):
    output[controllers.index(data._connection_header["topic"])] = data.data
    
def listener():  
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
