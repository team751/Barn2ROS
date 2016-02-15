#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import laser_geometry

rospy.init_node('listener', anonymous=True)
pub = rospy.Publisher('scanpc', PointCloud2, queue_size=10)


laserGeo = laser_geometry.LaserProjection()

def callback(data):
    transformed = laserGeo.projectLaser(data, -1, (laser_geometry.LaserProjection.ChannelOption.VIEWPOINT | laser_geometry.LaserProjection.ChannelOption.INDEX | laser_geometry.LaserProjection.ChannelOption.INTENSITY))
    rospy.loginfo("DATA IN");
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", transformed)
    pub.publish(transformed)
    
def listener():
    rospy.Subscriber("/laserscan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()