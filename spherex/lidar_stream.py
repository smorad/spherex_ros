#!/usr/bin/env python
from laser_geometry import LaserProjection
import rospy
#from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import String

GAZEBO = True

class Lidar(object):
    def __init__(self):
        if GAZEBO:
            return
        print('Spherex lidar booting...')
        rospy.init_node('lidar')
        rospy.Subscriber('lidar_ctrl', String, self.scan)
        rospy.Subscriber('raw_pointcloud_stream', PointCloud, self.buffer)

        self.scan_pub = rospy.Publisher('raw_cloud_buffer', PointCloud, queue_size=10)
        self.scan_data = None
        rospy.spin()

    def buffer(self, data):
        self.cloud_data = data

    def scan(self, _):
        print('cloud data requested')
        if hasattr(self, 'cloud_data') and self.cloud_data:
            self.cloud_pub.publish
            self.cloud_data = None


if __name__ == '__main__':
    pass
    #Lidar()
