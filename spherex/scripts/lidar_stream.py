#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from laser_geometry import LaserProjection

class Lidar(object):
    def __init__(self):
        print('Spherex lidar booting...')
        rospy.init_node('lidar')
        rospy.Subscriber('lidar_ctrl', String, self.scan)
        rospy.Subscriber('base_scan', LaserScan, self.buffer)
        self.scan_pub = rospy.Publisher('lidar_stream', LaserScan, queue_size=10)
        self.laser = LaserProjection()
        self.scan_data = None
        rospy.spin()

    def buffer(self, data):
        self.scan_data = data

    def scan(self, _):
        print('scan data requested')
        if hasattr(self, 'scan_data') and self.scan_data:
            print('publishing scan data')
            self.scan_pub.publish(self.scan_data)
            self.scan_data = None



if __name__ == '__main__':
    Lidar()
