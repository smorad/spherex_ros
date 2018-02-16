#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud


class SpherexCtrl(object):

    def __init__(self):
        print('Spherex control booting...')
        rospy.init_node('spherex_ctrl')
        rate = rospy.Rate(1);
        self.input_cloud = rospy.Subscriber(
                'raw_pointcloud_stream', PointCloud, self.buffer)
        self.output_cloud = rospy.Publisher('pointcloud_buffer', PointCloud)
        while not rospy.is_shutdown():
            rate.sleep()
            self.publish_cloud()
            
    def buffer(self, data):
        self.cloud = data

    def publish_cloud(self):
        self.output_cloud.publish(self.cloud)
        print('published scan data to processor')
        #points = point_cloud2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
        #for p in points:
        #    self.lidar_stream.publish(points)
            

if __name__ == '__main__':
    SpherexCtrl()
