#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2


class SpherexCtrl(object):

    def __init__(self):
        print('Spherex control booting...')
        rospy.init_node('spherex_ctrl')
        rate = rospy.Rate(1);
        self.lidar_ctrl = rospy.Publisher('lidar_ctrl', String, queue_size=10)
        self.lidar_stream = rospy.Publisher('lidar_stream', PointCloud2, queue_size=10)
        self.raw_lidar_stream = rospy.Subscriber(
                'raw_lidar_stream', LaserScan, self.handle_scan)
        self.laser_projection = LaserProjection()
        while not rospy.is_shutdown():
            self.lidar_ctrl.publish('scan')
            #print('writing')
            rate.sleep()

    def handle_scan(self, data):
        cloud = self.laser_projection.projectLaser(data)
        self.lidar_stream.publish(cloud)
        #points = point_cloud2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
        #for p in points:
        #    self.lidar_stream.publish(points)
            

def main():
    pass
    # lidar scan
    # read lidar data
    # convert to cartesian


if __name__ == '__main__':
    SpherexCtrl()
