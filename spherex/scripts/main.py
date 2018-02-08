#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class SpherexCtrl(object):

    def __init__(self):
        print('Spherex control booting...')
        rospy.init_node('spherex_ctrl')
        rate = rospy.Rate(1);
        self.lidar_ctrl = rospy.Publisher('lidar_ctrl', String, queue_size=10)
        self.lidar_stream = rospy.Subscriber('lidar_stream', LaserScan, self.handle_scan)
        while not rospy.is_shutdown():
            self.lidar_ctrl.publish('scan')
            #print('writing')
            rate.sleep()

    def handle_scan(self, data):
        print('handle scan got ', data)
            

def main():
    pass
    # lidar scan
    # read lidar data
    # convert to cartesian


if __name__ == '__main__':
    SpherexCtrl()
