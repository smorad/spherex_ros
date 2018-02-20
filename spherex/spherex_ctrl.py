#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64MultiArray


class SpherexCtrl(object):
    
    run_once = False

    def __init__(self):
        print('Spherex control booting...')
        rospy.init_node('spherex_ctrl')
        rate = rospy.Rate(0.5);
        self.input_cloud = rospy.Subscriber(
                                            'raw_pointcloud_stream', PointCloud, self.buffer)
        self.output_cloud = rospy.Publisher(
                                            'pointcloud_buffer', PointCloud)                
        self.hop_ctrl = rospy.Publisher(
                                        'thruster_cmd', Float64MultiArray, queue_size=100)
        while not rospy.is_shutdown():
            rate.sleep()
            self.publish_cloud()
            if not self.run_once:
                self.run_once = True
                self.hop(0, 100, 0, 2, 0)
                self.hop(100, 0, 0, 2, 0)

            
    def buffer(self, data):
        self.cloud = data
        
    def hop(self, x, y, z, duration, use_global_frame):
        hop_impulse = Float64MultiArray(data=[x, y, z, duration, use_global_frame])
        self.hop_ctrl.publish(hop_impulse)

    def publish_cloud(self):
        if hasattr(self, 'cloud') and self.cloud:
            self.output_cloud.publish(self.cloud)
            print('published scan data to processor')
            

if __name__ == '__main__':
    SpherexCtrl()
