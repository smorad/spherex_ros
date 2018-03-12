#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64MultiArray


class SpherexCtrl(object):
    
    run_once = False

    def __init__(self):
        print('Spherex control booting...')
        rospy.init_node('spherex_ctrl')
        # hz
        rate = rospy.Rate(20);
        self.input_cloud = rospy.Subscriber(
                                            'raw_pointcloud_stream', PointCloud, self.buffer)
        self.output_cloud = rospy.Publisher(
                                            'pointcloud_buffer', PointCloud, queue_size=1)                
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
            
    def compute_free_boundary(self):
        '''Given the merged point cloud data, compute the free
        boundary'''
        pass
    
    def compute_target_vector(self):
        '''Given the free boundary, compute the target vector'''
        # d = v0*cos(alpha) * t
        # t = t1 + t2
        # t1 = v0*sin(alpha) / g
        # t2 = sqrt(
        #   (v0 * sin(alpha)) ** 2 / g ** 2 - 2 * h / g
        # )
        
        # d should be max visible distance, optimize for minimum v0
        pass
    
    def compute_trajectory(self):
        '''Given the target vector, jump as far as safely possible
        (where we still have map data)'''
        pass

if __name__ == '__main__':
    SpherexCtrl()
