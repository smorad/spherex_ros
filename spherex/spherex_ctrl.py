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
                                            'pointcloud_buffer', PointCloud, queue_size=1000)                
        self.hop_ctrl = rospy.Publisher(
                                        'thruster_cmd', Float64MultiArray, queue_size=1000)
        #self.cam1_stream = rospy.Subscriber('cam1_stream', Image, queue_size=10)
        rospy.Timer(rospy.Duration(5), self.do_hop)
        while not rospy.is_shutdown():
            rate.sleep()
            self.publish_cloud()
            #if not self.run_once:
                #self.run_once = True
            #self.hop(10, 0.0, 0, 0.001, 1.0)
            #self.hop(5000, 0, 0, 0.00001, 1)
            
            
            
            
    def do_hop(self, *args, **kwargs):
        self.hop([1.0, 0.0, 1.0], 2.0, 1.0)
            
    def buffer(self, data):
        self.cloud = data
        
    def hop(self, unit_vector, v0, use_global_frame):
        # ensure vec is normed
        unit_vector = [i * i / sum(unit_vector) for i in unit_vector]
        hop_impulse = Float64MultiArray(
                                        data=[unit_vector[0], unit_vector[1], unit_vector[2], v0, use_global_frame])
        self.hop_ctrl.publish(hop_impulse)
        

    def publish_cloud(self):
        if hasattr(self, 'cloud') and self.cloud:
            self.output_cloud.publish(self.cloud)
            print('published scan data to processor')

if __name__ == '__main__':
    SpherexCtrl()
