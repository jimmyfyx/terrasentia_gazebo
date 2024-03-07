#!/usr/bin/env python3
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Float32MultiArray
from sensor_msgs.msg import CameraInfo
from scipy.spatial.transform import Rotation as R
import sys
import numpy as np

def mylogerr(msg=''):
    rospy.logerr('['+str(rospy.get_name() + '] ' + str(msg)))

class RowfollowGt:
    def __init__(self,lane_width):
 
        self.lane_width = lane_width
        self.r = R.from_euler('zyx', [0, 0, 0], degrees=True).as_quat()
        self.r_opp =  R.from_euler('zyx', [180, 0, 0], degrees=True).as_quat()

        self.gt_sub = rospy.Subscriber('/terrasentia/ground_truth',Odometry,callback=self.gt_callback,queue_size=1)
     
        self.heading_pub = rospy.Publisher('/terrasentia/heading_error', Float32MultiArray, queue_size = 1) 
        self.distance_pub = rospy.Publisher('/terrasentia/distance_error', Float32MultiArray, queue_size = 1) 

        self.row_num = 0


    def gt_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.row_num_update(pos)
        yaw, roll, pitch  = R.from_quat([ori.x, ori.y, ori.z, ori.w]).as_euler('zxy', degrees=False)

        if self.row_num%2==0:
            heading = -yaw
            dl_cg = (-pos.y-(self.row_num*self.lane_width))
            dr_cg = self.lane_width-dl_cg

            l1 = np.array([-dl_cg,0,-pos.z])
            l2 = np.array([-dl_cg,0.23222,-pos.z])

            r1 = np.array([dr_cg,0,-pos.z])
            r2 = np.array([dr_cg,0.23222,-pos.z])

        else:
            heading = np.pi-yaw
            dr_cg = (-pos.y-(self.row_num*self.lane_width))
            dl_cg = self.lane_width-dr_cg

            l1 = np.array([-dl_cg,0,-pos.z])
            l2 = np.array([-dl_cg,0.23222,-pos.z])

            r1 = np.array([dr_cg,0,-pos.z])
            r2 = np.array([dr_cg,0.23222,-pos.z])

        van_lines_rot = R.from_euler('zyx', [-yaw, -pitch, -roll], degrees=True).as_matrix()
        l1 = van_lines_rot@l1
        l2 = van_lines_rot@l2

        r1 = van_lines_rot@r1
        r2 = van_lines_rot@r2

    
        self.heading_pub.publish(Float32MultiArray(data=[heading]))
        self.distance_pub.publish(Float32MultiArray(data=[(-l2[0]-r2[0])/2]))

    def row_num_update(self,pos):
        self.row_num = int(-pos.y//self.lane_width)

def main():
    rospy.init_node('rowfollow_gt_node')
    lane_width = 0.76
    rowfollowgt = RowfollowGt(lane_width)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        mylogerr("Shutting down ROS Image neural network prediction module")
 
        

if __name__ == '__main__':
    sys.exit(main() or 0)
