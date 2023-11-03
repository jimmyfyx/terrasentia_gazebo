#!/usr/bin/env python3
import rospy
import numpy as np
from random import uniform
from scipy.spatial.transform import Rotation

from std_srvs.srv import Empty
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from fpn_msgs.msg import MPCInput, GoalMultiArray, Goal

GPS_GOAL = True
REF_LAT = 40.072209
REF_LON = -88.209822
INIT_X = 0.0
INIT_Y = 0.0
  
class RandomExploration(object):
    def __init__(self):
        self._goal_x = uniform(0, 9)
        self._goal_y  = uniform(0, 9)
        self._odomsub = rospy.Subscriber('/terrasentia/ground_truth', Odometry , self.receiveOdom)
        if GPS_GOAL:
            self._goalpub = rospy.Publisher("/terrasentia/goals", GoalMultiArray, queue_size=1)
        else:
            self._goalpub = rospy.Publisher("/terrasentia/path2", MPCInput, queue_size=1)
        self._start_time2goal = rospy.Time.now().secs

    def newRandomGoal(self):      
        self._goal_x = uniform(0, 9)
        self._goal_y  = uniform(0, 9)

    def receiveOdom(self, msg):
		# Current robot's position
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        rotate = msg.pose.pose.orientation
        #(roll, pitch, yaw) = euler_from_quaternion([rotate.x, rotate.y, rotate.z, rotate.w])
        R_local2map = Rotation.from_quat([rotate.x, rotate.y, rotate.z, rotate.w])

        # Check if robot has tipped over
        yaw, pitch, roll  = R_local2map.as_euler('zxy', degrees=True)
        if np.abs(pitch) > 80.0 or np.abs(roll) > 80.0:
            rospy.wait_for_service('/gazebo/reset_world')
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
        
        d_state = np.array([[self._goal_x - curr_x,
							 self._goal_y - curr_y,
							 0]]).T

        d_state = R_local2map.inv().as_matrix() @ d_state

        delta_x = d_state[0]
        delta_y = d_state[1]
        dist_goal = np.sqrt(delta_x**2 + delta_y**2)

        if((dist_goal < 0.2) or (rospy.Time.now().secs - self._start_time2goal > 8.0)):
            self.newRandomGoal()
            self._start_time2goal = rospy.Time.now().secs
            return
        
        if GPS_GOAL:
            '''
            The output message is a custom fpn_msgs::MPCInput msg.
            This is used as reference state trajectory for the Model Predictive Controller
            we use in the TerraSentia robots
            '''
            R = 6378137.0   # Earth's radius
            # Coordinate offsets in radians
            dLat = delta_y/R
            dLon = delta_x/(R*np.cos(np.pi*REF_LAT/180.0))

            # OffsetPosition, decimal degrees
            new_lat = REF_LAT + dLat * 180/np.pi
            new_lon = REF_LON + dLon * 180/np.pi 

            goal_msg = GoalMultiArray()
            goal_msg.zero_x = INIT_X
            goal_msg.zero_y = INIT_Y
            goal_msg.zero_lat = REF_LAT
            goal_msg.zero_lon = REF_LON

            # goal point
            gps_goal = Goal()
            gps_goal.latitude = new_lat
            gps_goal.longitude = new_lon
            goal_msg.data.append(gps_goal)

            self._goalpub.publish(goal_msg)
        else:
            '''
            The output message is a custom fpn_msgs::GoalMultiArray msg.
            This is used as reference state trajectory for the Model Predictive Controller
            we use in the TerraSentia robots
            '''
            mpc_reference = MPCInput()
            mpc_reference.header.stamp = rospy.Time.now()
            mpc_reference.header.frame_id = "/base_link"
            
            path_msg = Path()
            path_msg.header = mpc_reference.header
            
            ds = np.linspace(0, dist_goal, 25)
            angle = np.arctan2(delta_y, delta_x)
            
            path_msg.poses.clear()
            print('pose.pose.position.x:')
            print('pose.pose.position.y:')
            for i in range(len(ds)):
                pose = PoseStamped()
                pose.pose.position.x = ds[i]*np.cos(angle)
                pose.pose.position.y = ds[i]*np.sin(angle)
                path_msg.poses.append(pose)
                print(pose.pose.position.x, pose.pose.position.y)
            
            mpc_reference.poses = path_msg.poses
            mpc_reference.reference_speed = 0.7

            self._goalpub.publish(mpc_reference)       

if __name__ == "__main__":
    rospy.init_node("random_exploration")
    random_exploration = RandomExploration()
    rospy.spin()
