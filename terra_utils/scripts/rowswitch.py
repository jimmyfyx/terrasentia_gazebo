#!/usr/bin/env python3
import rospy 
import rospkg 
import sys
import numpy as np
import json

from scipy.spatial.transform import Rotation as R

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8, Float32MultiArray
from sensor_msgs.msg import CameraInfo, CompressedImage


def mylogerr(msg=''):
    rospy.logerr('['+str(rospy.get_name() + '] ' + str(msg)))

class RowSwitch:
    def __init__(self):
        self.pub_path_noi = rospy.Publisher("/terrasentia/mpc_path_noi", Path, queue_size=10)
        self.pub_path_ref = rospy.Publisher("/terrasentia/mpc_path_ref", Path, queue_size=10)
        self.sub_odom = rospy.Subscriber("/terrasentia/ground_truth", Odometry, self.odom_callback)

        # Read routes
        self.env_config_path = "/home/jimmy/catkin_ws/src/terra-simulation/terra_worlds/configs/env_0"
        f = open(f"{self.env_config_path}/routes_config.json")
        self.routes = json.load(f)

        self.mpc_path_world_noi = []
        self.mpc_path_world_ref = []
        self.mpc_path_body_noi = []
        self.mpc_path_body_ref = []
        self.set_robot = True  # Flag to reset robot initial position for the new route
        self.cur_route = 0  # Track current route index
        self.num_routes = len(self.routes)  # Total number of routes 
        self.env_complete = False  # Flag to record all routes are completed

        # Real-time robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_quatx = 0.0
        self.robot_quaty = 0.0
        self.robot_quatz = 0.0
        self.robot_quatw = 0.0
    
    @staticmethod
    def get_quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_quatx = msg.pose.pose.orientation.x
        self.robot_quaty = msg.pose.pose.orientation.y
        self.robot_quatz = msg.pose.pose.orientation.z
        self.robot_quatw = msg.pose.pose.orientation.w

        # Transformation
        robot_pos = np.array([self.robot_x, self.robot_y, 0.0])
        robot_quat = np.array([self.robot_quatx, self.robot_quaty, self.robot_quatz, self.robot_quatw])
        norm = np.linalg.norm(robot_quat)
        robot_quat_norm = robot_quat / norm

        if not self.set_robot and not self.env_complete:
            route = self.routes[f"route_{self.cur_route}"]
            noi_route = route["noi_waypoints"]
            ref_route = route["ref_waypoints"]
            init_lane = route["init_lane"]
            target_lane = route["target_lane"]
            noi_target_x = route["noi_target_x"]
            noi_target_y = route["noi_target_y"]

            # Whether the robot reach destination of current route and complete all routes
            if abs(self.robot_x - noi_target_x) < 0.05 and abs(self.robot_y - noi_target_y) < 0.05:
                self.cur_route += 1
                if self.cur_route > self.num_routes:
                    self.env_complete = True  # Complete all routes
                else:
                    self.set_robot = True  # Complete current route
                    self.set_robot_state()
            else:
                # Prepare MPC path (noisy and reference)
                # Detremine MPC path in world frame
                self.mpc_path_world_noi = []
                self.mpc_path_world_ref = []
                if target_lane < init_lane:
                    for i in range(len(noi_route)):
                        if noi_route[i][1] < self.robot_y:
                            self.mpc_path_world_noi.append(noi_route[i])
                    for i in range(len(ref_route)):
                        if ref_route[i][1] < self.robot_y:
                            self.mpc_path_world_ref.append(ref_route[i])
                else:
                    for i in range(len(noi_route)):
                        if noi_route[i][1] > self.robot_y:
                            self.mpc_path_world_noi.append(noi_route[i])
                    for i in range(len(ref_route)):
                        if ref_route[i][1] > self.robot_y:
                            self.mpc_path_world_ref.append(ref_route[i])         
                
                # Transform MPC path to body frame
                self.mpc_path_body_noi = []
                self.mpc_path_body_ref = []
                for i in range(len(self.mpc_path_world_noi)):
                    point = np.array([self.mpc_path_world_noi[i][0], self.mpc_path_world_noi[i][1], 0.0])
                    translation = point - robot_pos
                    rotation = R.from_quat(robot_quat_norm)
                    rotation_inv = rotation.inv()
                    point_body = rotation_inv.apply(translation)
                    self.mpc_path_body_noi.append([point_body[0], point_body[1]])
                for i in range(len(self.mpc_path_world_ref)):
                    point = np.array([self.mpc_path_world_ref[i][0], self.mpc_path_world_ref[i][1], 0.0])
                    translation = point - robot_pos
                    rotation = R.from_quat(robot_quat_norm)
                    rotation_inv = rotation.inv()
                    point_body = rotation_inv.apply(translation)
                    self.mpc_path_body_ref.append([point_body[0], point_body[1]])
                
                # Publish path
                mpc_path_noi = Path()
                mpc_path_noi.header.frame_id = "map"
                mpc_path_ref = Path()
                mpc_path_ref.header.frame_id = "map"
                
                for i in range(len(self.mpc_path_body_noi)):
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = self.mpc_path_body_noi[i][0]
                    pose.pose.position.y = self.mpc_path_body_noi[i][1]
                    mpc_path_noi.poses.append(pose)
                for i in range(len(self.mpc_path_body_ref)):
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = self.mpc_path_body_ref[i][0]
                    pose.pose.position.y = self.mpc_path_body_ref[i][1]
                    mpc_path_ref.poses.append(pose)
                
                mpc_path_noi.header.stamp = rospy.Time.now()
                mpc_path_ref.header.stamp = rospy.Time.now()
                self.pub_path_noi.publish(mpc_path_noi)
                self.pub_path_ref.publish(mpc_path_ref)
        elif self.set_robot and not self.env_complete:
            self.set_robot_state()

    def set_robot_state(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            # Prepare robot pose
            route = self.routes[f"route_{self.cur_route}"]
            init_x = route["init_x"]
            init_y = route["init_y"]
            init_yaw = route["init_yaw"]
            quat = self.get_quaternion_from_euler(0.0, 0.0, init_yaw)

            state_msg = ModelState()
            state_msg.model_name = 'terrasentia'
            state_msg.pose.position.x = init_x
            state_msg.pose.position.y = init_y
            state_msg.pose.position.z = 0.2
            state_msg.pose.orientation.x = quat[0]
            state_msg.pose.orientation.y = quat[1]
            state_msg.pose.orientation.z = quat[2]
            state_msg.pose.orientation.w = quat[3]

            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg)
            self.set_robot = False
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed")
            
def main():
    rospy.init_node('rowswitch_node')
    row_switch_node = RowSwitch()
    rospy.loginfo("Rowswitch node has started...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        mylogerr("Shutting down node")
 
        

if __name__ == '__main__':
    sys.exit(main() or 0)