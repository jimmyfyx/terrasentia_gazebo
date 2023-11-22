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
        self.pub_route_id = rospy.Publisher("/terrasentia/route_id", Int8, queue_size=10)
        self.pub_twist = rospy.Publisher("/terrasentia/cmd_vel", TwistStamped, queue_size=10)
        self.sub_odom = rospy.Subscriber("/terrasentia/ground_truth", Odometry, self.odom_callback)
        self.sub_mpc_cmd = rospy.Subscriber("/terrasentia/mpc_cmd_vel", TwistStamped, self.mpc_cmd_callback)
        

        # Read routes
        self.env_config_path = rospy.get_param('~env_config_path', "/home/daslab/catkin_row_turning/src/terrasentia_gazebo/terra_worlds/configs/env_0")
        # self.env_config_path = "/home/daslab/catkin_row_turning/src/terrasentia_gazebo/terra_worlds/configs/env_0"
        f = open(f"{self.env_config_path}/routes_config.json")
        self.routes = json.load(f)

        self.mpc_path_world_noi = []
        self.mpc_path_world_ref = []
        self.mpc_path_body_noi = []
        self.mpc_path_body_ref = []
        self.cur_route = 0  # Track current route index
        self.nearest_wp_index = 0
        self.final_stage = False  # Flag to indicate robot at the final stage of every route
        self.num_routes = len(self.routes)  # Total number of routes 
        self.env_complete = False  # Flag to record all routes are completed

        # Real-time robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_quatx = 0.0
        self.robot_quaty = 0.0
        self.robot_quatz = 0.0
        self.robot_quatw = 0.0

        self.set_robot_state()
    
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
    def mpc_cmd_callback(self,msg):
        """
        Adding noise to the MPC command
        """
        twist_stamped = TwistStamped()
        twist_stamped.twist.linear.x = msg.twist.linear.x
        twist_stamped.twist.linear.y = msg.twist.linear.y
        twist_stamped.twist.linear.z = msg.twist.linear.z
        twist_stamped.twist.angular.x = msg.twist.angular.x
        twist_stamped.twist.angular.y = msg.twist.angular.y
        if 8 <= self.nearest_wp_index <= 13: # adding noise in this interval
            twist_stamped.twist.angular.z = np.clip(msg.twist.angular.z + 10.0*(np.random.rand()-0.5), -6.0, 6.0)
        else:
            twist_stamped.twist.angular.z = msg.twist.angular.z
        self.pub_twist.publish(twist_stamped)
        # print(self.nearest_wp_index)
        
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_quatx = msg.pose.pose.orientation.x
        self.robot_quaty = msg.pose.pose.orientation.y
        self.robot_quatz = msg.pose.pose.orientation.z
        self.robot_quatw = msg.pose.pose.orientation.w

        # Transformation matrix (world -> body)
        robot_pos = np.array([self.robot_x, self.robot_y, 0.0])
        robot_quat = np.array([self.robot_quatx, self.robot_quaty, self.robot_quatz, self.robot_quatw])
        norm = np.linalg.norm(robot_quat)
        robot_quat_norm = robot_quat / norm

        if not self.env_complete and not self.final_stage:
            route = self.routes[f"route_{self.cur_route}"]
            noi_route = route["noi_waypoints"]
            ref_route = route["ref_waypoints"]
            init_lane = route["init_lane"]
            target_lane = route["target_lane"]
            init_x = route["init_x"]
            init_y = route["init_y"]
            ref_init_x = route["ref_init_x"] #note ref_init_x is same as init_x
            ref_init_y = route["ref_init_y"]
            ref_target_x = route["ref_target_x"]
            ref_target_y = route["ref_target_y"]

            # ref_target_x = route["ref_target_x"]
            # ref_target_y = route["ref_target_y"]
            # noi_target_x = route["noi_target_x"]
            # noi_target_y = route["noi_target_y"]

            # print('x_diff', self.robot_x - noi_target_x)
            # print('y_diff', self.robot_y - noi_target_y)
            # print(f'robot_x:{self.robot_x}, robot_y:{self.robot_y}')
            
            # Publish current route id
            self.pub_route_id.publish(self.cur_route)

            # Whether the robot complete all routes or current route
            if (abs(self.robot_x - ref_target_x) < 0.1 and abs(self.robot_y - ref_target_y) < 0.1) or self.nearest_wp_index==14:
                print('goal reached')
                self.final_stage = True
            else:
                # Prepare MPC path (noisy and reference)
                # Detremine MPC path in world frame

                self.mpc_path_world_noi = noi_route.copy()

                for i in range(self.nearest_wp_index,len(ref_route)):
                    if abs(self.robot_x - ref_route[self.nearest_wp_index][0]) < 0.2 and abs(self.robot_y - ref_route[self.nearest_wp_index][1]) < 0.2:
                        self.nearest_wp_index += 1
                    self.mpc_path_world_ref = ref_route[self.nearest_wp_index:]

                

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
                mpc_path_noi.header.frame_id = "base_footprint"
                mpc_path_ref = Path()
                mpc_path_ref.header.frame_id = "base_footprint"
                
                for i in range(len(self.mpc_path_body_noi)):
                    pose = PoseStamped()
                    pose.header.frame_id = "base_footprint"
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = self.mpc_path_body_noi[i][0]
                    pose.pose.position.y = self.mpc_path_body_noi[i][1]
                    mpc_path_noi.poses.append(pose)
                for i in range(len(self.mpc_path_body_ref)):
                    pose = PoseStamped()
                    pose.header.frame_id = "base_footprint"
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = self.mpc_path_body_ref[i][0]
                    pose.pose.position.y = self.mpc_path_body_ref[i][1]
                    mpc_path_ref.poses.append(pose)
                
                mpc_path_noi.header.stamp = rospy.Time.now()
                mpc_path_ref.header.stamp = rospy.Time.now()
                self.pub_path_noi.publish(mpc_path_noi)
                self.pub_path_ref.publish(mpc_path_ref)

        elif not self.env_complete and self.final_stage:
           
            self.cur_route += 1
            if self.cur_route >= self.num_routes:
                self.env_complete = True  # Complete all routes
                self.final_stage = False
            else:
                self.final_stage = False
                self.nearest_wp_index = 0
                self.set_robot_state()

        else:
            # Stop the robot
            twist_stamped = TwistStamped()
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.linear.y = 0.0
            twist_stamped.twist.linear.z = 0.0
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = 0.0

            self.pub_twist.publish(twist_stamped)

        
            

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
