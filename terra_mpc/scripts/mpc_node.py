#!/usr/bin/env python3
import copy
import time
import numpy as np

import rospy
from std_msgs.msg import ColorRGBA
from fpn_msgs.msg import MPCInput, MPCOutput
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseArray, Pose
from sensor_msgs.msg import Image
from fpn_msgs.srv import ZZsetString, ZZsetStringResponse
import yaml

from mpc.mpc import MPC_CONTROLLER
from utils.mpc_utils import euler_from_quaternion, quaternion_from_euler

def myloginfo(msg=''):
    rospy.loginfo('['+str(rospy.get_name() + '] ' + str(msg)))
def mylogwarn(msg=''):
    rospy.logwarn('['+str(rospy.get_name() + '] ' + str(msg)))
def mylogerr(msg=''):
    rospy.logerr('['+str(rospy.get_name() + '] ' + str(msg)))

class MPC_NODE:
    def __init__(self):
        self.params = {}
        self.params['verbose'] = True
        self.params['frame_id'] = "base_footprint"


        # Initialize reference path
        self.mpc_reference = {'x': [], 'y': [], 'theta': [], 'speed': np.array([]), 'x_coeff': np.array([]), 'y_coeff': np.array([])}

        # Create data structure for state measurements
        self.odom_data = [0.0, 0.0, 0.0, 0.0]

        self.updateParams('/terrasentia/mpc2_node')
        # Create MHE object
        self.mpc = MPC_CONTROLLER(self.params)

        # And after setting everything we can set the publishers and subscribers
        twist_topic = rospy.get_param('~twist_topic_name', "/terrasentia/mpc_cmd_vel")
        #wp_topic    = rospy.get_param('~wp_topic_name', "/terrasentia/path")
        wp_topic = "/terrasentia/mpc_path_ref"

        # Subscribers
        rospy.Subscriber("/terrasentia/ekf", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(wp_topic + '2', Path, self.mpc_callback, queue_size=1)
        

        # Publisher
        self.pub_twist = rospy.Publisher(twist_topic, TwistStamped, queue_size=1)
        # Debug topics:
        self.pub_pred_vals = rospy.Publisher("mpc_node/mpc_pred_vals", PoseArray, queue_size=1)
        self.pub_pts_car = rospy.Publisher("mpc_node/pts_car", PoseArray, queue_size=1)
        #self.pub_marker = rospy.Publisher("mpc_node/pred_trav", Marker, queue_size=1)
        self.pub_output = rospy.Publisher("mpc_node/output", MPCOutput, queue_size=1)

        # Set run_step to wait for reference
        self.run_step = False

        # ROS Rate at 40Hz
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            self.run_mpc()
            rate.sleep()

    def updateParams(self, name):
        if not rospy.has_param(name + '/gain_ctrack_error_x'):
            mylogerr('MPC params under name ' + str(name) + ' were not found!!!')
            return False
        myloginfo('Loading MPC params under name ' + str(name) + '!!!')
        # Controller Hyper-parameters
        # self.params['dt']           = 0.2   # time between steps in seconds
        # self.params['N']            = 10    # number of look ahead steps
        # self.params['Lbase']        = 0.27  # distance between wheels
        # self.params['v_max']        = 1.5
        # self.params['v_lin_max']    = 0.7
        # self.params['v_ang_max']    = 6.0
        # self.params['n_states']     = 3
        # self.params['n_controls']   = 2
        # self.params['mu']           = 1.0
        # self.params['nu']           = 0.4
        # self.params['eps']          = 1e-6
        self.params['alpha']        = rospy.get_param(name + '/alpha', 1) #alpha=dt/(Rc+dt)
        self.params['dt']           = rospy.get_param(name + '/dt', 0.2)   # time between steps in seconds
        self.params['N']            = rospy.get_param(name + '/N', 10)     # number of look ahead steps
        self.params['Lbase']        = rospy.get_param(name + '/Lbase', 0.27)  # distance between wheels
        self.params['v_max']        = rospy.get_param(name + '/v_max', 1.5)
        self.params['v_lin_max']    = rospy.get_param(name + '/v_lin_max', 0.7)
        self.params['v_ang_max']    = rospy.get_param(name + '/v_ang_max', 6.0)
        self.params['n_states']     = rospy.get_param(name + '/n_states', 3)
        self.params['n_controls']   = rospy.get_param(name + '/n_controls', 2)
        self.params['mu']           = rospy.get_param(name + '/mu', 1.0)
        self.params['nu']           = rospy.get_param(name + '/nu', 0.4)
        self.params['eps']          = rospy.get_param(name + '/eps', 1e-6)
        self.params['v_ref']        = rospy.get_param(name + '/v_ref', 0.6)
        self.params['gain_ctrack_error_x']              = rospy.get_param(name + '/gain_ctrack_error_x', 20)
        self.params['gain_ctrack_error_y']              = rospy.get_param(name + '/gain_ctrack_error_y', 20)
        self.params['gain_ctrack_error_theta']          = rospy.get_param(name + '/gain_ctrack_error_theta', 1)
        self.params['terminal_cost_multiplier_x']       = rospy.get_param(name + '/terminal_cost_multiplier_x', 100)
        self.params['terminal_cost_multiplier_y']       = rospy.get_param(name + '/terminal_cost_multiplier_y', 100)
        self.params['terminal_cost_multiplier_theta']   = rospy.get_param(name + '/terminal_cost_multiplier_theta', 10)
        self.params['gain_control_effort_linear']       = rospy.get_param(name + '/gain_control_effort_linear', 10)
        self.params['gain_control_effort_angular']      = rospy.get_param(name + '/gain_control_effort_angular', 1)
        self.params['use_delay']                        = rospy.get_param(name + '/use_delay', False)
        return True


    def run_mpc(self):

        # Run only when a new reference path arrives
        if not self.run_step:
            return

        mpc_reference = copy.deepcopy(self.mpc_reference)


        if len(mpc_reference['x']) == 0: # self.params['N'] or len(mpc_reference['y']) < self.params['N'] or len(mpc_reference['theta']) < self.params['N']:
            if self.params['verbose']:
                rospy.loginfo("MPC empty wps")
            
            mpc_cmd = TwistStamped()
            mpc_cmd.header.stamp = rospy.Time.now()
            self.pub_twist.publish(mpc_cmd)
        
        else:
            if self.params['verbose']:
                rospy.loginfo("MPC wps are not empty")

            start = time.time()
            u, mpc_output, ss_error = self.mpc.solve_mpc(mpc_reference, self.odom_data[3])

            if self.params['verbose']:
                print('solve_mpc time:', time.time() - start)
                print('u:', u)
                print('mpc_output:', mpc_output)
                print('ss_error:', ss_error)

            # Publish command
            mpc_cmd = TwistStamped()
            mpc_cmd.header.stamp = rospy.Time.now()
            mpc_cmd.twist.linear.x = u[0,0]
            mpc_cmd.twist.angular.z = -u[1,0]

            # publish commands
            self.pub_twist.publish(mpc_cmd)
            # publish for debug
            self.debugPubs(mpc_output, mpc_reference)

        self.run_step = False

    def odom_callback(self, msg):
        _,_,heading = euler_from_quaternion(msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z,
                                            msg.pose.pose.orientation.w)
        # add new odomtry reading to the buffer
        self.odom_data = [msg.pose.pose.position.x, msg.pose.pose.position.y, heading, msg.twist.twist.angular.z]

    def mpc_callback(self, mpc_msg):
        # First, let's prevent the MPC to run while we are preparing the reference
        self.run_step = False
        # The input waypoints are interpolated to have the same distance between points as the distance step used in the MPC
        if len(mpc_msg.poses) == 0:
            if self.params['verbose']:
                print("Received empty path")
            
            self.mpc_reference['x'].clear()
            self.mpc_reference['y'].clear()
            self.mpc_reference['theta'].clear()
            self.mpc_reference['speed'] = np.array([])
    
        elif len(mpc_msg.poses) < 2:
            if self.params['verbose']:
                print("Path arrived with less than 4 points...")

            # Then we follow the points as the only value without regression
            self.mpc_reference['x'].clear()
            self.mpc_reference['y'].clear()
            self.mpc_reference['theta'].clear()
            self.mpc_reference['speed'] = np.ones(self.params['N']+1) * self.params['v_ref']
            
            for t in range(self.params['N']+1):
                x_ref = mpc_msg.poses[0].pose.position.x
                y_ref = mpc_msg.poses[0].pose.position.y
                heading_ref = np.arctan2(y_ref, x_ref)

                # Check if speed is negative and invert the heading to face forward
                if self.mpc_reference['speed'][t] < 0:
                    # Normalize angle to [-pi,pi)
                    heading_old = heading_ref
                    heading_ref = (heading_ref + np.pi) % (2*np.pi) - np.pi
                self.mpc_reference['x'].append(x_ref)
                self.mpc_reference['y'].append(y_ref)
                self.mpc_reference['theta'].append(heading_ref)
        
        else:
            if self.params['verbose']:
                print("Received path size:", len(mpc_msg.poses))
                print("\nCalling interpolatePath with ds", self.params['v_ref'] * self.params['dt'])

            # Number of points to use in the regression algorithm
            regress_length = len(mpc_msg.poses)

            # Populate vector for regression model
            wps_x = [mpc_msg.poses[0].pose.position.x]
            wps_y = [mpc_msg.poses[0].pose.position.y]
            wps_time = [0.0] #[np.sqrt(wps_x[0]*wps_x[0]+wps_y[0]*wps_y[0])/np.abs(self.params['v_ref'])]

            for i in range(1,regress_length):
                wps_x.append(mpc_msg.poses[i].pose.position.x)
                wps_y.append(mpc_msg.poses[i].pose.position.y)
                wps_time.append(wps_time[i-1] + np.sqrt((wps_x[i]-wps_x[i-1])**2+(wps_y[i]-wps_y[i-1])**2)/np.abs(self.params['v_ref']))

            # Fit 3rd order polynomial
            self.mpc_reference['x_coeff'] = np.polyfit(wps_time, wps_x, 3)
            self.mpc_reference['y_coeff'] = np.polyfit(wps_time, wps_y, 3)
            self.mpc_reference['speed'] = np.ones(self.params['N']+1) * self.params['v_ref']

            self.mpc_reference['x'].clear()
            self.mpc_reference['y'].clear()
            self.mpc_reference['theta'].clear()
            
            for t in range(self.params['N']+1):
                x_ref = np.polyval(self.mpc_reference['x_coeff'], t*self.params['dt'])
                y_ref = np.polyval(self.mpc_reference['y_coeff'], t*self.params['dt'])
                dxdt_ref = np.polyval(np.polyder(self.mpc_reference['x_coeff']), t*self.params['dt'])
                dydt_ref = np.polyval(np.polyder(self.mpc_reference['y_coeff']), t*self.params['dt'])

                heading_ref = np.arctan2(dydt_ref, dxdt_ref)

                # Check if speed is negative and invert the heading to face forward
                if self.mpc_reference['speed'][t] < 0:
                    # Normalize angle to [-pi,pi)
                    heading_old = heading_ref
                    heading_ref = (heading_ref + 2*np.pi) % (2*np.pi) - np.pi

                self.mpc_reference['x'].append(x_ref)
                self.mpc_reference['y'].append(y_ref)
                self.mpc_reference['theta'].append(heading_ref)

        # Now MPC can run
        self.run_step = True
    

    def debugPubs(self, mpc_output, mpc_reference):
        # Publish MPC predicted path
        pa = PoseArray()
        pa.header.frame_id = self.params['frame_id']
        for i in range(mpc_output.shape[1]):
            pose = Pose()
            pose.position.x = mpc_output[0,i]
            pose.position.y = mpc_output[1,i]

            q = quaternion_from_euler(0, 0, mpc_output[2,i])

            pose.orientation.w = q[0]
            pose.orientation.x = q[1]
            pose.orientation.y = q[2]
            pose.orientation.z = q[3]

            pa.poses.append(pose)         

        self.pub_pred_vals.publish(pa)

        # Publish MPC predicted output
        mpc_out = MPCOutput()
        for i in range(mpc_output.shape[1]):
            mpc_out.x.append(mpc_output[0,i])
            mpc_out.y.append(mpc_output[1,i])
        self.pub_output.publish(mpc_out)

        # Publish MPC reference path
        ref_pa = PoseArray()
        ref_pa.header.frame_id = self.params['frame_id']
        for i in range(len(mpc_reference['x'])):
            pose = Pose()
            pose.position.x = mpc_reference['x'][i]
            pose.position.y = mpc_reference['y'][i]

            q = quaternion_from_euler(0, 0, mpc_reference['theta'][i])
            pose.orientation.w = q[0]
            pose.orientation.x = q[1]
            pose.orientation.y = q[2]
            pose.orientation.z = q[3]
            ref_pa.poses.append(pose)

        self.pub_pts_car.publish(ref_pa)


def publish_params_from_yaml(namespace, yaml_file):
    with open(yaml_file, 'r') as stream:
        params = yaml.safe_load(stream)

    for key, value in params.items():
        full_param_name = f"{namespace}/{key}"
        rospy.set_param(full_param_name, value)
        rospy.loginfo(f"Published parameter '{full_param_name}' with value {value}")


if __name__ == '__main__':

    param_namespace = '/terrasentia'

    yaml_file_path = '../configs/mpc.yaml'  # Replace with the path to your YAML file
    publish_params_from_yaml(param_namespace, yaml_file_path)


    rospy.init_node('mpc_controller')
    MPC_NODE()
