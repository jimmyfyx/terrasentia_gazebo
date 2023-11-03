#!/usr/bin/env python3
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from scipy.spatial.transform import Rotation as R
import sys
import numpy as np
from std_srvs.srv import Empty

def mylogerr(msg=''):
    rospy.logerr('['+str(rospy.get_name() + '] ' + str(msg)))

class Teleporter:
    def __init__(self, n_lanes, lane_len, lane_width):
        self.n_lanes = n_lanes
        self.lane_len = lane_len
        self.lane_width = lane_width
        self.r = R.from_euler('zyx', [0, 0, 0], degrees=True).as_quat()
        self.r_opp =  R.from_euler('zyx', [180, 0, 0], degrees=True).as_quat()

        self.wps = self.create_waypoints()

        self.gt_sub = rospy.Subscriber('/terrasentia/ground_truth',Odometry,callback=self.gt_callback,queue_size=1)
        self.total_int_pub = rospy.Publisher('/terrasentia/intervention_count',Int8, queue_size = 1)
        self.auto_mode_pub = rospy.Publisher('/terrasentia/enable_auto_mode', Int8, queue_size = 1)
        self.row_num_pub = rospy.Publisher('/terrasentia/row_num', Int8, queue_size = 1)
        self.n_int = 0
        self.mission_finish = False
        self.row_finish = False
        self.row_num = 0


    def create_waypoints(self):
        wps = []
        for i in range(self.n_lanes):
            if i%2==0:
                wps.append({'start':{'y':-(2*i+1)*self.lane_width/2,'x':0, 'r':self.r}, 
                               'end':{'y':-(2*i+1)*self.lane_width/2,'x':self.lane_len, 'r':self.r} })
        
            else:
                wps.append({'start':{'y':-(2*i+1)*self.lane_width/2,'x':self.lane_len, 'r':self.r_opp}, 
                               'end':{'y':-(2*i+1)*self.lane_width/2,'x':0, 'r':self.r_opp} })

        return wps

    def gt_callback(self, msg):
        pos = msg.pose.pose.position
        lin_vel = msg.twist.twist.linear

        self.autonomy_handler(pos)
        self.row_num_update(pos)
        self.row_finish_check(pos)
        if self.row_num==self.n_lanes-1 and self.row_finish:
            self.mission_finish = True
        self.teleport_handler(pos,lin_vel)
        self.total_int_pub.publish(Int8(data=self.n_int))


    def row_num_update(self,pos):
        if self.auto_mode==True:
            self.row_num = int(-pos.y//self.lane_width)
            self.row_num_pub.publish(Int8(data=self.row_num))

    def autonomy_handler(self,pos):
        y_max = 0
        y_min = self.n_lanes*self.lane_width
        x_min = 0
        x_max = self.lane_len

        if pos.x>=x_min and pos.x<=x_max and pos.y>=y_min and pos.y<=y_max:
            self.auto_mode_pub.publish(Int8(data=1))
            self.auto_mode = True
        else:
            self.auto_mode_pub.publish(Int8(data=0))
            self.auto_mode = False


    def teleport_handler(self,pos,lin_vel):
        
        if abs(lin_vel.x)<0.01 and not self.mission_finish:
            if not self.row_finish:
                goal_y = self.wps[self.row_num]['start']['y']
                goal_x = pos.x
                goal_r = self.wps[self.row_num]['start']['r']

                self.teleport(goal_x,goal_y,goal_r)
                self.n_int+=1
                

            else:
                goal_x = self.wps[self.row_num+1]['start']['x']
                goal_y = self.wps[self.row_num+1]['start']['y']
                goal_r = self.wps[self.row_num+1]['start']['r']

                self.teleport(goal_x,goal_y,goal_r)
                self.row_finish = False


    def row_finish_check(self,pos):
        current_pos = np.array((pos.x, pos.y))
        mission_goal = self.wps[self.row_num]['end']
        goal_pos = np.array((mission_goal['x'],mission_goal['y']))

        dist = np.linalg.norm(current_pos-goal_pos)
        if self.auto_mode==False and dist<0.3:
            self.row_finish = True
        else:
            self.row_finish = False


    def teleport(self,goal_x, goal_y, goal_r):
        state_msg = ModelState()
        state_msg.model_name = 'terrasentia'
        state_msg.pose.position.z = 0
        state_msg.pose.position.x = goal_x
        state_msg.pose.position.y = goal_y
              
        state_msg.pose.orientation.x = goal_r[0]
        state_msg.pose.orientation.y = goal_r[1]
        state_msg.pose.orientation.z = goal_r[2]
        state_msg.pose.orientation.w = goal_r[3]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException as e:
            print("Service call failed")

    

def main():
    rospy.init_node('teleport_node')
    n_lanes = 7
    lane_len = 21
    lane_width = 0.76
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_world()
    teleport = Teleporter(n_lanes,lane_len, lane_width)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        mylogerr("Shutting down ROS Image neural network prediction module")
 
        

if __name__ == '__main__':
    sys.exit(main() or 0)