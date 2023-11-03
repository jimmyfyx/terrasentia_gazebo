#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Float32MultiArray
from sensor_msgs.msg import CameraInfo, CompressedImage
from scipy.spatial.transform import Rotation as R
import sys
import numpy as np
import cv2

#Note:  gt_callback is not fully fixed yet

def mylogerr(msg=''):
    rospy.logerr('['+str(rospy.get_name() + '] ' + str(msg)))

class RowfollowGt:
    def __init__(self,lane_width, lane_len,n_lanes):
 
        self.lane_width = lane_width
        self.lane_len = lane_len
        self.n_lanes = n_lanes
        self.r = R.from_euler('zyx', [0, 0, 0], degrees=True).as_quat()
        self.r_opp =  R.from_euler('zyx', [180, 0, 0], degrees=True).as_quat()

        self.row_num = 0
        self.fx = 0
        self.fy = 0
        self.px = 0
        self.py = 0
        self.height = 0
        self.width = 0
        self.vpx = 0
        self.vpy = 0
        self.r_int_x = 0
        self.l_int_x = 0

        self.points = self.crop_row_pos()

        self.cam_intrinsics_sub = rospy.Subscriber('/terrasentia/front_cam_node/camera_info',CameraInfo,callback=self.calib_update,queue_size=1)
        self.row_num_sub = rospy.Subscriber('/terrasentia/row_num', Int8,callback=self.row_num_update, queue_size = 1)
        
        self.gt_sub = rospy.Subscriber('/terrasentia/ground_truth',Odometry,callback=self.gt_callback,queue_size=1)
        self.img_subscriber = rospy.Subscriber('/terrasentia/front_cam_node/image_raw/compressed',
            CompressedImage, self.kp_publisher, queue_size = 1, buff_size=2**24)  

        self.heading_pub = rospy.Publisher('/terrasentia/vision/heading', Float32MultiArray, queue_size = 1) 
        self.distance_pub = rospy.Publisher('/terrasentia/vision/distance', Float32MultiArray, queue_size = 1) 
        self.kp_gt_img = rospy.Publisher('/terrasentia/front_cam_node/kp_gt_img/compressed',CompressedImage, queue_size=1)

        

    def calib_update(self, msg):
        K = msg.K
        self.fx = K[0]
        self.fy = K[4]
        self.px = K[2]
        self.py = K[5]
        self.height = msg.height
        self.width = msg.width

    def crop_row_pos(self):
        #creating points referring to position of start and end of crop rows in camera coordinate frame  (but without rotating the axis to align with default camera frames)
        points = []
        for i in range(self.n_lanes):
            if i%2==0:
                points.append([[[-0.23222,-i*self.lane_width+0.38,-0.27],[self.lane_len-0.23222,-i*self.lane_width+0.38,-0.27]],
                                [[-0.23222,-(i+1)*self.lane_width+0.38,-0.27],[self.lane_len-0.23222,-(i+1)*self.lane_width+0.38,-0.27]]])
        
            else:
                points.append([[[self.lane_len-0.23222,-(i+1)*self.lane_width+0.38,-0.27],[-0.23222,-(i+1)*self.lane_width+0.38,-0.27]], 
                               [[self.lane_len-0.23222,-i*self.lane_width+0.38,-0.27],[-0.23222,-i*self.lane_width+0.38,-0.27]]])

        return points

    
    # TO DO: following function doesn't work yet 
    def gt_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        rot = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        yaw, roll, pitch  = rot.as_euler('zxy', degrees=True)
       
        cg_init_pos = [0,-0.38,0.13]
        cam_init_pos = [0.23222,-0.38,0.27]

        cg_new_pos = [pos.x,pos.y,pos.z]
        translation = [cg_new_pos[i]-cg_init_pos[i] for i in range(len(cg_init_pos))]
        print('rot is {}'.format(rot.as_matrix()))
        cam_rot = rot.apply(cam_init_pos)
        cam_new_pos = [cam_rot[i]+translation[i] for i in range(3)]

        #cam_new_pos = rot.apply([cam_init_pos[i]+translation[i] for i in range(3)])
        #cam_new_pos = [cam_init_pos[i]+translation[i] for i in range(3)]

        print('yaw is {}'.format(yaw))
        print('roll is {}'.format(roll))
        print('pitch is {}'.format(pitch))

        print('cg init pos is {}'.format(cg_init_pos))
        print('cg new pos is {}'.format(cg_new_pos))
        print('translation is {}'.format(translation))
        print('cam init pos is {}'.format(cam_init_pos))
        print('cam new pos is {}'.format(cam_new_pos))

        if self.row_num%2==0:
            heading = -yaw
            dl = (-cam_new_pos[1]-(self.row_num*self.lane_width))
            dr = self.lane_width-dl

        else:
            heading = 180-yaw
            dr = (-cam_new_pos[1]-(self.row_num*self.lane_width))
            dl = self.lane_width-dr

        self.heading_pub.publish(Float32MultiArray(data=[heading]))
        self.distance_pub.publish(Float32MultiArray(data=[dl,dr]))

        pts =  self.points.copy()[self.row_num]

        print('pts is {}'.format(pts))
        pts = np.reshape(pts,(4,3))
        pts_new = rot.inv().apply(pts)-np.array(translation)

        #transform to the frame convention of cameras 
        cam_rot = R.from_euler('zyx', [-90, 0, 90], degrees=True)
        pts_new_rot = cam_rot.apply(pts_new)

        projected_pts = self.project_points(pts_new_rot)
        la = projected_pts[1][1]-projected_pts[0][1]
        lb = -projected_pts[1][0]+projected_pts[0][0]
        lc = -projected_pts[0][0]*projected_pts[1][1]+projected_pts[1][0]*projected_pts[0][1]

        ra = projected_pts[3][1]-projected_pts[2][1]
        rb = -projected_pts[3][0]+projected_pts[2][0]
        rc = -projected_pts[2][0]*projected_pts[3][1]+projected_pts[3][0]*projected_pts[2][1]

        self.vpx = int((lb*rc-rb*lc)/(la*rb-ra*lb))
        self.vpy = int((ra*lc-la*rc)/(la*rb-ra*lb))

        l_int_y = self.height-1
        self.l_int_x = int((-lb*l_int_y-lc)/la)

        r_int_y = self.height-1
        self.r_int_x = int((-rb*r_int_y-rc)/ra)

    def kp_publisher(self,msg):

        np_arr = np.fromstring(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)         
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        image = cv2.circle(image, (self.vpx,self.vpy), 5, (255,0,0), -1)
        image = cv2.line(image,(self.vpx,self.vpy),(self.l_int_x,self.height-1),(0,255,0),2)
        image = cv2.line(image,(self.vpx,self.vpy),(self.r_int_x,self.height-1),(0,0,255),2)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        kp_gt_img = CompressedImage()
        kp_gt_img.header.stamp = rospy.Time.now()
        kp_gt_img.format = "jpeg"
        kp_gt_img.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        self.kp_gt_img.publish(kp_gt_img)


    def project_points(self,points):
        
        proj_points = []
        for point in points:
            z = point[2]
            x = self.fx*point[0]/z+self.px
            y = self.fx*point[1]/z+self.py
            proj_points.append([x,y])
        return proj_points
        
    def row_num_update(self, msg):
        self.row_num = msg.data

def main():
    rospy.init_node('rowfollow_gt_node')
    lane_width = 0.76
    lane_len = 21
    n_lanes = 7
    rowfollowgt = RowfollowGt(lane_width, lane_len, n_lanes)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        mylogerr("Shutting down ROS Image neural network prediction module")
 
        

if __name__ == '__main__':
    sys.exit(main() or 0)