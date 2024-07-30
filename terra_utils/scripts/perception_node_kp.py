#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
import logging as log
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import models
from scipy.special import softmax
import argparse 
import sys
import os
import message_filters



def myloginfo(msg=''):
    rospy.loginfo('['+str(rospy.get_name() + '] ' + str(msg)))

def mylogwarn(msg=''):
    rospy.logwarn('['+str(rospy.get_name() + '] ' + str(msg)))

def mylogerr(msg=''):
    rospy.logerr('['+str(rospy.get_name() + '] ' + str(msg)))

class Up(nn.Module):
    def __init__(self, in_channels, out_channels, scale_factor=2):
        super().__init__()

        self.up = nn.Upsample(scale_factor=scale_factor, mode='bilinear',
                              align_corners=True)

        self.conv = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

    def forward(self, x1, x2):
        x1 = self.up(x1)
        x1 = torch.cat([x2, x1], dim=1)
        return self.conv(x1)


class RowFollowNet(nn.Module):
    def __init__(self):
        super(RowFollowNet, self).__init__()        
            
        resnet18 = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        encoder = nn.Sequential(*list(resnet18.children())[:8])
        
        self.conv1 = encoder[0]
        self.bn1 = encoder[1]
        self.relu = encoder[2]
        self.maxpool = encoder[3]
        
        self.layer1 = encoder[4]
        self.layer2 = encoder[5]
        self.layer3 = encoder[6]
        self.layer4 = encoder[7]


        self.up1 = Up(512+256, 256, scale_factor=2)
        self.up2 = Up(256+128,128, scale_factor=2)
        self.up3 = Up(128+64, 64)
          
        self.head = nn.Conv2d(64,3, kernel_size=1, padding=0)
        
    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x1 = self.maxpool(x)
        
        x2 = self.layer1(x1)
        x3 = self.layer2(x2)
        x4 = self.layer3(x3)
        x5 = self.layer4(x4)

        x_up = self.up1(x5, x4)
        x_up = self.up2(x_up,x3)
        x_up = self.up3(x_up,x2)
        x = self.head(x_up)

        return x
  
    
class Perception:

    def __init__(self,model,device):

        self.model = model
        self.device = device
        self.heatmaps_scale = 4
        
        self.publishers()
        self.subscribers()

    def subscribers(self):        
        left_img_topic = '/terrasentia/zed2/left/image_rect_color/compressed'
        right_img_topic = '/terrasentia/zed2/right/image_rect_color/compressed'
        left_cam_sub = message_filters.Subscriber(left_img_topic, CompressedImage)
        right_cam_sub = message_filters.Subscriber(right_img_topic, CompressedImage)

        # Create a time synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([left_cam_sub, right_cam_sub], queue_size=1, slop=0.5)

        # Register the callback to the synchronizer
        ts.registerCallback(self.stereocam_callback)

    def publishers(self):
        # Output image topics for left camera
        self.keypoint_heatmap_pub_left = rospy.Publisher('/terrasentia/vision/left/keypoint_heatmap', Float32MultiArray, queue_size=1)
        self.keypoint_heatmap_norm_pub_left = rospy.Publisher('/terrasentia/vision/left/keypoint_heatmap_normalized/compressed', CompressedImage, queue_size=1)
        self.keypoint_vis_heatmap_pub_left = rospy.Publisher('/terrasentia/vision/left/keypoint_vis_heatmap/compressed', CompressedImage, queue_size=1)
        self.keypoint_vis_argmax_pub_left = rospy.Publisher('/terrasentia/vision/left/keypoint_vis_argmax/compressed', CompressedImage, queue_size=1)

        # Output image topics for right camera
        self.keypoint_heatmap_pub_right = rospy.Publisher('/terrasentia/vision/right/keypoint_heatmap', Float32MultiArray, queue_size=1)
        self.keypoint_heatmap_norm_pub_right = rospy.Publisher('/terrasentia/vision/right/keypoint_heatmap_normalized/compressed', CompressedImage, queue_size=1)
        self.keypoint_vis_heatmap_pub_right = rospy.Publisher('/terrasentia/vision/right/keypoint_vis_heatmap/compressed', CompressedImage, queue_size=1)
        self.keypoint_vis_argmax_pub_right = rospy.Publisher('/terrasentia/vision/right/keypoint_vis_argmax/compressed', CompressedImage, queue_size=1)


    def Image_processing(self,ros_data):

        np_arr = np.frombuffer(ros_data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
       
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image[24:696,160:1120]  
        image = cv2.resize(image,[320,224])

        data = image/255
        data[:,:,0] = (data[:,:,0]-0.485)/0.229
        data[:,:,1] = (data[:,:,1]-0.456)/0.224
        data[:,:,2] = (data[:,:,2]-0.406)/0.225
        data = np.expand_dims(data,axis=0)
        data = np.transpose(data, axes=[0,3,1,2]).astype(np.float32)
    
        return data,image
    

    def stereocam_callback(self, left_data, right_data):

        left_tensor, left_image = self.Image_processing(left_data)
        right_tensor, right_image = self.Image_processing(right_data)
        combined_tensor = np.concatenate((left_tensor, right_tensor))

        pred = self.model(torch.from_numpy(combined_tensor).to(self.device)).detach().cpu().numpy()
        pred_left = pred[0]
        pred_right = pred[1]
        
        keypoint_heatmap_msg_left,keypoint_heatmap_norm_msg_left,keypoint_vis_heatmap_msg_left,keypoint_vis_argmax_msg_left = self.process_output(pred_left,left_image)
        keypoint_heatmap_msg_right,keypoint_heatmap_norm_msg_right,keypoint_vis_heatmap_msg_right,keypoint_vis_argmax_msg_right = self.process_output(pred_right,right_image)

        self.keypoint_heatmap_pub_left.publish(keypoint_heatmap_msg_left)
        self.keypoint_vis_heatmap_pub_left.publish(keypoint_vis_heatmap_msg_left)
        self.keypoint_vis_argmax_pub_left.publish(keypoint_vis_argmax_msg_left)
        self.keypoint_heatmap_norm_pub_left.publish(keypoint_heatmap_norm_msg_left)

        self.keypoint_heatmap_pub_right.publish(keypoint_heatmap_msg_right)
        self.keypoint_vis_heatmap_pub_right.publish(keypoint_vis_heatmap_msg_right)
        self.keypoint_vis_argmax_pub_right.publish(keypoint_vis_argmax_msg_right)
        self.keypoint_heatmap_norm_pub_right.publish(keypoint_heatmap_norm_msg_right)

    def process_output(self,pred,image):
        keypoint_heatmap_msg = Float32MultiArray()

        channels, height, width = pred.shape

        # Create layout for the Float32MultiArray
        layout = MultiArrayLayout()
        layout.dim.append(MultiArrayDimension())
        layout.dim.append(MultiArrayDimension())
        layout.dim.append(MultiArrayDimension())

        # Set the layout dimensions
        layout.dim[0].label = "height"
        layout.dim[0].size = height
        layout.dim[0].stride = width * channels

        layout.dim[1].label = "width"
        layout.dim[1].size = width
        layout.dim[1].stride = channels

        layout.dim[2].label = "channels"
        layout.dim[2].size = channels
        layout.dim[2].stride = 1

        # Assign layout to the message
        keypoint_heatmap_msg.layout = layout

        # Assign the image data to the message
        keypoint_heatmap_msg.data = pred.flatten().tolist()

        pred[0] = softmax(pred[0])
        pred[1] = softmax(pred[1])
        pred[2] = softmax(pred[2])

        pred_vis = 0.5*cv2.cvtColor(image, cv2.COLOR_BGR2RGB)+0.5*(cv2.resize(self.normalize(pred),image.shape[:2][::-1])*255)

        keypoint_heatmap_norm_msg = CompressedImage()
        keypoint_heatmap_norm_msg.header.stamp = rospy.Time.now()
        keypoint_heatmap_norm_msg.format = "jpeg"
        keypoint_heatmap_norm_msg.data = np.array(cv2.imencode('.jpg', cv2.resize(self.normalize(pred),image.shape[:2][::-1])*255)[1]).tobytes()
        
        keypoint_vis_heatmap_msg = CompressedImage()
        keypoint_vis_heatmap_msg.header.stamp = rospy.Time.now()
        keypoint_vis_heatmap_msg.format = "jpeg"
        keypoint_vis_heatmap_msg.data = np.array(cv2.imencode('.jpg', pred_vis)[1]).tobytes()

    
        vp_y,vp_x = np.unravel_index(pred[0].argmax(), pred[0].shape)
        ll_y,ll_x = np.unravel_index(pred[1].argmax(), pred[1].shape)
        lr_y, lr_x = np.unravel_index(pred[2].argmax(), pred[2].shape)

        argmax_img = cv2.circle(image, (vp_x*self.heatmaps_scale,vp_y*self.heatmaps_scale), 5, (255,0,0), -1)
        argmax_img = cv2.line(argmax_img,(vp_x*self.heatmaps_scale,vp_y*self.heatmaps_scale),(ll_x*self.heatmaps_scale,ll_y*self.heatmaps_scale),(0,255,0),2)
        argmax_img = cv2.line(argmax_img,(vp_x*self.heatmaps_scale,vp_y*self.heatmaps_scale),(lr_x*self.heatmaps_scale,lr_y*self.heatmaps_scale),(0,0,255),2)
        argmax_img = cv2.cvtColor(argmax_img, cv2.COLOR_RGB2BGR)

        keypoint_vis_argmax_msg = CompressedImage()
        keypoint_vis_argmax_msg.header.stamp = rospy.Time.now()
        keypoint_vis_argmax_msg.format = "jpeg"
        keypoint_vis_argmax_msg.data = np.array(cv2.imencode('.jpg', argmax_img)[1]).tobytes()

        return keypoint_heatmap_msg,keypoint_heatmap_norm_msg,keypoint_vis_heatmap_msg,keypoint_vis_argmax_msg

      
    
    def normalize(self,input):
        #input is c,h,w which we change to w,h,c
        input = input.transpose((1,2,0))

        output = input.copy()

        r =  input[:,:,0]
        g =  input[:,:,1]
        b = input[:,:,2]

        r_min = r.min()
        g_min = g.min()
        b_min = b.min()

        r_max = r.max()
        g_max = g.max()
        b_max = b.max()

        output[:,:,0] = (input[:,:,2]-b_min)/(b_max-b_min) 
        output[:,:,1] = (input[:,:,1]-g_min)/(g_max-g_min)
        output[:,:,2] = (input[:,:,0]-r_min)/(r_max-r_min)
        
        output = np.where(output > 0.1, output, np.zeros_like(output))

        return output
    
    
def main():
    rospy.init_node('vision_perception', anonymous=True)

    '''Initializes and cleanup ros node'''
    log.basicConfig(format="[ %(levelname)s ] %(message)s", level=log.INFO, stream=sys.stdout)
    model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),'models/kp.ckpt')
   
    checkpoint = torch.load(model_path)['state_dict']
    ckpt = checkpoint.copy()
    del_key_list = ['cf_encoder_conv.weight', 'cf_encoder_conv.bias', 'cf_encoder_fc.weight', 'cf_encoder_fc.bias']
    for key in checkpoint.keys():
        if key in del_key_list:
            del ckpt[key]

    device = torch.device("cuda:0")

    model = RowFollowNet()
    model.load_state_dict(ckpt)
    model.to(device)
    model.eval()
    
    inference_node = Perception(model,device)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        mylogerr("Shutting down ROS Image neural network prediction module")
 

if __name__ == '__main__':
    sys.exit(main() or 0)
