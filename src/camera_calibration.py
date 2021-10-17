#! /usr/bin/python3
import sys 
import os
import cv2

import rospy 
import rospkg
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from rosgraph_msgs.msg import Clock

import matplotlib.pyplot as plt
import matplotlib.image as img

import time
import numpy as np
import math
import copy
np.set_printoptions(threshold=np.inf)



bridge = CvBridge()

class Camera():
    def __init__(self):
        self.cv_img =None
        self.cv_depth =None
        rospack = rospkg.RosPack()
        PATH = rospack.get_path('robot_arm') + "/config/"
        self.clean_table_img = np.load(os.path.join(PATH, 'clean_table.npy'))
        rgb_sub =  message_filters.Subscriber("/CAM/camera/image", Image)
        depth_sub = message_filters.Subscriber("/CAM/range_finder/range_image", Image)
        visual_data = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1, allow_headerless=True)  # (ref)http://wiki.ros.org/message_filters
        visual_data.registerCallback(self.Show_VisualSensing_topics)
        
        while self.cv_img is None:
            time.sleep(0.1)
        while self.cv_depth is None:
            time.sleep(0.1)
        print("camera is ready")

    def Show_VisualSensing_topics(self,imgMsg, depthMsg):
        # (ref) https://cyberbotics.com/doc/reference/camera?tab-language=ros#camera-functions
        # (ref) https://cyberbotics.com/doc/reference/rangefinder?tab-language=ros#wb_range_finder_get_range_image
        
        try:
            self.cv_img = bridge.imgmsg_to_cv2(imgMsg, "passthrough")
            self.cv_depth = bridge.imgmsg_to_cv2(depthMsg, "passthrough")
        except CvBridgeError as e:
            print(e)
        
        # cv2.normalize(self.cv_depth, self.cv_depth, 0, 1, cv2.NORM_MINMAX)  # (ref) https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/	

        # cv2.imshow("webots_camera_view", self.cv_img)
        # cv2.imshow("webots_depth_view", self.cv_depth)
        # cv2.setMouseCallback("webots_depth_view", self.on_EVENT_LBUTTONDOWN)
        # cv2.waitKey(1)	

    def object_pos(self,boundering): # boundering = [[a,b],[a',b']]
        # width_wide_pixel = np.radians(32)/320
        # use deep learn algorithm to predict object bounding box value ((a,b),(a',b')) from pic
        # find object depth by using mean value, then get x value from world coordination
            # use  delta_img = np.abs(cal_img - cal_img(with clean table) )
        cal_img = self.depth_calibration()
        height,width = self.cv_depth.shape[0:2]
        black_img = np.zeros((height,width))
        delta_img = np.abs(cal_img - self.clean_table_img)
        black_img[np.where(delta_img > 0.01)] = cal_img[np.where(delta_img > 0.01)]
        x = 1.35272 - np.mean(cal_img [ np.where( delta_img[boundering[0,0]:boundering[1,0,boundering[0,1]:,boundering[1,1]]] > 0.01 ) ])
        
        # find object pos by using predict center value (mean(a),mean(b)) -> (ma,mb)
        # and mean value of depth (x) , then get z value from world coordination
        m = abs(x - 0.22)
        n = abs(0.93 - x)
        pix_x = (m*310 + n*145)/(m+n)
        a = abs(pix_x -145)
        b = abs(310 - pix_x)
        pix_z = (a*550 + b*330)/(a+b)
        middle_z = (boundering[0,1]+boundering[1,1])/2
        z = (1.2/pix_z) * (-(middle_z-320))
        return x,z
        
    def depth_calibration(self):
        height_wide_pixel = 1.8*np.radians(24)/240
        height,width = self.cv_depth.shape[0:2]
        cal_depth = np.zeros((height,width))
        for y in range(height):
            for x in range(width):
                # cal_depth[y,x] = depth_img[y,x] * math.cos( abs(y-(height/2)) * height_wide_pixel ) * math.cos( abs(x-(height/2)) * width_wide_pixel )
                cal_depth[y,x] = 1.03 * ( self.cv_depth[y,x] * math.cos( abs(y) * height_wide_pixel ) )
                # cal_depth[y,x] = 1.35272 - cal_depth[y,x] # translate to world coordination
        return cal_depth

if __name__ == '__main__':
    _node_name = 'camera_node'
    rospy.init_node(_node_name)
    rospy.loginfo('{0} is up!'.format(_node_name))
    
    cam = Camera()
        
    img = cam.depth_calibration()
    # clean_table_img = copy.deepcopy(cam.clean_table_img)
    plt.title("cal_img")
    plt.imshow(img,cmap='gray')
    plt.show()
    
    # print(np.shape(img))
    # cv2.namedWindow("image")
    # cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
    # cv2.imshow("image", img)
    # cv2.waitKey(0)	
    
    
    # np.save(os.path.join(PATH, 'clean_table'), np.array(img))

    
    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down...")
        
    # cv2.destroyAllWindows()