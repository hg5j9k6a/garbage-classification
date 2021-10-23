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
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
import matplotlib.image as img

import time
import numpy as np
import math
from copy import deepcopy
np.set_printoptions(threshold=np.inf)



bridge = CvBridge()

class Camera():
    def __init__(self):
        self.cv_img =None
        self.cv_depth =None
        self.detect_info = None
        rospack = rospkg.RosPack()
        PATH = rospack.get_path('robot_arm') + "/config/"
        self.clean_table_img = np.load(os.path.join(PATH, 'clean_table.npy'))
        # self.RGB_pub = rospy.Publisher("/camera/image",Image,queue_size=10)
        # self.img_date = cv2.imread(os.path.join(PATH, '20211013072806.jpg'))
        # print(np.shape(self.img_date))
        rgb_sub =  message_filters.Subscriber("/CAM/camera/image", Image)
        depth_sub = message_filters.Subscriber("/CAM/range_finder/range_image", Image)
        detection_sub = rospy.Subscriber("/object/data",Float32MultiArray,self._object_data_cb)
        
        visual_data = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1, allow_headerless=True)  # (ref)http://wiki.ros.org/message_filters
        visual_data.registerCallback(self.Show_VisualSensing_topics)
        
        while self.cv_img is None:
            time.sleep(0.1)
        while self.cv_depth is None:
            time.sleep(0.1)
        
        # print("camera is ready")

    def Show_VisualSensing_topics(self,imgMsg, depthMsg):
        # (ref) https://cyberbotics.com/doc/reference/camera?tab-language=ros#camera-functions
        # (ref) https://cyberbotics.com/doc/reference/rangefinder?tab-language=ros#wb_range_finder_get_range_image
        
        try:
            self.cv_img = bridge.imgmsg_to_cv2(imgMsg, "passthrough")
            self.cv_depth = bridge.imgmsg_to_cv2(depthMsg, "passthrough")
            
            # self.RGB_pub.publish( bridge.cv2_to_imgmsg(cv2.cvtColor(self.cv_img,cv2.COLOR_BGR2RGB),"bgr8"))
            # self.RGB_pub.publish( bridge.cv2_to_imgmsg(cv2.cvtColor(self.img_date,cv2.COLOR_BGR2RGB),"bgr8"))
            
        except CvBridgeError as e:
            print(e)
        
        # cv2.normalize(self.cv_depth, self.cv_depth, 0, 1, cv2.NORM_MINMAX)  # (ref) https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/	

        # cv2.imshow("webots_camera_view", self.cv_img)
        # cv2.imshow("webots_depth_view", self.cv_depth)
        # cv2.setMouseCallback("webots_depth_view", self.on_EVENT_LBUTTONDOWN)
        # cv2.waitKey(1)	
    def _object_data_cb(self,dataMsg):
        # if self.cv_depth is not None:
        self.detect_data = np.array(dataMsg.data)
        
    
    def detect_object_pose(self):
        detect_data = deepcopy(self.detect_data)
        detect_data = np.reshape(detect_data,(len(detect_data)//6,6))
        object_data = []
        object_order = []
        cal_img = self.depth_calibration()
        for i in detect_data:
            boundering= np.array([[i[1],i[0]],[i[3],i[2]]]) 
            x,z = self.object_pos(boundering,cal_img)
            garbage_name,classification = self.determind_class(i)
            object_data.append([x,z,garbage_name,classification])
        for j in object_data:
            object_order.append( math.sqrt( np.square(j[0] - 0.92) + np.square(j[1] - 0.62) ) )
        self.detect_info = deepcopy(object_data)
        # print(self.detect_info)
        # print("return : ",object_data[np.argmin(object_order)])
        return object_data[np.argmin(object_order)]
    
    def determind_class(self,array):
        data = array[5]
        if data == 0:
            return "coke_bottle","glass_box"
        elif data == 1:
            return "coke_can","can_box"
        elif data == 2:
            return "milk_2L","paper_box"
        elif data == 3:
            return "milk_250ml","paper_box"
        elif data == 4:
            return "pain_killer","palstic_box"
        elif data == 5:
            return "chip_bag","palstic_bag_box"
        elif data == 6:
            return "tissue","paper_box"
        elif data == 7:
            return "water_bottle","palstic_box"
        elif data == 8:
            return "wine_bottle","glass_box"
        elif data == 9:
            return "beer_bottle","glass_box"
        elif data == 10:
            return "beer_can","can_box"
        elif data == 11:
            return "coffee_can","can_box"
        
    def object_pos(self,boundering,cal_img): # boundering = [[a,b],[a',b']]
        a = int(boundering[0,0])
        b = int(boundering[0,1])
        _a = int(boundering[1,0])
        _b = int(boundering[1,1])
        
        # width_wide_pixel = np.radians(32)/320
        # use deep learn algorithm to predict object bounding box value ((a,b),(a',b')) from pic
        # find object depth by using mean value, then get x value from world coordination
        
        height,width = self.cv_depth.shape[0:2]
        black_img = np.zeros((height,width))
        
        delta_img = np.abs(cal_img - self.clean_table_img)
        black_img[np.where(delta_img > 0.01)] = cal_img[np.where(delta_img > 0.01)]
        # print(boundering[0,0],boundering[1,0])
        # x_img = cal_img [ np.where( delta_img[a:_a,b:_b] > 0.02 ) ]
        
        point_depth = []
        for i in np.arange(a,_a+1):
            for j in np.arange(b,_b+1):
                if black_img[i,j] > 0.01:
                    point_depth.append(black_img[i,j])
        
        ## filter
        point_depth = np.sort(point_depth)
        mean_depth = np.mean(point_depth[int(len(point_depth) * 0.4):int(len(point_depth) * 0.75)])
        # plt.title("x_img")
        # plt.imshow(black_img[a:_a,b:_b],cmap='gray')
        # plt.show()
        x = (1.35272 - mean_depth)
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
            cal_depth[y] = 1.03 * ( self.cv_depth[y] * math.cos( abs(y) * height_wide_pixel ) )
            
            # for x in range(width):
                # cal_depth[y,x] = depth_img[y,x] * math.cos( abs(y-(height/2)) * height_wide_pixel ) * math.cos( abs(x-(height/2)) * width_wide_pixel )
                # cal_depth[y,x] = 1.03 * ( self.cv_depth[y,x] * math.cos( abs(y) * height_wide_pixel ) )
                # cal_depth[y,x] = 1.35272 - cal_depth[y,x] # translate to world coordination

        return cal_depth
    
        

if __name__ == '__main__':
    _node_name = 'camera_node'
    rospy.init_node(_node_name)
    rospy.loginfo('{0} is up!'.format(_node_name))
    
    cam = Camera()
    # start_time = time.time()
    # print(cam.detect_object_pose())
    # print("time:",time.time()- start_time)
    
    img = cam.depth_calibration()
    clean_table_img = deepcopy(cam.clean_table_img)

    plt.title("cal_img")
    plt.imshow(img,cmap='gray')
    plt.show()

    boundering= np.array([[200,151],[250,200]]) 
    # boundering= np.array([[230,135],[280,172]]) 
    # boundering= np.array([[148,189],[180,210]]) 
    # boundering= np.array([[240,250],[280,285]])  # [img[y,x] -> img[y',x']]
    cal_img = cam.depth_calibration()
    
    print(cam.object_pos(boundering,cal_img))
    
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