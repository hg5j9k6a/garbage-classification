#! /usr/bin/env python3

import rospy
import time
import math
import copy
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class camera_calibration():
    def __init__(self):
        self.img = None
        self.bridge = CvBridge()
        rospy.Subscriber("CAM/camera/image",Image,self._camera_img_cb)
        
        while self.img is None:
            time.sleep(0.03)
            
        print("camera_node is ready!")
        print(np.shape(self.img))
        # print(self.img)
        
    def _camera_img_cb(self,msg):
        # cv_image = self.bridge.imgmsg_to_cv2(msg.data, desired_encoding='passthrough')
        img = np.array(bytearray(msg.data))
        self.img_height = msg.height
        self.img_width = msg.width
        self.img = np.reshape(img,(self.img_height,self.img_width,4))
        
        # print(self.img_height,self.img_width)
        
        
if __name__ == '__main__':
    _node_name = 'camera_node'
    rospy.init_node(_node_name)
    rospy.loginfo('{0} is up!'.format(_node_name))
    cam = camera_calibration()
    while True:
        img = copy.deepcopy(cam.img)
        # depth = img[:,:,0]
        # img = np.zeros((256,256,3))
        cv2.imshow('My Image', img[:,:,0:3])
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    rospy.spin()