#! /usr/bin/python3
import sys 
import os

import rospy 
import rospkg

import matplotlib.pyplot as plt
import matplotlib.image as img

import time
import numpy as np
import math
from copy import deepcopy
np.set_printoptions(threshold=np.inf)

from arm_pub import *
from camera_calibration import *
from move_arm_very_quickly import *
from object_pos import object_pos

if __name__ == "__main__":
    _node_name = 'robot_main'
    rospy.init_node(_node_name)
    rospy.loginfo('{0} is up!'.format(_node_name))
    op = object_pos()
    arm = ur10_pub()
    rospy.loginfo('IK READY!')
    cam = Camera()
    rospy.loginfo('Camera READY!')
    
    grab_object = cam.detect_object_pose()
    target = [grab_object[0],grab_object[1]]
    
    while grab_object is not None:
        print("target:",grab_object)
        if grab_object[2] == "chip_bag":
            do_things(target,eval("op."+grab_object[2]),eval("op."+grab_object[3]),True)
        else:
            do_things(target,eval("op."+grab_object[2]),eval("op."+grab_object[3]))
        grab_object = cam.detect_object_pose()
        target = [grab_object[0],grab_object[1]]
        
    rospy.spin()