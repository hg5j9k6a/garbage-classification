#! /usr/bin/python3
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from copy import deepcopy
from object_pos import object_pos

rospy.init_node("node_that_moves_arm_quickly")
pub = rospy.Publisher("/move_ik_arm", Twist, queue_size=2)
gripper_pub = rospy.Publisher("/move_grip", Float32, queue_size=2)

def do_things(target_pos,target_dic,box_dic,tissue = False):
    t_x = target_pos[0]
    t_z = target_pos[1]
    
    t_y = target_dic["y"]
    if t_x < 0.5:
        t_yaw = 0.0
    else:
        t_yaw = target_dic["yaw"]

    # Trash can
    tbox_x = box_dic["x"]
    tbox_y = box_dic["y"]
    tbox_z = box_dic["z"]
    tbox_roll = box_dic["roll"]
    tbox_pitch = box_dic["pitch"]
    tbox_yaw = box_dic["yaw"]

    gripper_pub.publish(Float32(0.0))
    time.sleep(1)

    targets = []
    msg = Twist()
    msg.linear.x = 0.2
    msg.linear.y = 1.2
    msg.linear.z = 0.5
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = -1.57
    targets.append(deepcopy(msg))

    msg.linear.x = t_x #- 0.1
    msg.linear.z = t_z 
    msg.angular.x = target_dic["roll"]
    msg.angular.z = t_yaw
    targets.append(deepcopy(msg))

    msg.linear.x = t_x #- 0.1
    msg.linear.y = 0.95
    targets.append(deepcopy(msg))

    msg.linear.x = t_x #- 0.02
    msg.linear.y = t_y
    targets.append(deepcopy(msg))

    # Go to target
    for m in targets:
        pub.publish(m)
        time.sleep(0.8)
        rospy.loginfo("Message published!")

    # Grab the target.
    time.sleep(0.5)
    for i in range(3):
        gripper_pub.publish(Float32(target_dic["grip"]))
        time.sleep(0.01)
    time.sleep(0.5)
        
    # move up
    msg.linear.y = 1.2
    msg.angular.x = 0.0
    pub.publish(msg)
    time.sleep(0.5)
    
    # Go to trash box.
    targets = []
    msg.linear.x = tbox_x/2
    msg.linear.y = tbox_y
    msg.linear.z = tbox_z
    msg.angular.x = tbox_roll
    msg.angular.y = tbox_pitch
    msg.angular.z = tbox_yaw
    targets.append(deepcopy(msg))
    msg.linear.x = tbox_x
    targets.append(deepcopy(msg))

    for m in targets:
        pub.publish(m)
        time.sleep(0.8)
        rospy.loginfo("Message published!")
    for i in range(3):
        gripper_pub.publish(Float32(0.0))
        time.sleep(0.01)
    time.sleep(1)
    if tissue:
        msg.angular.x = target_dic["roll"]
        pub.publish(msg)
        time.sleep(0.5)
        msg.linear.y = 0.75
        pub.publish(msg)
        time.sleep(0.5)
        msg.linear.y = 1.1
        pub.publish(msg)
        time.sleep(0.25)
        msg.linear.y = 0.75
        pub.publish(msg)
        time.sleep(0.25)
        msg.linear.y = 1.1
        pub.publish(msg)
        time.sleep(0.25)
    else:
        msg.linear.x = 0.2
        msg.linear.y = 1.2
        msg.linear.z = 0.5
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -1.57
        pub.publish(msg)
        time.sleep(0.5)
        
if __name__ == '__main__':
    op = object_pos()
    x ,z = (0.7885217083801764, 0.12446616459097752)
    
    do_things((x,z),op.milk_250ml,op.paper_box,False)
