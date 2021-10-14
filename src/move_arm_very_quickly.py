import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from time import sleep
from copy import deepcopy

rospy.init_node("node_that_moves_arm_quickly")
pub = rospy.Publisher("/move_ik_arm", Twist, queue_size=1)
gripper_pub = rospy.Publisher("/move_grip", Float32, queue_size=1)

def do_things(target):
    t_x = target[0]
    t_y = target[1]
    t_z = target[2]
    # Trash can
    tc_x = -0.4
    tc_y = 1.0
    tc_z = 0.8

    gripper_pub.publish(Float32(0.0))
    sleep(1)

    targets = []
    msg = Twist()
    msg.linear.x = 0.2
    msg.linear.y = 1.2
    msg.linear.z = 0.5
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = -1.57
    targets.append(deepcopy(msg))

    msg.linear.x = t_x - 0.1
    msg.linear.z = t_z 
    targets.append(deepcopy(msg))

    msg.linear.x = t_x - 0.1
    msg.linear.y = 0.9
    targets.append(deepcopy(msg))

    msg.linear.x = t_x - 0.02
    msg.linear.y = t_y
    targets.append(deepcopy(msg))

    # Go to target
    for m in targets:
        pub.publish(m)
        sleep(0.5)
        rospy.loginfo("Message published!")

    # Grab the target.
    gripper_pub.publish(Float32(75.0))
    sleep(1)

    # Go to trash can.
    targets = []
    msg.linear.y = tc_y
    targets.append(deepcopy(msg))

    msg.linear.x = tc_x
    msg.linear.y = tc_y
    msg.linear.z = tc_z
    targets.append(deepcopy(msg))

    for m in targets:
        pub.publish(m)
        sleep(1)
        rospy.loginfo("Message published!")

    gripper_pub.publish(Float32(0.0))


#do_things((0.3, 0.744191, -0.497339))
#do_things((0.567898, 0.753014, -0.495423))
#do_things((0.5698586, 0.7684, -0.2483))
do_things((0.724, 0.747, 0.2586))
