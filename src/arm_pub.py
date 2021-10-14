#! /usr/bin/env python3

import rospy
import time
import math
import numpy as np

from ur10e_forward import *
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from robot_arm.srv import ComputeKinematicPose,ComputeKinematicPoseResponse
from std_msgs.msg import Float32,Bool

class ur10_pub():
    
    def __init__(self):
        self.data =  FollowJointTrajectoryActionGoal()
        self.grip_data = FollowJointTrajectoryActionGoal()
        self.current_joint = None
        self.current_grip_joint = None
        self.target_theta = None
        self.joint_sub = rospy.Subscriber('/joint_states',JointState,self._joint_cb)
        self.grip_joint_sub = rospy.Subscriber('//gripper_joint_states',JointState,self._grip_joint_cb)
        self.move_arm_topice = rospy.Subscriber('/move_ik_arm',Twist,self._move_arm_cb)
        self.move_grip = rospy.Subscriber('/move_grip',Float32,self._move_grip_cb)
        while (self.current_joint is None) or (self.current_grip_joint is None):
            time.sleep(0.1)
        
        # print("init_theta : ",self.current_joint)
        
        self.jointNames = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.data.goal.trajectory.joint_names = self.jointNames
        self.data.goal.trajectory.points = [JointTrajectoryPoint(),JointTrajectoryPoint()]
        self.data.goal.trajectory.points[0].velocities =[ 0 for i in range(len(self.jointNames))]
        self.data.goal.trajectory.points[1].velocities =[ 0 for i in range(len(self.jointNames))]
        self.publisher = rospy.Publisher('/follow_joint_trajectory/goal',FollowJointTrajectoryActionGoal, queue_size=5 )
        self.grip_publisher = rospy.Publisher('/grp_follow_joint_trajectory/goal',FollowJointTrajectoryActionGoal, queue_size=5 )
        self.get_pose = rospy.Service("/get_kinematic_pos",ComputeKinematicPose,self.compute_pose)
        
        self.grip_joint_name = ["finger_1_joint_1",
                                "finger_2_joint_1",
                                "finger_middle_joint_1"]
        self.grip_data.goal.trajectory.joint_names = self.grip_joint_name
        self.grip_data.goal.trajectory.points = [JointTrajectoryPoint(),JointTrajectoryPoint()]
        self.grip_data.goal.trajectory.points[0].velocities =[ 0 for i in range(len(self.grip_joint_name))]
        self.grip_data.goal.trajectory.points[1].velocities =[ 0 for i in range(len(self.grip_joint_name))]
        
        self.move_done = rospy.Publisher('/move_done',Bool, queue_size=2 )
        time.sleep(1)
        self.pub(np.array([-0.2,-0.2,0.58,0,0,-1.57]))
        
            
    def pub(self, target_pos): # target_pos = [x,y,z,roll,pitch,yaw] units in m and radians. pitch should be 0, or it wont find the answer by some unknow reason.
        self.data.goal.trajectory.points[0].positions = self.current_joint
        self.target_theta = IK(self.current_joint,target_pos)
        self.data.goal.trajectory.points[1].positions = [ t for t in self.target_theta]
        for i in range(5):
            self.publisher.publish(self.data)
            time.sleep(0.01)
    
    def pub_theta(self,theta):
        self.data.goal.trajectory.points[0].positions = self.current_joint
        self.data.goal.trajectory.points[1].positions = [ t for t in theta]
        for i in range(5):
            self.publisher.publish(self.data)
            time.sleep(0.01)
            
    def grip_pub(self,msg):
        self.grip_data.goal.trajectory.points[0].positions = self.current_grip_joint
        value = np.interp(msg,[i for i in np.linspace(0,100,100)],[j for j in np.linspace(0.05,1.2,100)])
        self.grip_data.goal.trajectory.points[1].positions = [ value for i in range(len(self.grip_joint_name))]
        for i in range(5):
            self.grip_publisher.publish(self.grip_data)
            time.sleep(0.01)
                
        
    def compute_pose(self,req):
        if req.t :
            return ComputeKinematicPoseResponse(fwd_kinematics(self.current_joint))

    def _joint_cb(self,msgs):
        self.current_joint = np.array(msgs.position)
        if self.target_theta is not None:
            if np.linalg.norm(self.current_joint - self.target_theta) < 0.05:
                self.move_done.publish(True)
            else:
                self.move_done.publish(False)
            
    def _grip_joint_cb(self,msgs):
        self.current_grip_joint = np.array(msgs.position)
        
    def _move_arm_cb(self,msgs):
        x = -msgs.linear.x
        y = msgs.linear.z
        z = msgs.linear.y - 0.7
        roll = msgs.angular.x
        pitch = msgs.angular.y
        yaw = msgs.angular.z
        target = np.array([x,y,z,roll,pitch,yaw]])
        print(target)
        self.pub(target)
        
    def _move_grip_cb(self,msgs):
        self.grip_pub(msgs.data )
        
            
if __name__ == '__main__':
    _node_name = 'robot_pub'
    rospy.init_node(_node_name)
    rospy.loginfo('{0} is up!'.format(_node_name))
    
    arm = ur10_pub()
    
    rospy.loginfo('IK READY!')
    
    # arm.pub(np.array([-0.2,-0.2,0.58,0,0,0]))
    # final joint [-1.286046663676204, -2.382987771690268, 2.2111795197108273]
    
    #Analy [0.36400542234189764, -0.25683430003406665, 0.8863516947600655]
    # arm.pub_theta([-0.78539816+2*math.pi ,-2.06107933 , 2.01931852, 0.04176081,-0.78539816,0])
    # arm.pub_theta([7.85398163e-01 ,-2.55606533e+00 , 1.64454607e+00 , 1.26519259e-01,  3.98163397e-04, -0.00000000e+00])
    # arm.pub_theta([7.85398163e-01 ,-2.55606533e+00 , 1.64454607e+00 , 1.26519259e-01,  3.98163397e-04, -0.00000000e+00])
    
    # arm.pub([0.0,  # pi and pi
    #          -2.1794420321027985, # < -pi/2
    #          2.5389897297459263,  # -pi ~ pi
    #          -0.3596062374668304, # -2*pi and 2*pi
    #          1.5751876439055811,  # -2*pi and 2*pi
    #          0.00020853489193576092]) # -2*pi and 2*pi
    # # print("PUB!")
    rospy.spin()
    
