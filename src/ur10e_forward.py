import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils import *
import numpy as np

def ur10e_fw(theta = [0,0,0,0,0,0]):
    # 'shoulder_pan_joint',
    # 'shoulder_lift_joint',
    # 'elbow_joint',
    # 'wrist_1_joint',
    # 'wrist_2_joint',
    # 'wrist_3_joint'
    
    # plotCoordinateSystem( ax, 0.1, 3.0 )
    
    shoulder_pan_translate = translate( 0.0, 0.0, 0.0 )
    shoulder_pan_rotation = rotateX(0)
    j0A = shoulder_pan_translate.dot(shoulder_pan_rotation)
    
    j1d = 0.1807
    j1a = 0
    j1T = DHH(theta[0],j1d,j1a,math.pi/2)
    
    j1A = j0A.dot(j1T)
    # plotCoordinateSystem( ax, 0.1, 2.0, j1A )
    # drawLink(ax, j0A, j1A, width=25 )
    
    j2d = 0
    j2a = -0.6127
    j2T = DHH(theta[1],j2d,j2a,0)
    
    j2A = j1A.dot(j2T)
    # plotCoordinateSystem( ax, 0.1, 2.0, j2A )
    # drawLink(ax, j1A, j2A, width=25 )

    j3d = 0
    j3a = -0.57155
    j3T = DHH(theta[2],j3d,j3a,0)
    
    j3A = j2A.dot(j3T)
    # plotCoordinateSystem( ax, 0.1, 2.0, j3A )
    # drawLink(ax, j2A, j3A, width=25 )
    
    j4d = 0.17415
    j4a = 0
    j4T = DHH(theta[3],j4d,j4a,math.pi/2)
    
    j4A = j3A.dot(j4T)
    # plotCoordinateSystem( ax, 0.1, 2.0, j4A )
    # drawLink(ax, j3A, j4A, width=25 )

    j5d = 0.11985
    j5a = 0
    j5T = DHH(theta[4],j5d,j5a,-math.pi/2)
    
    j5A = j4A.dot(j5T)
    # plotCoordinateSystem( ax, 0.1, 2.0, j5A )
    # drawLink(ax, j4A, j5A, width=25 )
    
    j6d = 0.11985 + 0.145 # gripper length
    j6a = 0
    j6T = DHH(theta[5],j6d,j6a,-math.pi/2)
    
    j6A = j5A.dot(j6T)
    # plotCoordinateSystem( ax, 0.1, 2.0, j6A )
    # drawLink(ax, j5A, j6A, width=25 )

    return j6A

def fwd_kinematics( theta ):
    end_effector_array = ur10e_fw(theta)
    end_effector = end_effector_array.dot([0.0, 0.0, 0.0, 1])[0:3]
    roll,pitch,yaw = rxyz(end_effector_array)
    e = np.array([end_effector[0],end_effector[1],end_effector[2],roll,pitch,yaw])
    return e

def Jacobian( theta, dt = 15.0/180.0*math.pi):
    J = np.zeros( (6, len(theta) ) )
    current_pos = fwd_kinematics( theta )
    # thetas_d = thetas.copy()
    
    for i,t in enumerate(theta):
        theta[i] = theta[i] + dt
        J[0:6,i] = ( fwd_kinematics(theta) - current_pos )
        theta[i] = theta[i] - dt
    return J


def IK(current_theta,target_pos, ACCEPTANCE = 0.0005):
    # new_theta = current_theta.copy()
    new_theta = analytical(target_pos)
    d_err = target_pos - fwd_kinematics( new_theta )  # fwd_kinematics is to calculate the End effector
    
    ### Momentum Parameter ###
    alpha = 1
    ##########################
    
    ### First Jacobian value ##
    jac = Jacobian( new_theta )
    d_theta = jac.T.dot(d_err)
    new_theta = new_theta + d_theta
    
    ### init the change theta (d_theta) to be first velocity ###
    velocity = d_theta.copy()
    ############################################################
    
    count = 1
    while(np.linalg.norm(d_err) > ACCEPTANCE ):
        ### Calculate Jacobian psedo code ###
        jac = Jacobian( new_theta )
        d_theta = jac.T.dot(d_err)
        new_theta = new_theta + d_theta
        #####################################
        if new_theta[2] > math.pi:
            new_theta[2] = math.pi
        if new_theta[4] > 2*math.pi/2:
            new_theta[4] = new_theta[4] - 4*math.pi/2
        if new_theta[4] < - 2*math.pi/2 :
            new_theta[4] = new_theta[4] + 4*math.pi/2 
            
        d_err = target_pos - fwd_kinematics( new_theta ) # d_err is error from [x,y,z,roll,pitch,yaw]
        
        ### Momentum method ###
        velocity = alpha * ( velocity + d_theta.copy() )
        Momentum = ( new_theta + velocity )
        if Momentum[2] > math.pi:
            Momentum[2] = math.pi
        if np.linalg.norm(target_pos - fwd_kinematics(Momentum )) < np.linalg.norm(d_err): # Apply momentum if momentum perform better
            new_theta = Momentum
        else: # Reset momentum to now change value If it's not better and don't apply momentum to new_theta
            velocity = d_theta.copy()
        #######################
        
        # for i in range(len(new_theta)):
            # new_theta = ((new_theta + math.pi) % 2*math.pi ) - math.pi
        
        count = count + 1
        if np.linalg.norm(d_err) < ACCEPTANCE:
            # if new_theta[2] < -math.pi:
            #     print("IK Fail!")
            #     return current_theta
            # print("IK Move! : ", count)
            # print(new_theta)
            pass
        if (count > 800 ):
            print("Fail to find the joint value !")
            # print("d_err",np.linalg.norm(d_err))
            # print("theta:",new_theta)
            # print("pos:",fwd_kinematics(new_theta))
            # return current_theta
            return new_theta
            
    return new_theta #, count , np.linalg.norm(d_err)


def analytical(target):
    x,y,z,roll,pitch,yaw = target
    theta = np.zeros(6) -math.pi/2
    theta[0] = math.atan2(-(y+0.4),-x)
    # if theta[0] < 0:
    #     theta[0] = theta[0] + 2 * math.pi

    angle_1 = (np.square(0.63)+ np.square(x)+np.square(y)+np.square(z) - np.square(0.58))/ (2*0.63*math.sqrt(np.square(x)+np.square(y)+np.square(z)))
    if angle_1 >= 1:
        angle_1 = 1
    if angle_1 <= -1:
        angle_1 = -1 
    theta[1] = -(math.atan2(z,math.sqrt(np.square(x)+np.square(y)))) - math.acos( angle_1 )
    angle_2 = (np.square(0.63) + np.square(0.58) - (np.square(x)+np.square(y)+np.square(z)) ) / (2*0.63*0.58)
    if angle_2 >= 1:
        angle_2 = 1
    if angle_2 <= -1:
        angle_2 = -1 
    theta[2] = math.pi - math.acos( angle_2 ) 
    theta[3] = -(theta[1] + theta[2]) -roll/2 #-math.pi/2
    theta[4] = 0#-yaw + theta[0] -roll/2
    theta[5] = -pitch 
    # print("Anal theta:",theta)
    return theta
        
if __name__ == "__main__":
                                                                                                                        
    # fig = plt.figure( figsize=(10,10) )
    # ax = fig.add_subplot(111, projection='3d')
    # ax.set_xlabel("X")
    # ax.set_ylabel("Y")
    # ax.set_zlabel("Z")
    
    # theta = [0.0,  # -2*pi and 2*pi
    #          -2.1794420321027985, # < -pi/2
    #          2.5389897297459263,  # -pi ~ pi
    #          -0.3596062374668304, # -2*pi and 2*pi
    #          1.5751876439055811,  # -2*pi and 2*pi
    #          0.00020853489193576092]
    
    init_theta = np.array([ 1.85389305e-01, -2.17944203e+00 , 2.53898973e+00, -3.59606237e-01, 1.85187644e-01 , 2.08534892e-04])
    # current_position = ur10e_fw([0,0,0,0,0,math.pi/2]).dot([0,0,0.1,1])[0:3]
    target_pos = np.array([-0.1593782806772388, -0.4550604286617699, 0.36242554949555406, 0,0,0])
    target_theta = IK(init_theta,target_pos)
    # print(current_position)
    
    # plt.show()
    