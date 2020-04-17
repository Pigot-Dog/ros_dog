#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
import kinematics_algorithm as ka
import math
from sensor_msgs.msg import Imu
import rpy_algorithm as rpy

rpy_angle = [0, 0, 0]

def action_pub(gait_data, data_length):
    j = 0
    while (j<1):
        for i in range(data_length):
            joint1_pos_pub.publish(gait_data.data[i, 0])
            joint2_pos_pub.publish(gait_data.data[i, 1])
            joint3_pos_pub.publish(gait_data.data[i, 2])
            joint4_pos_pub.publish(gait_data.data[i, 3])
            joint5_pos_pub.publish(gait_data.data[i, 4])
            joint6_pos_pub.publish(gait_data.data[i, 5])
            joint7_pos_pub.publish(gait_data.data[i, 6])
            joint8_pos_pub.publish(gait_data.data[i, 7])
            joint9_pos_pub.publish(gait_data.data[i, 8])
            joint10_pos_pub.publish(gait_data.data[i, 9])
            joint11_pos_pub.publish(gait_data.data[i, 10])
            joint12_pos_pub.publish(gait_data.data[i, 11])
            pause.sleep()
        j = j + 1
    return

def command_analysis(action_command):
    
    default_angle = [0, 0, 0]
    rpy_angle = rospy.get_param('/pigot/RPY_angle_new', default_angle)
    
    if (action_command == 'w'):     
        rate, gait_np_data = omni_control.diagonal_tranlate_control(1, 0, 0, 0, 0.25, 0.15, 0.15, rpy_angle)
    elif (action_command == 'x'):
        rate, gait_np_data = omni_control.diagonal_tranlate_control(0, 1, 0, 0, 0.25, 0.15, 0.15, rpy_angle)
    elif (action_command == 'a'):
        rate, gait_np_data = omni_control.diagonal_tranlate_control(0, 0, 1, 0, 0.25, 0.15, 0.15, rpy_angle)
    elif (action_command == 'd'):
        rate, gait_np_data = omni_control.diagonal_tranlate_control(0, 0, 0, 1, 0.25, 0.15, 0.15, rpy_angle)
    elif (action_command == 'q'):
        rate, gait_np_data = omni_control.diagonal_tranlate_control(1, 0, 1, 0, 0.25, 0.15, 0.15, rpy_angle)        
    elif (action_command == 'e'):
        rate, gait_np_data = omni_control.diagonal_tranlate_control(1, 0, 0, 1, 0.25, 0.15, 0.15, rpy_angle)
    elif (action_command == 'z'):
        rate, gait_np_data = omni_control.diagonal_tranlate_control(0, 1, 1, 0, 0.25, 0.15, 0.15, rpy_angle)
    elif (action_command == 'c'):
        rate, gait_np_data = omni_control.diagonal_tranlate_control(0, 1, 0, 1, 0.25, 0.15, 0.15, rpy_angle)
    elif (action_command == 's'):
        rate, gait_np_data = omni_control.diagonal_tranlate_control(0, 0, 0, 0, 0.25, 0.15, 0.15, rpy_angle)
    elif (action_command == 'r'):
        rate, gait_np_data = omni_control.diagonal_turn_control(1, 0.3, 70, rpy_angle)
    elif (action_command == 't'):
        rate, gait_np_data = omni_control.diagonal_turn_control(-1, 0.3, 70, rpy_angle)
#    elif (action_command == 'j'):
#        rate, gait_np_data = ka.jump_gait()
    elif (action_command == 'k'):
        rate, gait_np_data = ka.keep_gait()
    elif (action_command == 'l'):
        rate, gait_np_data = ka.clam_gait()
#    elif (action_command == 'q'):
#        rate, gait_np_data = ka.slantleft_gait()
#    elif (action_command == 'e'):
#        rate, gait_np_data = ka.slantright_gait()
    elif (action_command == '5' or '2' or '1' or '3' or '4' or '6' or '0'):
        
        RPY_angle_data_old = rospy.get_param('/pigot/RPY_angle_old', default_angle)
        RPY_angle_data_new_traj = rpy_angle
        
        gain_angle = 0.15
        if (action_command == '5'):
            RPY_angle_data_new_traj = [RPY_angle_data_new_traj[0], RPY_angle_data_new_traj[1]+gain_angle, RPY_angle_data_new_traj[2]]
        elif (action_command == '2'):
            RPY_angle_data_new_traj = [RPY_angle_data_new_traj[0], RPY_angle_data_new_traj[1]-gain_angle, RPY_angle_data_new_traj[2]]
        elif (action_command == '1'):
            RPY_angle_data_new_traj = [RPY_angle_data_new_traj[0]+gain_angle, RPY_angle_data_new_traj[1], RPY_angle_data_new_traj[2]]
        elif (action_command == '3'):
            RPY_angle_data_new_traj = [RPY_angle_data_new_traj[0]-gain_angle, RPY_angle_data_new_traj[1], RPY_angle_data_new_traj[2]]
        elif (action_command == '4'):
            RPY_angle_data_new_traj = [RPY_angle_data_new_traj[0], RPY_angle_data_new_traj[1], RPY_angle_data_new_traj[2]+gain_angle]
        elif (action_command == '6'):
            RPY_angle_data_new_traj = [RPY_angle_data_new_traj[0], RPY_angle_data_new_traj[1], RPY_angle_data_new_traj[2]-gain_angle]
        elif (action_command == '0'):
            RPY_angle_data_new_traj = RPY_angle_data_new_traj
            
        if_traj_plan = True
        error = np.linalg.norm(np.array(RPY_angle_data_new_traj))
        if (error>0.05):
            if if_traj_plan:
                rate, gait_np_data, RPY_B_traj = rpy.rpy_gait(RPY_angle_data_old, RPY_angle_data_new_traj)
                rospy.set_param('/pigot/RPY_angle_old', RPY_angle_data_new_traj)
            
    rospy.set_param('/pigot/in_wait', True)
    return rate, gait_np_data

        
if __name__ == '__main__':
    
    omni_control = ka.OMNI_CONTROL(0.2, 0.1, 0.1, 40)
    
    try:
        # Initialize the node and define the Publisher.
        rospy.init_node('pos_pub_node', anonymous=True)
    
        joint1_pos_pub = rospy.Publisher('/pigot/joint1_position_controller/command', Float64, queue_size=10)
        joint2_pos_pub = rospy.Publisher('/pigot/joint2_position_controller/command', Float64, queue_size=10)
        joint3_pos_pub = rospy.Publisher('/pigot/joint3_position_controller/command', Float64, queue_size=10)
        joint4_pos_pub = rospy.Publisher('/pigot/joint4_position_controller/command', Float64, queue_size=10)
        joint5_pos_pub = rospy.Publisher('/pigot/joint5_position_controller/command', Float64, queue_size=10)
        joint6_pos_pub = rospy.Publisher('/pigot/joint6_position_controller/command', Float64, queue_size=10)
        joint7_pos_pub = rospy.Publisher('/pigot/joint7_position_controller/command', Float64, queue_size=10)
        joint8_pos_pub = rospy.Publisher('/pigot/joint8_position_controller/command', Float64, queue_size=10)
        joint9_pos_pub = rospy.Publisher('/pigot/joint9_position_controller/command', Float64, queue_size=10)
        joint10_pos_pub = rospy.Publisher('/pigot/joint10_position_controller/command', Float64, queue_size=10)
        joint11_pos_pub = rospy.Publisher('/pigot/joint11_position_controller/command', Float64, queue_size=10)
        joint12_pos_pub = rospy.Publisher('/pigot/joint12_position_controller/command', Float64, queue_size=10)
        while not rospy.is_shutdown():
            # Read action command.
            action_command = rospy.get_param('/pigot/action_state_param', 'k') 
                    
            # Analyze the action command and do gait planning. Note that the gait data returned here is a numpy array.
            rate, gait_np_data = command_analysis(action_command)

            # Calculate the pause time for each step of publish to make the total publish frequency equal to the "rate" in gait planning.
            # Note that a total publish contains 40 steps of publish. (Number of nodes in the gait planning)
            data_length = gait_np_data.shape[0]
            pause = rospy.Rate(data_length * rate)
            
            # Assign the gait data to gait_data and publish.
            gait_data = Float32MultiArray() # Define the gait data as std_msgs.msg data because it is to be published to the topic.
            gait_data.data = gait_np_data
            action_pub(gait_data, data_length)

    except rospy.ROSInterruptException:
        pass

    

