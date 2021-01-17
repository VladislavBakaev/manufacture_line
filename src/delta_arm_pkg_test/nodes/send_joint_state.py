#!/usr/bin/env python
import rospy  
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64 
import math

joint_pub = rospy.Publisher("/delta_robot/joint_states", JointState, queue_size=10)
fromEncToRadFor1066428 = 4096/(2*math.pi)
zeroPose = [2048,2048,2048]

def app_new_joint_state(joint_state_msg,joint_name,position,zero):
    joint_state_msg.name.append(joint_name)
    position = (position-zero)/fromEncToRadFor1066428
    joint_state_msg.position.append(round(position,2))
    
    return joint_state_msg

def read_pose_callback(data_list):
    joint_state = JointState()

    joint_state = app_new_joint_state(joint_state,'delta_joint1',data_list.position[0],zeroPose[0])
    joint_state = app_new_joint_state(joint_state,'delta_joint2',data_list.position[1],zeroPose[1])
    joint_state = app_new_joint_state(joint_state,'delta_joint3',data_list.position[2],zeroPose[2])

    joint_state.header.stamp = rospy.Time.now()
    joint_pub.publish(joint_state)

if __name__ == '__main__':
    rospy.init_node('Send_joint_state')
    #rospy.loginfo('Send_joint_state node was started')
    rospy.Subscriber('/arm_joint_states', JointState, read_pose_callback)
    rospy.spin()
