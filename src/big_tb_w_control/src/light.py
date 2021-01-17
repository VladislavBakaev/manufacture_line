#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

light_threshold = 200
linear_speed = 0.20
step_angular_speed = 0.15
pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 10)
id_point = -1
ang_vel = 0


def frontCallBack(msg):
    global id_point,ang_vel
    if(id_point == 0):
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        light_bin = []
        msg_data = msg.data
        for el in msg_data:
            if(el-light_threshold<0):
                light_bin.append(1)
            else:
                light_bin.append(0)
        if (light_bin.count(1)>3):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            ang_vel = 0
            id_point = -1
        elif(light_bin.count(1)==0):
            pass
        else:
            ang_vel = 0
            for i in range(len(light_bin)):
                ang_vel +=light_bin[i]*(3-i)*step_angular_speed
        vel_msg.angular.z = ang_vel
        pub.publish(vel_msg)

def backCallBack(msg):
    global id_point,ang_vel
    if(id_point == 1):
        vel_msg = Twist()
        vel_msg.linear.x = -linear_speed
        light_bin = []
        msg_data = msg.data
        for el in msg_data:
            if(el - light_threshold < 0):
                light_bin.append(1)
            else:
                light_bin.append(0)
        if(light_bin.count(1)>3):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            ang_vel = 0
            id_point = -1
        elif(light_bin.count(1) == 0):
            pass
        else:
            ang_vel = 0
            for i in range(len(light_bin)):
                ang_vel -=light_bin[i]*(2-i)*step_angular_speed
        vel_msg.angular.z = ang_vel
        pub.publish(vel_msg)

def idPointCallBack(msg):
    global id_point
    id_point = msg.data

if __name__ == "__main__":
    rospy.init_node('light_node')
    rospy.Subscriber('/front_light_array',UInt16MultiArray,frontCallBack)
    rospy.Subscriber('/back_light_array',UInt16MultiArray,backCallBack)
    rospy.Subscriber('/point_id',Int32,idPointCallBack)
    rospy.spin()
