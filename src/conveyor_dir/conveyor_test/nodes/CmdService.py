#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from srv_msg.srv import DistCmd, DistCmdResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from math import fabs

cmdVelPub = rospy.Publisher('/cmd_vel',Float64,queue_size=10)

cmdVel = 3
curVel = 0
r = 0.025

def Sign(num):
    if (num > 0):
        return 1
    else:
        return -1

def UpdateCurVel(cur):
    global curVel
    curVel = cur.velocity[0]*r

def LinDistControl(req):
    curDist = 0
    sign = Sign(req.cmd)
    rate = rospy.Rate(10)
    lastTime = rospy.get_time()
    while (fabs(curDist) < fabs(req.cmd)):
        cmdVelPub.publish(sign*cmdVel)
        curTime = rospy.get_time()
        curDist +=curVel*(curTime-lastTime)
        lastTime = curTime
        rate.sleep()
    while (not curVel == 0):
        cmdVelPub.publish(0)
        rate.sleep()

    return DistCmdResponse(True)

if __name__ == "__main__":

    rospy.init_node('cmd_server_lin_dist')
    sl = rospy.Service('/cmd_lin_dist', DistCmd, LinDistControl)
    rospy.Subscriber('/vel_joint_states', JointState,UpdateCurVel)
    rospy.spin()
