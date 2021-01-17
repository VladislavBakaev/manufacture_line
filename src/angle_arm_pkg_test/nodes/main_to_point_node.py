#!/usr/bin/env python
import rospy  
import math
from sensor_msgs.msg import JointState
from RoboticArmClass import RoboticArm
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from inverse_problem_srv.srv import publish_cmd,publish_cmdResponse

nameList = ['ang_joint_1','ang_joint_2','ang_joint_3','ang_joint_4','ang_joint_5','gripper']
jointStateSrv = rospy.ServiceProxy('/cmd_joint_state_in_manip_coord', publish_cmd)
gripperPose = '40'
curJointState = [0,0,0,0,0]

def ParseMsg(msg):
    try:
        coord_list = msg.point.split()
        x = float(coord_list[0])
        y = float(coord_list[1])
        z = float(coord_list[2])
        pith = float(coord_list[3])
        roll = float(coord_list[4])
        return x,y,z,pith,roll
    except ValueError:
        rospy.logerr('Input Error')

def MoveToPointCallback(msg):
    global lastGoalJointState
    x,y,z,pitch,roll = ParseMsg(msg)
    roboticArm = RoboticArm()
    availJointState,goalJointState = roboticArm.InversProblem(x,y,z,pitch,roll)

    if (not availJointState):
        #rospy.loginfo('Point cannot be reached')
        return point_cmdResponse(False)
    else:
        #rospy.loginfo('Wait...')
        goalJointState = [str(el) for el in goalJointState]
        strName = ' '.join(nameList)
        strJS = ' '.join(goalJointState) + ' ' + gripperPose
        strCmd = strName + ' ' + strJS
        jointStateSrv(strCmd)
        #rospy.loginfo('Well Done!!!')
    	return point_cmdResponse(True)

def GripperCmdCallback(msg):
    global gripperPose
    gripperPose = msg.point
    strName = ' '.join(nameList)
    curJointState_str = list(map(str,curJointState))
    strJS = ' '.join(curJointState_str) + ' ' + gripperPose
    strCmd = strName + ' ' + strJS
    jointStateSrv(strCmd)
    #rospy.loginfo('Well Done!!!')
    return point_cmdResponse(True)

def UpdateJointState(jointState):
    global curJointState
    curJointState = jointState.position[0:5]


if __name__=='__main__':
    global lastGoalJointState
    rospy.init_node('angle_main_to_point_node')
    rospy.Service('/cmd_point', point_cmd, MoveToPointCallback)
    rospy.Service('/gripper_cmd', point_cmd, GripperCmdCallback)
    rospy.Subscriber("/joint_states",JointState,UpdateJointState)
    rospy.spin()
