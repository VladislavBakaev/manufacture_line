#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import math
from inverse_problem_srv.srv import publish_cmd,publish_cmdResponse
from std_msgs.msg import Bool

jointPub = rospy.Publisher("/cmd_joints",JointState,queue_size = 100)

fromEncToRadFor1066428 = 4096/(2*math.pi)
fromRncToRadFor12 = (1024*180)/(300*math.pi)
minEncPoseForGripper = 390
fromEncToLinGripper = 7.75
zeroPose = [2048, 2048, 2048, 2048, 512, 680]

currentState = JointState()
currentState.position = [0,0,0,0,0,0]
kVelRadSToMotorVel = 0.11/30*math.pi
kAccRadToMotorAcc = 8.58/180*math.pi
countOfJoint = 6
kVel = 1.5
stop_move = False

def UpdateCurJointState(curJS):
    global currentState
    currentState.position = curJS.position

def reachingOfThePoint(currentStateList,goalStatelList,error):
    reachingPoint = False
    if(math.fabs(currentStateList[0]-goalStatelList[0])<error and
       math.fabs(currentStateList[1]-goalStatelList[1])<error and
       math.fabs(currentStateList[2]-goalStatelList[2])<error and
       math.fabs(currentStateList[3]-goalStatelList[3])<error and
       math.fabs(currentStateList[4]-goalStatelList[4])<error):
       reachingPoint = True
    return reachingPoint

def way_compute(posList):
    global currentState
    wayList = []
    for i in range(len(posList)):
        wayList.append(posList[i]-currentState.position[i])
    return wayList

def max_way(wayList):
    maxWay = wayList[len(wayList)-1]
    for i in range(len(wayList)-1):
        if (math.fabs(wayList[i])>math.fabs(maxWay)):
            maxWay = wayList[i]
        else:
            continue
    return maxWay

def parse_msg(msg):
    jointState = JointState()
    msgList = msg.jointState.split()
    jointState.name = msgList[0:countOfJoint]
    jointState.position = msgList[countOfJoint:]
    jointState.position = list(map(float,jointState.position))
    return jointState

def convert_pose_vel_acc(name,poseList,velList,accList):
    jointcmd = JointState()
    jointcmd.name = name
    poseListPub = []
    velListPub = []
    accListPub = []
    for i in range(len(name)):
        if(name[i] == 'ang_joint_5'):
            velListPub.append(math.fabs(round(velList[i]/kVelRadSToMotorVel)))
            accListPub.append(math.fabs(round(accList[i]/kAccRadToMotorAcc)))
            if (accListPub[i]==0):
                accListPub[i] = 1
            if (velListPub[i]==0):
                velListPub[i] = 1
            poseListPub.append(round((poseList[i]*fromRncToRadFor12)+zeroPose[i]))
        elif(name[i] == 'gripper'):
            poseListPub.append(round(poseList[i]*fromEncToLinGripper + minEncPoseForGripper))
            velListPub.append(math.fabs(60))
            accListPub.append(math.fabs(2))
        else:
            poseListPub.append(round((poseList[i]*fromEncToRadFor1066428)+zeroPose[i]))
            velListPub.append(math.fabs(round(velList[i]/kVelRadSToMotorVel)))
            accListPub.append(math.fabs(round(accList[i]/kAccRadToMotorAcc)))
            if (accListPub[i]==0):
                accListPub[i] = 1
            if (velListPub[i]==0):
                velListPub[i] = 1
    jointcmd.position = poseListPub
    jointcmd.velocity = velListPub
    jointcmd.effort = accListPub

    return jointcmd    

def move_of_trapeze_principle(msg):
    velList = []
    accList = []
    msg = parse_msg(msg)
    wayList = way_compute(msg.position[0:countOfJoint-1])
    maxWay = max_way(wayList)
    time = 0
    if(not maxWay == 0):
        maxVelocity = math.sqrt(math.fabs(maxWay)/kVel)
        time = 3*maxWay/(2*maxVelocity)
    else:
        time = 1
    for i in range(len(msg.position)-1):
        velList.append(wayList[i]*3/(2*time))
        accList.append(wayList[i]*9/(2*time**2))
    jointCmd = convert_pose_vel_acc(msg.name,msg.position,velList,accList)

    jointCmd.header.stamp = rospy.Time.now()
    jointPub.publish(jointCmd)
    rate = rospy.Rate(2)
    while(not reachingOfThePoint(currentState.position,msg.position,0.05)):
        jointCmd.header.stamp = rospy.Time.now()
        jointPub.publish(jointCmd)
        rate.sleep()
        if(stop_move == True):
            curState = convert_pose_vel_acc(msg.name,currentState.position,velList,accList)
            curState.velocity = [60,60,60,60,60,60]
            curState.effort = [0,0,0,0,0,0]
            i=0
            while i<10:
                jointPub.publish(curState)
                i+=1
            break
    return publish_cmdResponse(True)

def UpdateRobotStop(mes):
    global stop_move
    stop_move = mes.data
     

if __name__=='__main__':
    rospy.init_node('convert_and_publish_state')
    rospy.Service('/cmd_joint_state_in_manip_coord', publish_cmd, move_of_trapeze_principle)
    rospy.Subscriber('/joint_states',JointState,UpdateCurJointState)
    rospy.Subscriber('/stop_robot',Bool,UpdateRobotStop)
    rospy.spin()
