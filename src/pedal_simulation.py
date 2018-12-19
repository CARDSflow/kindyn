#!/usr/bin/env python
from __future__ import print_function

#roslaunch kindyn robot.launch robot_name:=rikshaw model_name:=rikshaw start_controllers:='joint_hip_right joint_knee_right joint_foot_right joint_hip_left joint_knee_left joint_foot_left joint_pedal'


import json
import math
import time

import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32


#############################
###   MODULE PARAMETERS   ###
#############################

PRINT_DEBUG = True

RECORDED_TRAJECTORY_FILENAME = "captured_trajectory_ik.json"

PEDAL_POSITION_ERROR_TOLERANCE = 0.03  # [meters]
PEDAL_SINGLE_ROTATION_DURATION = 20  # [seconds]
TRAJECTORY_POINT_DURATION      = 1
CONTROLLER_FREQUENCY           = 10  # [Hz]
MIN_JOINT_VEL                  = -1
MAX_JOINT_VEL                  = 1

############################
###   GLOBAL VARIABLES   ###
############################

ROS_JOINT_HIP_RIGHT   = "joint_hip_right"
ROS_JOINT_KNEE_RIGHT  = "joint_knee_right"
ROS_JOINT_ANKLE_RIGHT = "joint_foot_right"
ROS_JOINT_HIP_LEFT    = "joint_hip_left"
ROS_JOINT_KNEE_LEFT   = "joint_knee_left"
ROS_JOINT_ANKLE_LEFT  = "joint_foot_left"

RIGHT_HIP_JOINT   = "right_hip"
RIGHT_KNEE_JOINT  = "right_knee"
RIGHT_ANKLE_JOINT = "right_ankle"
LEFT_HIP_JOINT    = "left_hip"
LEFT_KNEE_JOINT   = "left_knee"
LEFT_ANKLE_JOINT  = "left_ankle"

_jointsList = [RIGHT_HIP_JOINT, RIGHT_KNEE_JOINT, RIGHT_ANKLE_JOINT, LEFT_HIP_JOINT, LEFT_KNEE_JOINT, LEFT_ANKLE_JOINT]


_parametersRightHip = {
    "param_p":               0.5,
    "param_i":               0.1,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersRightKnee = {
    "param_p":               0.5,
    "param_i":               0.1,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersRightAnkle = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersLeftHip = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersLeftKnee = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersLeftAnkle = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_jointsControlData = {
    RIGHT_HIP_JOINT:    _parametersRightHip,
    RIGHT_KNEE_JOINT:   _parametersRightKnee,
    RIGHT_ANKLE_JOINT:  _parametersRightAnkle,
    LEFT_HIP_JOINT:     _parametersLeftHip,
    LEFT_KNEE_JOINT:    _parametersLeftKnee,
    LEFT_ANKLE_JOINT:   _parametersLeftAnkle
}

_jointsStatusData = {
    RIGHT_HIP_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    LEFT_HIP_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    RIGHT_KNEE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    LEFT_KNEE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    RIGHT_ANKLE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    LEFT_ANKLE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    }
}

numTrajectoryPoints = 0
trajectoryStartingPoint = 0

pedalTrajectory = []
hipTrajectory   = []
kneeTrajectory  = []
ankleTrajectory = []

##############################
###   UTILITY FUNCTIONS   ###
##############################

def jointStateCallback(joint_data):
    global _jointsStatusData
    # Assert order of joints
    for stringIter in range(len(joint_data.names)):
        if joint_data.names[stringIter] == ROS_JOINT_HIP_RIGHT:
            _jointsStatusData[RIGHT_HIP_JOINT]["Pos"] = joint_data.q[stringIter]
            _jointsStatusData[RIGHT_HIP_JOINT]["Vel"] = joint_data.qd[stringIter]
        elif joint_data.names[stringIter] == ROS_JOINT_HIP_LEFT:
            _jointsStatusData[LEFT_HIP_JOINT]["Pos"] = joint_data.q[stringIter]
            _jointsStatusData[LEFT_HIP_JOINT]["Vel"] = joint_data.qd[stringIter]
        elif joint_data.names[stringIter] == ROS_JOINT_KNEE_RIGHT:
            _jointsStatusData[RIGHT_KNEE_JOINT]["Pos"] = joint_data.q[stringIter]
            _jointsStatusData[RIGHT_KNEE_JOINT]["Vel"] = joint_data.qd[stringIter]
        elif joint_data.names[stringIter] == ROS_JOINT_KNEE_LEFT:
            _jointsStatusData[LEFT_KNEE_JOINT]["Pos"] = joint_data.q[stringIter]
            _jointsStatusData[LEFT_KNEE_JOINT]["Vel"] = joint_data.qd[stringIter]
        elif joint_data.names[stringIter] == ROS_JOINT_ANKLE_RIGHT:
            _jointsStatusData[RIGHT_ANKLE_JOINT]["Pos"] = joint_data.q[stringIter]
            _jointsStatusData[RIGHT_ANKLE_JOINT]["Vel"] = joint_data.qd[stringIter]
        elif joint_data.names[stringIter] == ROS_JOINT_ANKLE_LEFT:
            _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"] = joint_data.q[stringIter]
            _jointsStatusData[LEFT_ANKLE_JOINT]["Vel"] = joint_data.qd[stringIter]

def importJointTrajectoryRecord():

    global numTrajectoryPoints
    global pedalTrajectory
    global hipTrajectory
    global kneeTrajectory
    global ankleTrajectory
    global PRINT_DEBUG

    with open(RECORDED_TRAJECTORY_FILENAME, "r") as read_file:
        loaded_data = json.load(read_file)

    if loaded_data["num_points"] == None:
        return 0
    else:
        numTrajectoryPoints = loaded_data["num_points"]

    # Deleting previous trajectory before loading new
    del pedalTrajectory[:]
    del hipTrajectory[:]
    del kneeTrajectory[:]
    del ankleTrajectory[:]
    for pointIterator in range (numTrajectoryPoints):
	if ("point_"+str(pointIterator) in loaded_data):
		pedalTrajectory.append(loaded_data["point_"+str(pointIterator)]["Pedal"])
		hipTrajectory.append(loaded_data["point_"+str(pointIterator)]["Hip"])
		kneeTrajectory.append(loaded_data["point_"+str(pointIterator)]["Knee"])
		ankleTrajectory.append(loaded_data["point_"+str(pointIterator)]["Ankle"])
	else:
		numTrajectoryPoints -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(numTrajectoryPoints)
        print("--------- Hip trajectory:")
        print(hipTrajectory)
        print("--------- Knee trajectory:")
        print(kneeTrajectory)
        print("--------- Ankle trajectory:")
        print(ankleTrajectory)


def getJointPosition(jointName):
    global _jointsStatusData
    return _jointsStatusData[jointName]["Pos"]


def getJointVelocity(jointName):
    return _jointsStatusData[jointName]["Vel"]

def getPosition(endeffector, frame):
    fkJointNamesList = [ROS_JOINT_HIP_RIGHT, ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_RIGHT, ROS_JOINT_ANKLE_LEFT]
    fkJointPositions = [_jointsStatusData[RIGHT_HIP_JOINT]["Pos"], _jointsStatusData[LEFT_HIP_JOINT]["Pos"], _jointsStatusData[RIGHT_KNEE_JOINT]["Pos"], _jointsStatusData[LEFT_KNEE_JOINT]["Pos"], _jointsStatusData[RIGHT_ANKLE_JOINT]["Pos"], _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv(endeffector, frame, fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException, e:
        print("Service call failed: %s"%(e))
    return [0.0, 0.0] #[x, z]

def getPositionLeftFoot():
    fkJointNamesList = [ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT]
    fkJointPositions = [_jointsStatusData[LEFT_HIP_JOINT]["Pos"], _jointsStatusData[LEFT_KNEE_JOINT]["Pos"], _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("pedal_left", "pedal_left", fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException, e:
        print("Service call failed: %s"%(e))
    return [0.0, 0.0] #[x, z]

def getPositionRightFoot():
    fkJointNamesList = [ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT]
    fkJointPositions = [_jointsStatusData[RIGHT_HIP_JOINT]["Pos"], _jointsStatusData[RIGHT_KNEE_JOINT]["Pos"], _jointsStatusData[RIGHT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("pedal_right", "foot_right", fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException, e:
        print("Service call failed: %s"%(e))

    print("ERROR ERROR in fk")
    return [0.0, 0.0] #[x, z]

def getDistance(point1,point2):

    x_diff = point2[0] - point1[0]
    y_diff = point2[1] - point1[1]

    return math.sqrt((x_diff*x_diff) + (y_diff*y_diff))

def setPedalSingleRotationDuration(new_duration_seconds):
    global PEDAL_SINGLE_ROTATION_DURATION
    PEDAL_SINGLE_ROTATION_DURATION = new_duration_seconds
    setTrajectoryPointDuration()
    return 1

def setTrajectoryPointDuration():
    global TRAJECTORY_POINT_DURATION
    if numTrajectoryPoints != 0:
        TRAJECTORY_POINT_DURATION = float(PEDAL_SINGLE_ROTATION_DURATION) / numTrajectoryPoints
    else:
        TRAJECTORY_POINT_DURATION = 1
        return 0

    return 1

def interpolateTrajectoryPoints(jointName, value1, value2, startTime, currTime, endTime):
    if currTime > endTime:
        return value2
    return value1 + (value2 - value1)*(float(currTime - startTime)/(endTime - startTime))


#############################
###   CONTROL FUNCTIONS   ###
#############################

# PID Controller
def computeVelocitySetpoint(jointName, goalPos, currTime, endTime):

    global _jointsControlData


    currPos = getJointPosition(jointName)
    jointError = goalPos - currPos

    #if currTime < (endTime-0.05):
    #    jointTravelTime = endTime - currTime
    #else:
    #    jointTravelTime = 3
    #jointVelocityReachGoal = float(jointTravelDistance) / jointTravelTime

    thisPosErrorDerivative = jointError - _jointsControlData[jointName]["prev_error"]

    #print("COMPUTED %s VELOCITY SETPOINT: %s (jointTravelTime: %s, jointTravelDistance: %s)" % (jointName, jointVelocityReachGoal, jointTravelTime, jointTravelDistance))

    #_jointsControlData[jointName]["pos_error_integral"] += float(jointError)/CONTROLLER_FREQUENCY
    #_jointsControlData[jointName]["prev_time"] = currTime
    #_jointsControlData[jointName]["prev_pos"] = currPos
    #_jointsControlData[jointName]["prev_error"] = jointError

    thisReturnVal = _jointsControlData[jointName]["param_p"]*jointError 

#+ _jointsControlData[jointName]["param_i"]*_jointsControlData[jointName]["pos_error_integral"] #+ _jointsControlData[jointName]["param_d"]*thisPosErrorDerivative
    if thisReturnVal > MAX_JOINT_VEL:
	thisReturnVal = MAX_JOINT_VEL

    if thisReturnVal < MIN_JOINT_VEL:
	thisReturnVal = MIN_JOINT_VEL

    if jointName == RIGHT_HIP_JOINT:
        #print("Right hip integral value: %s \t\t Setpoint: %s \t\t currPos: %s \t\t goalPos: %s " % (_jointsControlData[jointName]["pos_error_integral"], thisReturnVal, currPos, goalPos), end='\r')
	print("\t\t\t\t%0.5f\t\t\t\t%0.5f" % (jointError, thisReturnVal), end='\r')
    elif jointName == RIGHT_KNEE_JOINT:
        #thisReturnVal = thisReturnVal*2
	print("\t\t\t\t\t\t\t\t\t\t\t\t%0.5f\t\t\t\t%0.5f" % (jointError, thisReturnVal), end='\r')
    return thisReturnVal


#########################
###   STATE MACHINE   ###
#########################

INIT              = "INIT"
PEDAL             = "PEDAL"
UPDATE_PARAMETERS = "UPDATE_PARAMETERS"

def FSM():

    global numTrajectoryPoints
    global _jointsControlData
    global _jointsList

    _runFSM = 1

    _currState = INIT
    _currTrajectoryPoint = -1

    _startTime = 0.0
    _endTime = 0.0
    _currTime = 0.0
    _prevTime = 0.0

    ros_right_hip_publisher = rospy.Publisher('/joint_hip_right/joint_hip_right/target', Float32, queue_size=2)
    ros_right_knee_publisher = rospy.Publisher('/joint_knee_right/joint_knee_right/target', Float32, queue_size=2)
    ros_right_ankle_publisher = rospy.Publisher('/joint_foot_right/joint_foot_right/target', Float32, queue_size=2)

    ros_left_hip_publisher = rospy.Publisher('/joint_hip_left/joint_hip_left/target', Float32, queue_size=2)
    ros_left_knee_publisher = rospy.Publisher('/joint_knee_left/joint_knee_left/target', Float32, queue_size=2)
    ros_left_ankle_publisher = rospy.Publisher('/joint_foot_left/joint_foot_left/target', Float32, queue_size=2)


    while _runFSM:

        ##############################################
        if _currState == INIT:
        ##############################################

            importJointTrajectoryRecord()
            setTrajectoryPointDuration()

            # Find starting point on the trajectory

            _currState = PEDAL

        ##############################################
        if _currState == PEDAL:
        ##############################################

            # Initialize state
            if _currTrajectoryPoint == -1:
                _currTrajectoryPoint = trajectoryStartingPoint
            if _startTime == 0.0:
                _startTime = time.time()
            if _endTime == 0.0:
                _endTime = _startTime + TRAJECTORY_POINT_DURATION
            if _prevTime == 0.0:
                _prevTime = time.time()

            # Regulate update frequency
            _currTime = time.time()
            while float(float(_currTime) - float(_prevTime)) < (1 / CONTROLLER_FREQUENCY):
                time.sleep(1)
                _currTime = time.time()
            _prevTime = _currTime

            # Check if trajectory point reached and act accordingly
            print("%0.5f" % (getDistance(getPositionRightFoot(), pedalTrajectory[_currTrajectoryPoint])), end='\r')
            if getDistance(getPositionRightFoot(), pedalTrajectory[_currTrajectoryPoint]) <= PEDAL_POSITION_ERROR_TOLERANCE and _currTime >= _endTime:
                for thisJointName in _jointsList: #getDistance(getPositionLeftFoot(), pedalTrajectory[_currTrajectoryPoint]) <= PEDAL_POSITION_ERROR_TOLERANCE and 
                    #!!!!_jointsControlData[thisJointName]["trajectory_startpoint"] = hipTrajectory[_currTrajectoryPoint]
                    _jointsControlData[thisJointName]["pos_error_integral"] = 0
                if (_currTrajectoryPoint < (numTrajectoryPoints-1)):
                    _currTrajectoryPoint += 1
                elif (_currTrajectoryPoint >= (numTrajectoryPoints-1)):
                    _currTrajectoryPoint = 0
                if PRINT_DEBUG:
                    print("UPDATING TRAJECTORY POINT. NEW POINT: %s" % (_currTrajectoryPoint))
                _startTime = time.time()
                _endTime = _startTime + TRAJECTORY_POINT_DURATION

            # Iterate through joints and update setpoints
            for thisJointName in _jointsList:

                thisJointPositionGoalpoint = 0.0
                if ((thisJointName == RIGHT_HIP_JOINT) or (thisJointName == LEFT_HIP_JOINT)):
                    thisJointPositionGoalpoint = hipTrajectory[_currTrajectoryPoint]
                elif ((thisJointName == RIGHT_KNEE_JOINT) or (thisJointName == LEFT_KNEE_JOINT)):
                    thisJointPositionGoalpoint = kneeTrajectory[_currTrajectoryPoint]
                elif ((thisJointName == RIGHT_ANKLE_JOINT) or (thisJointName == LEFT_ANKLE_JOINT)):
                    thisJointPositionGoalpoint = ankleTrajectory[_currTrajectoryPoint]


                _currTime = time.time()

                #thisJointPositionSetpoint = interpolateTrajectoryPoints(thisJointName, _jointsControlData[thisJointName]["trajectory_startpoint"], thisJointPositionGoalpoint, _startTime, _currTime, _endTime)

                thisJointVelocitySetpoint = computeVelocitySetpoint(thisJointName, thisJointPositionGoalpoint, _currTime, _endTime)

                if thisJointName == RIGHT_HIP_JOINT:
                    ros_right_hip_publisher.publish(thisJointVelocitySetpoint)
                elif thisJointName == RIGHT_KNEE_JOINT:
                    ros_right_knee_publisher.publish(thisJointVelocitySetpoint)
                elif thisJointName == RIGHT_ANKLE_JOINT:
                    ros_right_ankle_publisher.publish(thisJointVelocitySetpoint)
                #elif thisJointName == LEFT_HIP_JOINT:
                #    ros_left_hip_publisher.publish(thisJointVelocitySetpoint)
                #elif thisJointName == LEFT_KNEE_JOINT:
                #    ros_left_knee_publisher.publish(thisJointVelocitySetpoint)
                #elif thisJointName == LEFT_ANKLE_JOINT:
                #    ros_left_ankle_publisher.publish(thisJointVelocitySetpoint)

        ##############################################
        #if _currState == UPDATE_PARAMETERS:
        ##############################################

            # Reload trajectory and PID parameters

    return 1

################
###   MAIN   ###
################

def main():

    rospy.init_node('pedal_simulation', anonymous=True)
    rospy.Subscriber("joint_state", JointState, jointStateCallback)

    FSM()

    return 1


if __name__ == '__main__':
    main()



