## @package pedaling

from __future__ import print_function

#roslaunch kindyn robot.launch robot_name:=rikshaw model_name:=rikshaw start_controllers:='joint_hip_right joint_knee_right joint_foot_right joint_hip_left joint_knee_left joint_foot_left joint_pedal'

#roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='joint_hip_left joint_hip_right joint_wheel_right joint_wheel_back joint_pedal spine_joint joint_wheel_left joint_front joint_pedal_right joint_pedal_left elbow_right_rot1 joint_foot_left joint_knee_right joint_knee_left joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left_rot1 elbow_left_rot0 left_wrist_0 left_wrist_1 right_shoulder_axis0 right_shoulder_axis2 right_shoulder_axis1 elbow_right_rot0 right_wrist_0 right_wrist_1 head_axis0 head_axis1 head_axis2'

import json
import math
import time

import matplotlib.pyplot as plt

import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from roboy_control_msgs.srv import SetControllerParameters
from std_msgs.msg import Float32


#############################
###   MODULE PARAMETERS   ###
#############################

PRINT_DEBUG = True

RECORDED_TRAJECTORY_FILENAME = "../trajectory_pedaling/desember_pedal_trajectory.json"

PEDAL_POSITION_ERROR_TOLERANCE   = 0.02  # [meters]
JOINT_TRAJECTORY_ERROR_TOLERANCE = 0.02
PEDAL_SINGLE_ROTATION_DURATION   = 20  # [seconds]
TRAJECTORY_POINT_DURATION        = 1
CONTROLLER_FREQUENCY             = 100  # [Hz]
MIN_JOINT_VEL                    = -500
MAX_JOINT_VEL                    = 500
JOINT_VELOCITY_FACTOR            = 1000

############################
###   GLOBAL VARIABLES   ###
############################

x_pedal_record = []
y_pedal_record = []

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

_jointsListROS = [ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT, ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT]


_parametersRightHip = {
    "param_p":               1500.0,
    "param_i":               0.05,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint":   0.0,
    "ideal_velocity":        0.0,
    "bool_update_iv":        True
}

_parametersRightKnee = {
    "param_p":               2000.0,
    "param_i":               0.05,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint":   0.0,
    "ideal_velocity":        0.0,
    "bool_update_iv":        True
}

_parametersRightAnkle = {
    "param_p":               1000.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint":   0.0,
    "ideal_velocity":        0.0,
    "bool_update_iv":        True
}

_parametersLeftHip = {
    "param_p":               1500.0,
    "param_i":               0.05,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint":   0.0,
    "ideal_velocity":        0.0,
    "bool_update_iv":        True
}

_parametersLeftKnee = {
    "param_p":               2000.0,
    "param_i":               0.05,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint":   0.0,
    "ideal_velocity":        0.0,
    "bool_update_iv":        True
}

_parametersLeftAnkle = {
    "param_p":               1000.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint":   0.0,
    "ideal_velocity":        0.0,
    "bool_update_iv":        True
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

numTrajectoryPoints = -1
trajectoryStartingPoint = 0

pedalTrajectoryRight = []
pedalTrajectoryLeft  = []
hipTrajectoryRight   = []
kneeTrajectoryRight  = []
ankleTrajectoryRight = []
hipTrajectoryLeft    = []
kneeTrajectoryLeft   = []
ankleTrajectoryLeft  = []

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


def setJointControllerParameters(proportionalVal, derivativeVal):

    for thisJointName in _jointsListROS:
        rospy.wait_for_service(thisJointName + '/' + thisJointName + '/params')
        try:
            param_srv = rospy.ServiceProxy(thisJointName + '/' + thisJointName + '/params', SetControllerParameters)
            param_srv(proportionalVal, derivativeVal)
        except rospy.ServiceException as e:
            print("Service call for " + thisJointName + "failed: " + e)

    print("Controller paramters updated")


def importJointTrajectoryRecord():

    global numTrajectoryPoints
    global pedalTrajectoryLeft
    global pedalTrajectoryRight
    global hipTrajectoryRight
    global kneeTrajectoryRight
    global ankleTrajectoryRight
    global hipTrajectoryLeft
    global kneeTrajectoryLeft
    global ankleTrajectoryLeft
    global PRINT_DEBUG

    with open(RECORDED_TRAJECTORY_FILENAME, "r") as read_file:
        loaded_data = json.load(read_file)

    if loaded_data["num_points"] is None:
        return 0
    else:
        numTrajectoryPoints = loaded_data["num_points"]

    # Deleting previous trajectory before loading new
    del pedalTrajectoryLeft[:]
    del pedalTrajectoryRight[:]
    del hipTrajectoryRight[:]
    del kneeTrajectoryRight[:]
    del ankleTrajectoryRight[:]
    del hipTrajectoryLeft[:]
    del kneeTrajectoryLeft[:]
    del ankleTrajectoryLeft[:]
    for pointIterator in range(numTrajectoryPoints):
        if "point_"+str(pointIterator) in loaded_data:
            pedalTrajectoryLeft.append(loaded_data["point_"+str(pointIterator)]["Left"]["Pedal"])
            pedalTrajectoryRight.append(loaded_data["point_"+str(pointIterator)]["Left"]["Pedal"])
            hipTrajectoryRight.append(loaded_data["point_"+str(pointIterator)]["Left"]["Hip"])
            kneeTrajectoryRight.append(loaded_data["point_"+str(pointIterator)]["Left"]["Knee"])
            ankleTrajectoryRight.append(loaded_data["point_"+str(pointIterator)]["Left"]["Ankle"])
            hipTrajectoryLeft.append(loaded_data["point_"+str(pointIterator)]["Left"]["Hip"])
            kneeTrajectoryLeft.append(loaded_data["point_"+str(pointIterator)]["Left"]["Knee"])
            ankleTrajectoryLeft.append(loaded_data["point_"+str(pointIterator)]["Left"]["Ankle"])
        else:
            print("WARNING: No point_%s in trajectory" % pointIterator)
            numTrajectoryPoints -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(numTrajectoryPoints)


def getJointPosition(jointName):
    global _jointsStatusData
    return _jointsStatusData[jointName]["Pos"]


def getJointVelocity(jointName):
    global _jointsStatusData
    return _jointsStatusData[jointName]["Vel"]


def getPosition(endeffector, frame):
    fkJointNamesList = [ROS_JOINT_HIP_RIGHT, ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_RIGHT, ROS_JOINT_ANKLE_LEFT]
    fkJointPositions = [_jointsStatusData[RIGHT_HIP_JOINT]["Pos"], _jointsStatusData[LEFT_HIP_JOINT]["Pos"], _jointsStatusData[RIGHT_KNEE_JOINT]["Pos"], _jointsStatusData[LEFT_KNEE_JOINT]["Pos"], _jointsStatusData[RIGHT_ANKLE_JOINT]["Pos"], _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv(endeffector, frame, fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException as e:
        print("Service call failed:", e)
    return [0.0, 0.0]  # [x, z]


def getPositionLeftFoot():
    fkJointNamesList = [ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT]
    fkJointPositions = [_jointsStatusData[LEFT_HIP_JOINT]["Pos"], _jointsStatusData[LEFT_KNEE_JOINT]["Pos"], _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("left_leg", "foot_left_tip", fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException as e:
        print("Service call failed:", e)

    print("ERROR fk foot_left failed")
    return [0.0, 0.0]  # [x, z]


def getPositionRightFoot():
    fkJointNamesList = [ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT]
    fkJointPositions = [_jointsStatusData[RIGHT_HIP_JOINT]["Pos"], _jointsStatusData[RIGHT_KNEE_JOINT]["Pos"], _jointsStatusData[RIGHT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("right_leg", "foot_right_tip", fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException as e:
        print("Service call failed:", e)

    print("ERROR fk foot_right failed")
    return [0.0, 0.0]  # [x, z]


def getDistance(point1, point2):

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
    global PEDAL_SINGLE_ROTATION_DURATION
    global numTrajectoryPoints
    if numTrajectoryPoints != 0:
        TRAJECTORY_POINT_DURATION = float(PEDAL_SINGLE_ROTATION_DURATION) / numTrajectoryPoints
    else:
        TRAJECTORY_POINT_DURATION = 1
        return 0

    return 1


#############################
###   CONTROL FUNCTIONS   ###
#############################

def interpolateTrajectoryPoints(value1, value2, startTime, currTime, endTime):
    if currTime > endTime:
        return value2
    return value1 + (value2 - value1)*(float(currTime - startTime)/(endTime - startTime))


def checkOutputLimits(inputVal):

    returnVal = inputVal

    if inputVal > MAX_JOINT_VEL:
        returnVal = MAX_JOINT_VEL
    elif inputVal < MIN_JOINT_VEL:
        returnVal = MIN_JOINT_VEL

    return returnVal


def computeVelocitySetpoint(jointName, endPos, startTime, currTime, endTime):

    global _jointsControlData
    global PRINT_DEBUG
    global JOINT_TRAJECTORY_ERROR_TOLERANCE
    global JOINT_VELOCITY_FACTOR

    

    # currPos = getJointPosition(jointName)
    # goalPos = interpolateTrajectoryPoints(_jointsControlData[jointName]["trajectory_startpoint"], endPos, startTime, currTime, endTime)
    # jointError = goalPos - currPos
    #
    # if _jointsControlData[jointName]["bool_update_iv"]:
    #     if currTime < endTime:
    #         jointTravelTime = endTime - currTime
    #         jointTravelDistance = endPos - currPos
    #         jointIdealVelocity = float(jointTravelDistance) / jointTravelTime
    #         _jointsControlData[jointName]["ideal_velocity"] = jointIdealVelocity
    #         _jointsControlData[jointName]["bool_update_iv"] = False
    #     else:
    #         print("ERROR in compute ideal joint velocity: currTime > endTime")
    #
    # thisPosErrorDerivative = float(jointError - _jointsControlData[jointName]["prev_error"])/CONTROLLER_FREQUENCY
    #
    # #print("COMPUTED %s VELOCITY SETPOINT: %s (jointTravelTime: %s, jointTravelDistance: %s)" % (jointName, jointVelocityReachGoal, jointTravelTime, jointTravelDistance))
    #
    # thisReturnVal = _jointsControlData[jointName]["ideal_velocity"]
    # thisReturnVal = thisReturnVal*JOINT_VELOCITY_FACTOR
    # # SWITCH CONTROL MODE IF STATEMENT IS TRUE (FROM IDEAL VELOCITY TO PID POSITION ERROR)
    # if currTime > endTime:  # jointError > JOINT_TRAJECTORY_ERROR_TOLERANCE or
    #     if PRINT_DEBUG:
    #         print("Switching control to PID for joint %s" % jointName)
    #     _jointsControlData[jointName]["pos_error_integral"] += float(jointError)/CONTROLLER_FREQUENCY
    #     _jointsControlData[jointName]["prev_time"] = currTime
    #     _jointsControlData[jointName]["prev_pos"] = currPos
    #     _jointsControlData[jointName]["prev_error"] = jointError
    #     thisReturnVal = _jointsControlData[jointName]["param_p"]*jointError + _jointsControlData[jointName]["param_i"]*_jointsControlData[jointName]["pos_error_integral"] + _jointsControlData[jointName]["param_d"]*thisPosErrorDerivative
    #
    # thisReturnVal = checkOutputLimits(thisReturnVal)
    #
    # if PRINT_DEBUG:
    #     if jointName == RIGHT_HIP_JOINT:
    #         print("\t\t\t\t%0.5f\t\t\t\t%0.5f" % (jointError, thisReturnVal), end='\r')
    #     elif jointName == RIGHT_KNEE_JOINT:
    #         print("\t\t\t\t\t\t\t\t\t\t\t\t%0.5f\t\t\t\t%0.5f" % (jointError, thisReturnVal), end='\r')

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
    global PRINT_DEBUG

    global x_pedal_record
    global y_pedal_record
    global pedalTrajectoryRight

    _runFSM = True

    _currState = INIT
    _currTrajectoryPoint = -1

    _startTime = 0.0
    _endTime = 0.0
    _currTime = 0.0
    _prevTime = 0.0

    initial_trajectory_point = 0
    past_initial_trajectory_point = False

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
                initial_trajectory_point = _currTrajectoryPoint
            if _startTime == 0.0:
                _startTime = time.time()
            if _endTime == 0.0:
                _endTime = _startTime + TRAJECTORY_POINT_DURATION
            if _prevTime == 0.0:
                _prevTime = time.time()

            currPedalPosXY = getPositionRightFoot()
            x_pedal_record.append(currPedalPosXY[0])
            y_pedal_record.append(currPedalPosXY[1])

            if _currTrajectoryPoint == initial_trajectory_point and past_initial_trajectory_point:
                print(len(pedalTrajectoryRight))
                print("Reached starting point")
                for pedal_pos in pedalTrajectoryRight:
                    plt.plot(pedal_pos[0], pedal_pos[1], '*')
                plt.plot(x_pedal_record, y_pedal_record)
                plt.show()
                past_initial_trajectory_point = False

            # Regulate update frequency
            _currTime = time.time()
            while float(float(_currTime) - float(_prevTime)) < (1 / CONTROLLER_FREQUENCY):
                time.sleep(1)
                currPedalPosXY = getPositionRightFoot()
                x_pedal_record.append(currPedalPosXY[0])
                y_pedal_record.append(currPedalPosXY[1])
                _currTime = time.time()
            _prevTime = _currTime

            # Check if trajectory point reached and act accordingly
            if PRINT_DEBUG:
                print("%0.5f" % (getDistance(getPositionRightFoot(), pedalTrajectoryRight[_currTrajectoryPoint])), end='\r')

            if jointsReachedGoal(_currTrajectoryPoint): #getDistance(getPositionRightFoot(), pedalTrajectoryRight[_currTrajectoryPoint]) <= PEDAL_POSITION_ERROR_TOLERANCE:  # and _currTime >= _endTime
                # getDistance(getPositionLeftFoot(), pedalTrajectoryLeft[_currTrajectoryPoint]) <= PEDAL_POSITION_ERROR_TOLERANCE and
                past_initial_trajectory_point = True
                if _currTrajectoryPoint < (numTrajectoryPoints - 1):
                    _currTrajectoryPoint += 1
                elif _currTrajectoryPoint >= (numTrajectoryPoints - 1):
                    _currTrajectoryPoint = 0
                if PRINT_DEBUG:
                    print("UPDATING TRAJECTORY POINT. NEW POINT: %s" % _currTrajectoryPoint)
                _startTime = time.time()
                _endTime = _startTime + TRAJECTORY_POINT_DURATION
                for thisJointName in _jointsList:
                    _jointsControlData[thisJointName]["trajectory_startpoint"] = getJointPosition(thisJointName)
                    _jointsControlData[thisJointName]["pos_error_integral"] = 0
                    _jointsControlData[thisJointName]["bool_update_iv"] = True

                # Iterate through joints and update setpoints
                for thisJointName in _jointsList:

                    thisJointPositionGoalpoint = 0.0
                    if thisJointName == RIGHT_HIP_JOINT:
                        thisJointPositionGoalpoint = hipTrajectoryRight[_currTrajectoryPoint]
                    elif thisJointName == RIGHT_KNEE_JOINT:
                        thisJointPositionGoalpoint = kneeTrajectoryRight[_currTrajectoryPoint]
                    elif thisJointName == RIGHT_ANKLE_JOINT:
                        thisJointPositionGoalpoint = ankleTrajectoryRight[_currTrajectoryPoint]
                    elif thisJointName == LEFT_HIP_JOINT:
                        thisJointPositionGoalpoint = hipTrajectoryLeft[_currTrajectoryPoint]
                    elif thisJointName == LEFT_KNEE_JOINT:
                        thisJointPositionGoalpoint = kneeTrajectoryLeft[_currTrajectoryPoint]
                    elif thisJointName == LEFT_ANKLE_JOINT:
                        thisJointPositionGoalpoint = ankleTrajectoryLeft[_currTrajectoryPoint]

                    _currTime = time.time()

                    #thisJointPositionSetpoint = interpolateTrajectoryPoints(_jointsControlData[thisJointName]["trajectory_startpoint"], thisJointPositionGoalpoint, _startTime, _currTime, _endTime)

                    thisJointVelocitySetpoint = computeVelocitySetpoint(thisJointName, thisJointPositionGoalpoint, _startTime, _currTime, _endTime)

                    if thisJointName == RIGHT_HIP_JOINT:
                        ros_right_hip_publisher.publish(thisJointVelocitySetpoint)
                    elif thisJointName == RIGHT_KNEE_JOINT:
                        ros_right_knee_publisher.publish(thisJointVelocitySetpoint)
                    elif thisJointName == RIGHT_ANKLE_JOINT:
                        ros_right_ankle_publisher.publish(thisJointVelocitySetpoint)
                    elif thisJointName == LEFT_HIP_JOINT:
                        ros_left_hip_publisher.publish(thisJointVelocitySetpoint)
                    elif thisJointName == LEFT_KNEE_JOINT:
                        ros_left_knee_publisher.publish(thisJointVelocitySetpoint)
                    elif thisJointName == LEFT_ANKLE_JOINT:
                        ros_left_ankle_publisher.publish(thisJointVelocitySetpoint)

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
