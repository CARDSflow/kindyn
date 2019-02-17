## @package pedaling

# roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='joint_hip_left joint_hip_right joint_wheel_right joint_wheel_back joint_pedal spine_joint joint_wheel_left joint_front joint_pedal_right joint_pedal_left elbow_right_rot1 joint_foot_left joint_knee_right joint_knee_left joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left_rot1 elbow_left_rot0 left_wrist_0 left_wrist_1 right_shoulder_axis0 right_shoulder_axis2 right_shoulder_axis1 elbow_right_rot0 right_wrist_0 right_wrist_1 head_axis0 head_axis1 head_axis2'

import json
import math
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
from scipy.misc import derivative

import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from roboy_control_msgs.srv import SetControllerParameters
from std_msgs.msg import Float32

#############################
###   MODULE PARAMETERS   ###
#############################

PRINT_DEBUG = True

RECORDED_TRAJECTORY_FILENAME = "capture_trajectory/desember_pedal_trajectory.json"

PEDAL_POSITION_ERROR_TOLERANCE = 0.02  # [meters]
PEDAL_ANGLE_ERROR_TOLERANCE = 0.02  # [meters]
JOINT_TRAJECTORY_ERROR_TOLERANCE = 0.02  # [rad]
PEDAL_SINGLE_ROTATION_DURATION = 40  # [seconds]
CONTROLLER_FREQUENCY = 100  # [Hz]
MIN_JOINT_VEL = -50
MAX_JOINT_VEL = 50
JOINT_VELOCITY_FACTOR = 1

PEDAL_CENTER_OFFSET_X = 0.20421
PEDAL_CENTER_OFFSET_Y = -0.00062
PEDAL_CENTER_OFFSET_Z = 0.2101

############################
###   GLOBAL VARIABLES   ###
############################

x_pedal_record = []
y_pedal_record = []

ROS_JOINT_HIP_RIGHT = "joint_hip_right"
ROS_JOINT_KNEE_RIGHT = "joint_knee_right"
ROS_JOINT_ANKLE_RIGHT = "joint_foot_right"
ROS_JOINT_HIP_LEFT = "joint_hip_left"
ROS_JOINT_KNEE_LEFT = "joint_knee_left"
ROS_JOINT_ANKLE_LEFT = "joint_foot_left"

RIGHT_HIP_JOINT = "right_hip"
RIGHT_KNEE_JOINT = "right_knee"
RIGHT_ANKLE_JOINT = "right_ankle"
LEFT_HIP_JOINT = "left_hip"
LEFT_KNEE_JOINT = "left_knee"
LEFT_ANKLE_JOINT = "left_ankle"

_jointsList = [RIGHT_HIP_JOINT, RIGHT_KNEE_JOINT, RIGHT_ANKLE_JOINT, LEFT_HIP_JOINT, LEFT_KNEE_JOINT, LEFT_ANKLE_JOINT]

_jointsListROS = [ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT, ROS_JOINT_HIP_LEFT,
                  ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT]

# _interpolate_x_dummy = [0.0, 0.5, 1.0, 1.5]
# _interpolate_z_dummy = [-0.5, 0.5, -0.5, 0.5]

_parametersRightHip = {
    "param_p": 150.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "pos_function": None  # interpolate.interp1d(_interpolate_x_dummy, _interpolate_z_dummy, kind = "cubic")
}

_parametersRightKnee = {
    "param_p": 200.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_vel": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "pos_function": None  # interpolate.interp1d(_interpolate_x_dummy, _interpolate_z_dummy, kind = "cubic")
}

_parametersRightAnkle = {
    "param_p": 100.0,
    "param_i": 0.0,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "pos_function": None  # interpolate.interp1d(_interpolate_x_dummy, _interpolate_z_dummy, kind = "cubic")
}

_parametersLeftHip = {
    "param_p": 150.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "pos_function": None  # interpolate.interp1d(_interpolate_x_dummy, _interpolate_z_dummy, kind = "cubic")
}

_parametersLeftKnee = {
    "param_p": 200.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "pos_function": None  # interpolate.interp1d(_interpolate_x_dummy, _interpolate_z_dummy, kind = "cubic")
}

_parametersLeftAnkle = {
    "param_p": 100.0,
    "param_i": 0.0,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "pos_function": None  # interpolate.interp1d(_interpolate_x_dummy, _interpolate_z_dummy, kind = "cubic")
}

_jointsControlData = {
    RIGHT_HIP_JOINT: _parametersRightHip,
    RIGHT_KNEE_JOINT: _parametersRightKnee,
    RIGHT_ANKLE_JOINT: _parametersRightAnkle,
    LEFT_HIP_JOINT: _parametersLeftHip,
    LEFT_KNEE_JOINT: _parametersLeftKnee,
    LEFT_ANKLE_JOINT: _parametersLeftAnkle
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

_numTrajectoryPoints = -1
_trajectoryStartingPoint = 0
_trajectoryPointDuration = 1.0
_pedalAngularVelocity = 0.1

_pedalTrajectoryRight = []
_pedalAngleTrajectoryRight = []
_hipTrajectoryRight = []
_kneeTrajectoryRight = []
_ankleTrajectoryRight = []
_pedalTrajectoryLeft = []
_pedalAngleTrajectoryLeft = []
_hipTrajectoryLeft = []
_kneeTrajectoryLeft = []
_ankleTrajectoryLeft = []


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
            print("Service call for " + thisJointName + "failed:" + e)

    print("Controller paramters updated")


def importJointTrajectoryRecord():
    global _numTrajectoryPoints
    global _pedalTrajectoryRight
    global _pedalAngleTrajectoryRight
    global _hipTrajectoryRight
    global _kneeTrajectoryRight
    global _ankleTrajectoryRight
    global _pedalTrajectoryLeft
    global _pedalAngleTrajectoryLeft
    global _hipTrajectoryLeft
    global _kneeTrajectoryLeft
    global _ankleTrajectoryLeft
    global PRINT_DEBUG

    with open(RECORDED_TRAJECTORY_FILENAME, "r") as read_file:
        loaded_data = json.load(read_file)

    if loaded_data["num_points"] is None:
        return 0
    else:
        _numTrajectoryPoints = loaded_data["num_points"]

    # Deleting previous trajectory before loading new
    del _pedalTrajectoryRight[:]
    del _pedalAngleTrajectoryRight[:]
    del _hipTrajectoryRight[:]
    del _kneeTrajectoryRight[:]
    del _ankleTrajectoryRight[:]
    del _pedalTrajectoryLeft[:]
    del _pedalAngleTrajectoryLeft[:]
    del _hipTrajectoryLeft[:]
    del _kneeTrajectoryLeft[:]
    del _ankleTrajectoryLeft[:]

    for pointIterator in range(_numTrajectoryPoints):
        if "point_" + str(pointIterator) in loaded_data:
            _pedalTrajectoryRight.append(loaded_data["point_" + str(pointIterator)]["Pedal"])
            # _pedalAngleTrajectoryRight.append(loaded_data["point_"+str(pointIterator)]["Pedal_angle"])
            _pedalAngleTrajectoryRight.append((2 * math.pi / _numTrajectoryPoints) * pointIterator)
            _hipTrajectoryRight.append(loaded_data["point_" + str(pointIterator)]["Hip"])
            _kneeTrajectoryRight.append((-1) * loaded_data["point_" + str(pointIterator)]["Knee"])
            _ankleTrajectoryRight.append(loaded_data["point_" + str(pointIterator)]["Ankle"])
            _pedalTrajectoryLeft.append(loaded_data["point_" + str(pointIterator)]["Pedal"])
            # _pedalAngleTrajectoryLeft.append(loaded_data["point_"+str(pointIterator)]["Pedal_angle"])
            _pedalAngleTrajectoryLeft.append((2 * math.pi / _numTrajectoryPoints) * pointIterator)
            _hipTrajectoryLeft.append(loaded_data["point_" + str(pointIterator)]["Hip"])
            _kneeTrajectoryLeft.append((-1) * loaded_data["point_" + str(pointIterator)]["Knee"])
            _ankleTrajectoryLeft.append(loaded_data["point_" + str(pointIterator)]["Ankle"])
        else:
            print("WARNING: No point_%s in trajectory" % pointIterator)
            _numTrajectoryPoints -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(_numTrajectoryPoints)


def getJointPosition(jointName):
    global _jointsStatusData
    return _jointsStatusData[jointName]["Pos"]


def getJointVelocity(jointName):
    global _jointsStatusData
    return _jointsStatusData[jointName]["Vel"]


def getPositionLeftFoot():
    fkJointNamesList = [ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT]
    fkJointPositions = [_jointsStatusData[LEFT_HIP_JOINT]["Pos"], _jointsStatusData[LEFT_KNEE_JOINT]["Pos"],
                        _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_left_tip", "foot_left_tip", fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException as e:
        print("Service call failed:", e)

    print("ERROR fk foot_left failed")
    return [0.0, 0.0]  # [x, z]


def getPositionRightFoot():
    fkJointNamesList = [ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT]
    fkJointPositions = [_jointsStatusData[RIGHT_HIP_JOINT]["Pos"], _jointsStatusData[RIGHT_KNEE_JOINT]["Pos"],
                        _jointsStatusData[RIGHT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_right_tip", "foot_right_tip", fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException as e:
        print("Service call failed:", e)

    print("ERROR fk foot_right failed")
    return [0.0, 0.0]  # [x, z]


def getDistance(point1, point2):
    x_diff = point2[0] - point1[0]
    y_diff = point2[1] - point1[1]

    return math.sqrt((x_diff * x_diff) + (y_diff * y_diff))


def setPedalSingleRotationDuration(new_duration_seconds):
    global PEDAL_SINGLE_ROTATION_DURATION
    PEDAL_SINGLE_ROTATION_DURATION = new_duration_seconds
    setTrajectoryPointDuration()
    setPedalAngularVelocity()
    return 1


def setTrajectoryPointDuration():
    global _trajectoryPointDuration
    global PEDAL_SINGLE_ROTATION_DURATION
    global _numTrajectoryPoints
    if _numTrajectoryPoints > 0:
        _trajectoryPointDuration = float(PEDAL_SINGLE_ROTATION_DURATION) / _numTrajectoryPoints
    else:
        print("ERROR: trajectory point duration can not be calculated without number of points count")
        return 0
    return 1


def setPedalAngularVelocity():
    global _pedalAngularVelocity
    global PEDAL_SINGLE_ROTATION_DURATION
    if PEDAL_SINGLE_ROTATION_DURATION > 0:
        _pedalAngularVelocity = float(PEDAL_SINGLE_ROTATION_DURATION) / (2 * math.pi)
        print("Pedal angular velocity: ", _pedalAngularVelocity)
    else:
        print("ERROR: trajectory point duration can not be calculated without number of points count")
        return 0
    return 1


def getCurrentAngle(current_point):
    current_x = current_point[0] - PEDAL_CENTER_OFFSET_X
    current_y = current_point[1] - PEDAL_CENTER_OFFSET_Y

    if current_x > 0 and current_y > 0:
        return np.arctan(current_y / current_x)
    elif current_x < 0 < current_y:
        return np.arctan(current_y / current_x) + np.pi
    elif current_x < 0 and current_y < 0:
        return np.arctan(current_y / current_x) + np.pi
    elif current_x > 0 > current_y:
        return np.arctan(current_y / current_x) + 2 * np.pi

    elif current_x == 0 and current_y > 0:
        return np.pi / 2
    elif current_x == 0 and current_y < 0:
        return np.pi * 3 / 2
    elif current_x > 0 and current_y == 0:
        return 0
    elif current_x < 0 and current_y == 0:
        return np.pi


def interpolateAllJointPositions():
    global _pedalAngleTrajectoryRight
    global _hipTrajectoryRight
    global _kneeTrajectoryRight
    global _ankleTrajectoryRight

    global _pedalAngleTrajectoryLeft
    global _hipTrajectoryLeft
    global _kneeTrajectoryLeft
    global _ankleTrajectoryLeft

    global _jointsControlData
    global RIGHT_HIP_JOINT
    global RIGHT_KNEE_JOINT
    global RIGHT_ANKLE_JOINT
    global LEFT_HIP_JOINT
    global LEFT_KNEE_JOINT
    global LEFT_ANKLE_JOINT

    _jointsControlData[RIGHT_HIP_JOINT]["pos_function"] = interpolate.interp1d(_pedalAngleTrajectoryRight,
                                                                               _hipTrajectoryRight, kind="cubic")
    _jointsControlData[RIGHT_KNEE_JOINT]["pos_function"] = interpolate.interp1d(_pedalAngleTrajectoryRight,
                                                                                _kneeTrajectoryRight, kind="cubic")
    _jointsControlData[RIGHT_ANKLE_JOINT]["pos_function"] = interpolate.interp1d(_pedalAngleTrajectoryRight,
                                                                                 _ankleTrajectoryRight, kind="cubic")
    _jointsControlData[LEFT_HIP_JOINT]["pos_function"] = interpolate.interp1d(_pedalAngleTrajectoryRight,
                                                                              _hipTrajectoryLeft, kind="cubic")
    _jointsControlData[LEFT_KNEE_JOINT]["pos_function"] = interpolate.interp1d(_pedalAngleTrajectoryRight,
                                                                               _kneeTrajectoryLeft, kind="cubic")
    _jointsControlData[LEFT_ANKLE_JOINT]["pos_function"] = interpolate.interp1d(_pedalAngleTrajectoryRight,
                                                                                _ankleTrajectoryLeft, kind="cubic")

    return 1


def printInterpolatedFunctions():
    global _pedalAngleTrajectoryRight
    global _hipTrajectoryRight
    global _kneeTrajectoryRight
    global _ankleTrajectoryRight

    global _pedalAngleTrajectoryLeft
    global _hipTrajectoryLeft
    global _kneeTrajectoryLeft
    global _ankleTrajectoryLeft

    global _jointsControlData
    global RIGHT_HIP_JOINT
    global RIGHT_KNEE_JOINT
    global RIGHT_ANKLE_JOINT
    global LEFT_HIP_JOINT
    global LEFT_KNEE_JOINT
    global LEFT_ANKLE_JOINT

    highDefPlotRange = np.linspace(0, 6, 500)

    plt.figure(1)
    plt.plot(_pedalAngleTrajectoryRight, _hipTrajectoryRight, '*')
    plt.plot(highDefPlotRange, _jointsControlData[RIGHT_HIP_JOINT]["pos_function"](highDefPlotRange), '-')

    plt.figure(2)
    plt.plot(_pedalAngleTrajectoryRight, _kneeTrajectoryRight, '*')
    plt.plot(highDefPlotRange, _jointsControlData[RIGHT_KNEE_JOINT]["pos_function"](highDefPlotRange), '-')

    plt.figure(3)
    plt.plot(_pedalAngleTrajectoryRight, _ankleTrajectoryRight, '*')
    plt.plot(highDefPlotRange, _jointsControlData[RIGHT_ANKLE_JOINT]["pos_function"](highDefPlotRange), '-')

    plt.show()


#############################
###   CONTROL FUNCTIONS   ###
#############################

def checkOutputLimits(inputVal):
    returnVal = inputVal

    if inputVal > MAX_JOINT_VEL:
        returnVal = MAX_JOINT_VEL
    elif inputVal < MIN_JOINT_VEL:
        returnVal = MIN_JOINT_VEL

    return returnVal


#########################
###   STATE MACHINE   ###
#########################

def FSM():
    global _numTrajectoryPoints
    global _jointsControlData
    global _jointsList
    global PRINT_DEBUG

    # Params for inspection of performance (temp)
    global x_pedal_record
    global y_pedal_record
    global _pedalTrajectoryRight
    initialTrajectoryPoint = 0
    pastInitialTrajectoryPoint = False

    global PEDAL_SINGLE_ROTATION_DURATION
    global _pedalAngularVelocity

    INIT = "INIT"
    PEDAL = "PEDAL"
    UPDATE_PARAMETERS = "UPDATE_PARAMETERS"

    runFSM = True
    currState = INIT
    currTrajectoryPoint = None
    prevTrajectoryPoint = None

    startTime = 0.0
    endTime = 0.0
    currTime = 0.0
    prevTime = 0.0

    ros_right_hip_publisher = rospy.Publisher('/joint_hip_right/joint_hip_right/target', Float32, queue_size=2)
    ros_right_knee_publisher = rospy.Publisher('/joint_knee_right/joint_knee_right/target', Float32, queue_size=2)
    ros_right_ankle_publisher = rospy.Publisher('/joint_foot_right/joint_foot_right/target', Float32, queue_size=2)

    ros_left_hip_publisher = rospy.Publisher('/joint_hip_left/joint_hip_left/target', Float32, queue_size=2)
    ros_left_knee_publisher = rospy.Publisher('/joint_knee_left/joint_knee_left/target', Float32, queue_size=2)
    ros_left_ankle_publisher = rospy.Publisher('/joint_foot_left/joint_foot_left/target', Float32, queue_size=2)

    while runFSM:

        ##############################################
        if currState == INIT:
            ##############################################

            importJointTrajectoryRecord()
            setPedalSingleRotationDuration(PEDAL_SINGLE_ROTATION_DURATION)
            setPedalAngularVelocity()
            interpolateAllJointPositions()

            # printInterpolatedFunctions()

            # Find starting point on the trajectory

            currState = PEDAL

        ##############################################
        if currState == PEDAL:
            ##############################################

            # Initialize state
            if currTrajectoryPoint == None:
                currTrajectoryPoint = _trajectoryStartingPoint
                prevTrajectoryPoint = currTrajectoryPoint
                initialTrajectoryPoint = currTrajectoryPoint
            if startTime == 0.0:
                startTime = time.time()
            if endTime == 0.0:
                endTime = startTime + _trajectoryPointDuration
            if prevTime == 0.0:
                prevTime = time.time()

            currPedalPosXY = getPositionRightFoot()
            x_pedal_record.append(currPedalPosXY[0])
            y_pedal_record.append(currPedalPosXY[1])

            if currTrajectoryPoint == initialTrajectoryPoint and pastInitialTrajectoryPoint:
                print(len(_pedalTrajectoryRight))
                print("Reached starting point")
                for pedal_pos in _pedalTrajectoryRight:
                    plt.plot(pedal_pos[0], pedal_pos[1], '*')
                plt.plot(x_pedal_record, y_pedal_record)
                plt.show()
                pastInitialTrajectoryPoint = False

            # Regulate update frequency
            currTime = time.time()
            while float(float(currTime) - float(prevTime)) < (1 / CONTROLLER_FREQUENCY):
                time.sleep(1)
                currPedalPosXY = getPositionRightFoot()
                x_pedal_record.append(currPedalPosXY[0])
                y_pedal_record.append(currPedalPosXY[1])
                currTime = time.time()
            prevTime = currTime

            # Check if trajectory point reached and act accordingly
            if PRINT_DEBUG:
                print("Distance to target: Right foot %0.5f, left foot %0.5f" % (
                    getDistance(getPositionRightFoot(), _pedalTrajectoryRight[currTrajectoryPoint]),
                    getDistance(getPositionLeftFoot(), _pedalTrajectoryLeft[currTrajectoryPoint])), end='\r')

            if (getDistance(getPositionRightFoot(), _pedalTrajectoryRight[
                currTrajectoryPoint]) <= PEDAL_POSITION_ERROR_TOLERANCE and getDistance(getPositionLeftFoot(),
                                                                                        _pedalTrajectoryLeft[
                                                                                            currTrajectoryPoint]) <= PEDAL_POSITION_ERROR_TOLERANCE and currTime >= endTime):
                pastInitialTrajectoryPoint = True
                prevTrajectoryPoint = currTrajectoryPoint
                if currTrajectoryPoint < (_numTrajectoryPoints - 1):
                    currTrajectoryPoint += 1
                elif currTrajectoryPoint >= (_numTrajectoryPoints - 1):
                    currTrajectoryPoint = 0
                if PRINT_DEBUG:
                    print("UPDATING TRAJECTORY POINT. NEW POINT: %s" % currTrajectoryPoint)
                startTime = time.time()
                endTime = startTime + _trajectoryPointDuration
                for thisJointName in _jointsList:
                    _jointsControlData[thisJointName]["pos_error_integral"] = 0

            # Iterate through joints and update setpoints
            for thisJointName in _jointsList:

                rightSide = False
                leftSide = False

                thisJointPositionGoalpoint = None
                prevJointPositionGoalpoint = None
                if thisJointName == RIGHT_HIP_JOINT:
                    thisJointPositionGoalpoint = _hipTrajectoryRight[currTrajectoryPoint]
                    prevJointPositionGoalpoint = _hipTrajectoryRight[prevTrajectoryPoint]
                    rightSide = True
                elif thisJointName == RIGHT_KNEE_JOINT:
                    thisJointPositionGoalpoint = _kneeTrajectoryRight[currTrajectoryPoint]
                    prevJointPositionGoalpoint = _kneeTrajectoryRight[prevTrajectoryPoint]
                    rightSide = True
                elif thisJointName == RIGHT_ANKLE_JOINT:
                    thisJointPositionGoalpoint = _ankleTrajectoryRight[currTrajectoryPoint]
                    prevJointPositionGoalpoint = _ankleTrajectoryRight[prevTrajectoryPoint]
                    rightSide = True
                elif thisJointName == LEFT_HIP_JOINT:
                    thisJointPositionGoalpoint = _hipTrajectoryLeft[currTrajectoryPoint]
                    prevJointPositionGoalpoint = _hipTrajectoryLeft[prevTrajectoryPoint]
                    leftSide = True
                elif thisJointName == LEFT_KNEE_JOINT:
                    thisJointPositionGoalpoint = _kneeTrajectoryLeft[currTrajectoryPoint]
                    prevJointPositionGoalpoint = _kneeTrajectoryLeft[prevTrajectoryPoint]
                    leftSide = True
                elif thisJointName == LEFT_ANKLE_JOINT:
                    thisJointPositionGoalpoint = _ankleTrajectoryLeft[currTrajectoryPoint]
                    prevJointPositionGoalpoint = _ankleTrajectoryLeft[prevTrajectoryPoint]
                    leftSide = True

                thisJointVelocitySetpoint = None
                # USE DERIVATIVE OF INTERPOLATED JOINT ANGLE FUNCTION
                if rightSide:
                    # currGoalPedalAngle = _pedalTrajectoryRight[currTrajectoryPoint] + (time.time() - startTime)*_pedalAngularVelocity
                    # currPedalError = currGoalPedalAngle - getCurrentAngle(getPositionRightFoot())
                    timeSpent = (time.time() - startTime)
                    if timeSpent > endTime - startTime:
                        timeSpent = endTime - startTime
                    currGoalJointAngle = prevJointPositionGoalpoint + timeSpent * _pedalAngularVelocity
                    currJointError = currGoalJointAngle - getJointPosition(thisJointName)
                    if abs(currJointError) > JOINT_TRAJECTORY_ERROR_TOLERANCE:
                        thisJointVelocitySetpoint = _jointsControlData[thisJointName]["param_p"] * currJointError
                    else:
                        thisJointVelocitySetpoint = derivative(func=_jointsControlData[thisJointName]["pos_function"],
                                                               x0=float(getCurrentAngle(getPositionRightFoot())),
                                                               dx=1e-6) * _pedalAngularVelocity
                elif leftSide:
                    currGoalJointAngle = prevJointPositionGoalpoint + (time.time() - startTime) * _pedalAngularVelocity
                    currJointError = currGoalJointAngle - getJointPosition(thisJointName)
                    if abs(currJointError) > JOINT_TRAJECTORY_ERROR_TOLERANCE:
                        thisJointVelocitySetpoint = _jointsControlData[thisJointName]["param_p"] * currJointError
                    else:
                        thisJointVelocitySetpoint = derivative(func=_jointsControlData[thisJointName]["pos_function"],
                                                               x0=float(getCurrentAngle(getPositionLeftFoot())),
                                                               dx=1e-6) * _pedalAngularVelocity

                thisJointVelocitySetpoint = checkOutputLimits(thisJointVelocitySetpoint)
                # thisJointVelocitySetpoint = thisJointVelocitySetpoint*(-1)

                # print("Velocity setpoint for ", thisJointName, ": ", thisJointVelocitySetpoint)

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
        # if currState == UPDATE_PARAMETERS:
        ##############################################

        # Reload trajectory and PID parameters

    return 1


################
###   MAIN   ###
################


def main():
    rospy.init_node('pedal_simulation', anonymous=True)
    rospy.Subscriber("joint_state", JointState, jointStateCallback)
    time.sleep(3)

    setJointControllerParameters(1, 0)
    FSM()

    return 1


if __name__ == '__main__':
    main()
