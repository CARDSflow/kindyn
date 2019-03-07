#!/usr/bin/env python
from __future__ import print_function

#roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='joint_hip_left joint_hip_right joint_wheel_right joint_wheel_back joint_pedal spine_joint joint_wheel_left joint_front joint_pedal_right joint_pedal_left elbow_right_rot1 joint_foot_left joint_knee_right joint_knee_left joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left_rot1 elbow_left_rot0 left_wrist_0 left_wrist_1 right_shoulder_axis0 right_shoulder_axis2 right_shoulder_axis1 elbow_right_rot0 right_wrist_0 right_wrist_1 head_axis0 head_axis1 head_axis2'


# Measurements:
# Lennards Computer: publish 1.0 ==> ~0.0018 rad/s with Kp: 0.1 & Forward Kinematics: 0.01
# Roboy Computer: publish 1.0 ==> ~0.008 rad/s with Kp: 0.1 & Forward Kinematics: 0.01

import json
import math
import time
from threading import Thread

from scipy import interpolate
from scipy.misc import derivative
import numpy as np

import matplotlib.pyplot as plt

import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32

#############################
###   MODULE PARAMETERS   ###
#############################

PRINT_DEBUG = False

RECORDED_TRAJECTORY_FILENAME = "capture_trajectory/old_captured_trajectory.json"

PEDAL_POSITION_ERROR_TOLERANCE = 0.02  # [meters]
JOINT_TRAJECTORY_ERROR_TOLERANCE = 0.02
PEDAL_SINGLE_ROTATION_DURATION = 20  # [seconds]
NUMBER_CIRCULATION_POINTS = 72
TRAJECTORY_POINT_DURATION = 10
CONTROLLER_FREQUENCY = 100  # [Hz]
MIN_JOINT_VEL = -500
MAX_JOINT_VEL = 500
JOINT_VELOCITY_FACTOR = 1000

############################
###   GLOBAL VARIABLES   ###
############################

x_pedal_record = [ ]
y_pedal_record = [ ]

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

f_interpolated_hip_right = None
f_interpolated_hip_left = None
f_interpolated_knee_right = None
f_interpolated_knee_left = None
f_interpolated_ankle_right = None
f_interpolated_ankle_left = None
f_interpolated_pedal_angle = None

velocity_error_factor_hip = 1.0
velocity_error_factor_knee = 1.0
velocity_error_factor_ankle = 1.0

ros_right_hip_publisher = rospy.Publisher('/joint_hip_right/joint_hip_right/target', Float32, queue_size=2)
ros_right_knee_publisher = rospy.Publisher('/joint_knee_right/joint_knee_right/target', Float32, queue_size=2)
ros_right_ankle_publisher = rospy.Publisher('/joint_foot_right/joint_foot_right/target', Float32, queue_size=2)

ros_left_hip_publisher = rospy.Publisher('/joint_hip_left/joint_hip_left/target', Float32, queue_size=2)
ros_left_knee_publisher = rospy.Publisher('/joint_knee_left/joint_knee_left/target', Float32, queue_size=2)
ros_left_ankle_publisher = rospy.Publisher('/joint_foot_left/joint_foot_left/target', Float32, queue_size=2)

PEDAL_CENTER_OFFSET_X = 0.20421
PEDAL_CENTER_OFFSET_Y = -0.00062
PEDAL_CENTER_OFFSET_Z = 0.2101

_jointsList = [ RIGHT_HIP_JOINT, RIGHT_KNEE_JOINT, RIGHT_ANKLE_JOINT, LEFT_HIP_JOINT, LEFT_KNEE_JOINT,
                LEFT_ANKLE_JOINT ]

_parametersRightHip = {
    "param_p": 1500.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersRightKnee = {
    "param_p": 2000.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersRightAnkle = {
    "param_p": 1000.0,
    "param_i": 0.0,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersLeftHip = {
    "param_p": 1500.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersLeftKnee = {
    "param_p": 2000.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersLeftAnkle = {
    "param_p": 1000.0,
    "param_i": 0.0,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
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

number_imported_trajectory_points = -1
trajectoryStartingPoint = 0

pedalTrajectoryRight = [ ]
pedalAngleTrajectoryRight = []
pedalTrajectoryLeft = [ ]
hipTrajectoryRight = [ ]
kneeTrajectoryRight = [ ]
ankleTrajectoryRight = [ ]
hipTrajectoryLeft = [ ]
kneeTrajectoryLeft = [ ]
ankleTrajectoryLeft = [ ]


##############################
###   UTILITY FUNCTIONS   ###
##############################


def jointStateCallback(joint_data):
    global _jointsStatusData
    # Assert order of joints
    for stringIter in range(len(joint_data.names)):
        if joint_data.names[ stringIter ] == ROS_JOINT_HIP_RIGHT:
            _jointsStatusData[ RIGHT_HIP_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            _jointsStatusData[ RIGHT_HIP_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_HIP_LEFT:
            _jointsStatusData[ LEFT_HIP_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            _jointsStatusData[ LEFT_HIP_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_KNEE_RIGHT:
            _jointsStatusData[ RIGHT_KNEE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            _jointsStatusData[ RIGHT_KNEE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_KNEE_LEFT:
            _jointsStatusData[ LEFT_KNEE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            _jointsStatusData[ LEFT_KNEE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_ANKLE_RIGHT:
            _jointsStatusData[ RIGHT_ANKLE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            _jointsStatusData[ RIGHT_ANKLE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_ANKLE_LEFT:
            _jointsStatusData[ LEFT_ANKLE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            _jointsStatusData[ LEFT_ANKLE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]


def importJointTrajectoryRecord():
    global number_imported_trajectory_points
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

    if loaded_data[ "num_points" ] is None:
        return 0
    else:
        number_imported_trajectory_points = loaded_data[ "num_points" ]

    # Deleting previous trajectory before loading new
    del pedalTrajectoryLeft[ : ]
    del pedalTrajectoryRight[ : ]
    del pedalAngleTrajectoryRight[ : ]
    del hipTrajectoryRight[ : ]
    del kneeTrajectoryRight[ : ]
    del ankleTrajectoryRight[ : ]
    del hipTrajectoryLeft[ : ]
    del kneeTrajectoryLeft[ : ]
    del ankleTrajectoryLeft[ : ]
    for pointIterator in range(number_imported_trajectory_points):
        if "point_" + str(pointIterator) in loaded_data:
            pedalTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Pedal" ])
            pedalTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Pedal" ])
            pedalAngleTrajectoryRight.append(loaded_data["point_"+str(pointIterator)]["Right"]["Pedal_angle"])
            hipTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Hip" ])
            kneeTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Knee" ])
            ankleTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Ankle" ])
            hipTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Hip" ])
            kneeTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Knee" ])
            ankleTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Ankle" ])
        else:
            print("WARNING: No point_%s in trajectory" % pointIterator)
            number_imported_trajectory_points -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(number_imported_trajectory_points)


def getJointPosition(jointName):
    global _jointsStatusData
    return _jointsStatusData[ jointName ][ "Pos" ]


def getJointVelocity(jointName):
    global _jointsStatusData
    return _jointsStatusData[ jointName ][ "Vel" ]


def getPosition(endeffector, frame):
    fkJointNamesList = [ ROS_JOINT_HIP_RIGHT, ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_KNEE_LEFT,
                         ROS_JOINT_ANKLE_RIGHT, ROS_JOINT_ANKLE_LEFT ]
    fkJointPositions = [ _jointsStatusData[ RIGHT_HIP_JOINT ][ "Pos" ], _jointsStatusData[ LEFT_HIP_JOINT ][ "Pos" ],
                         _jointsStatusData[ RIGHT_KNEE_JOINT ][ "Pos" ], _jointsStatusData[ LEFT_KNEE_JOINT ][ "Pos" ],
                         _jointsStatusData[ RIGHT_ANKLE_JOINT ][ "Pos" ],
                         _jointsStatusData[ LEFT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv(endeffector, frame, fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)
    return [ 0.0, 0.0 ]  # [x, z]


def getPositionLeftFoot():
    fkJointNamesList = [ ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT ]
    fkJointPositions = [ _jointsStatusData[ LEFT_HIP_JOINT ][ "Pos" ], _jointsStatusData[ LEFT_KNEE_JOINT ][ "Pos" ],
                         _jointsStatusData[ LEFT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_left_tip", "foot_left_tip", fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    print("ERROR fk foot_left failed")
    return [ 0.0, 0.0 ]  # [x, z]


def getPositionRightFoot():
    fkJointNamesList = [ ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT ]
    fkJointPositions = [ _jointsStatusData[ RIGHT_HIP_JOINT ][ "Pos" ], _jointsStatusData[ RIGHT_KNEE_JOINT ][ "Pos" ],
                         _jointsStatusData[ RIGHT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_right_tip", "foot_right_tip", fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    print("ERROR fk foot_right failed")
    return [ 0.0, 0.0 ]  # [x, z]


def getDistance(point1, point2):
    x_diff = point2[ 0 ] - point1[ 0 ]
    y_diff = point2[ 1 ] - point1[ 1 ]

    return math.sqrt((x_diff * x_diff) + (y_diff * y_diff))


def setPedalSingleRotationDuration(new_duration_seconds):
    global PEDAL_SINGLE_ROTATION_DURATION
    PEDAL_SINGLE_ROTATION_DURATION = new_duration_seconds
    setTrajectoryPointDuration()
    return 1


def setTrajectoryPointDuration():
    global TRAJECTORY_POINT_DURATION
    global PEDAL_SINGLE_ROTATION_DURATION
    global NUMBER_CIRCULATION_POINTS
    if NUMBER_CIRCULATION_POINTS != 0:
        TRAJECTORY_POINT_DURATION = float(PEDAL_SINGLE_ROTATION_DURATION) / NUMBER_CIRCULATION_POINTS
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
    return value1 + (value2 - value1) * (float(currTime - startTime) / (endTime - startTime))


def checkOutputLimits(inputVal):
    returnVal = inputVal

    if inputVal > MAX_JOINT_VEL:
        returnVal = MAX_JOINT_VEL
    elif inputVal < MIN_JOINT_VEL:
        returnVal = MIN_JOINT_VEL

    return returnVal


def computeVelocitySetpoint(jointName, next_joint_angle, current_joint_angle, startTime, currTime, endTime):
    global _jointsControlData
    global PRINT_DEBUG
    global JOINT_TRAJECTORY_ERROR_TOLERANCE
    global JOINT_VELOCITY_FACTOR

    current_point = getJointPosition(jointName)
    joint_difference = next_joint_angle - current_joint_angle

    if _jointsControlData[ jointName ][ "bool_update_iv" ]:
        if currTime < endTime:
            jointTravelTime = endTime - currTime
            jointTravelDistance = joint_difference
            jointIdealVelocity = float(jointTravelDistance) / jointTravelTime
            _jointsControlData[ jointName ][ "ideal_velocity" ] = jointIdealVelocity
            _jointsControlData[ jointName ][ "bool_update_iv" ] = False
        else:
            print("ERROR in compute ideal joint velocity: currTime > endTime")

    thisPosErrorDerivative = float(
        joint_difference - _jointsControlData[ jointName ][ "prev_error" ]) / CONTROLLER_FREQUENCY

    # print("COMPUTED %s VELOCITY SETPOINT: %s (jointTravelTime: %s, jointTravelDistance: %s)" % (jointName, jointVelocityReachGoal, jointTravelTime, jointTravelDistance))

    thisReturnVal = _jointsControlData[ jointName ][ "ideal_velocity" ]
    thisReturnVal = thisReturnVal * JOINT_VELOCITY_FACTOR
    # SWITCH CONTROL MODE IF STATEMENT IS TRUE (FROM IDEAL VELOCITY TO PID POSITION ERROR)
    if currTime > endTime:  # jointError > JOINT_TRAJECTORY_ERROR_TOLERANCE or
        if PRINT_DEBUG:
            print("Switching control to PID for joint %s" % jointName)
        _jointsControlData[ jointName ][ "pos_error_integral" ] += float(joint_difference) / CONTROLLER_FREQUENCY
        _jointsControlData[ jointName ][ "prev_time" ] = currTime
        _jointsControlData[ jointName ][ "prev_pos" ] = current_point
        _jointsControlData[ jointName ][ "prev_error" ] = joint_difference
        thisReturnVal = _jointsControlData[ jointName ][ "param_p" ] * joint_difference + \
                        _jointsControlData[ jointName ][ "param_i" ] * _jointsControlData[ jointName ][
                            "pos_error_integral" ] + _jointsControlData[ jointName ][
                            "param_d" ] * thisPosErrorDerivative

    thisReturnVal = checkOutputLimits(thisReturnVal)

    if PRINT_DEBUG:
        if jointName == RIGHT_HIP_JOINT:
            print("\t\t\t\t%0.5f\t\t\t\t%0.5f" % (joint_difference, thisReturnVal), end='\r')
        elif jointName == RIGHT_KNEE_JOINT:
            print("\t\t\t\t\t\t\t\t\t\t\t\t%0.5f\t\t\t\t%0.5f" % (joint_difference, thisReturnVal), end='\r')

    return thisReturnVal


def get_joint_angle(thisJointName, trajectory_angle):

    if thisJointName == RIGHT_HIP_JOINT:
        return f_interpolated_hip_right(trajectory_angle)
    elif thisJointName == RIGHT_KNEE_JOINT:
        return f_interpolated_knee_right(trajectory_angle)
    elif thisJointName == RIGHT_ANKLE_JOINT:
        return f_interpolated_ankle_right(trajectory_angle)
    elif thisJointName == LEFT_HIP_JOINT:
        return f_interpolated_hip_left(trajectory_angle)
    elif thisJointName == LEFT_KNEE_JOINT:
        return f_interpolated_knee_left(trajectory_angle)
    elif thisJointName == LEFT_ANKLE_JOINT:
        return f_interpolated_ankle_left(trajectory_angle)


def interpolate_functions():
    global f_interpolated_hip_right
    global f_interpolated_hip_left
    global f_interpolated_knee_right
    global f_interpolated_knee_left
    global f_interpolated_ankle_right
    global f_interpolated_ankle_left
    global f_interpolated_pedal_angle

    f_interpolated_hip_right = interpolate.interp1d(pedalAngleTrajectoryRight, hipTrajectoryRight, kind="cubic")
    f_interpolated_knee_right = interpolate.interp1d(pedalAngleTrajectoryRight, kneeTrajectoryRight, kind="cubic")
    f_interpolated_ankle_right = interpolate.interp1d(pedalAngleTrajectoryRight, ankleTrajectoryRight, kind="cubic")
    f_interpolated_hip_left = interpolate.interp1d(pedalAngleTrajectoryRight, hipTrajectoryLeft, kind="cubic")
    f_interpolated_knee_left = interpolate.interp1d(pedalAngleTrajectoryRight, kneeTrajectoryLeft, kind="cubic")
    f_interpolated_ankle_left = interpolate.interp1d(pedalAngleTrajectoryRight, ankleTrajectoryLeft, kind="cubic")


def evaluate_current_angle(current_point):
    current_x = current_point[ 0 ] - PEDAL_CENTER_OFFSET_X
    current_y = current_point[ 1 ] - PEDAL_CENTER_OFFSET_Y

    print(current_x)
    print(current_y)

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


def publish_velocity(thisJointName, next_joint_angle, current_joint_angle, _startTime, _currTime, _endTime):
    thisJointVelocitySetpoint = computeVelocitySetpoint(thisJointName, next_joint_angle, current_joint_angle,
                                                        _startTime, _currTime, _endTime)

    publisher = None
    factor = 1

    if thisJointName == RIGHT_HIP_JOINT:
        publisher = ros_right_hip_publisher
        factor = velocity_error_factor_hip
    elif thisJointName == RIGHT_KNEE_JOINT:
        publisher = ros_right_knee_publisher
        factor = velocity_error_factor_knee
    elif thisJointName == RIGHT_ANKLE_JOINT:
        publisher = ros_right_ankle_publisher
        factor = velocity_error_factor_ankle
    elif thisJointName == LEFT_HIP_JOINT:
        publisher = ros_left_hip_publisher
        factor = velocity_error_factor_hip
    elif thisJointName == LEFT_KNEE_JOINT:
        publisher = ros_left_knee_publisher
        factor = velocity_error_factor_knee
    elif thisJointName == LEFT_ANKLE_JOINT:
        publisher = ros_left_ankle_publisher
        factor = velocity_error_factor_ankle

    begin_time = rospy.get_rostime()
    duration = rospy.Duration(TRAJECTORY_POINT_DURATION)
    end_time = duration + begin_time
    while rospy.get_rostime() < end_time:
        publisher.publish(thisJointVelocitySetpoint * factor)
        rospy.sleep(rospy.Duration(0, 1000000))


#########################
###   STATE MACHINE   ###
#########################

INIT = "INIT"
PEDAL = "PEDAL"
UPDATE_PARAMETERS = "UPDATE_PARAMETERS"


def FSM():
    global NUMBER_CIRCULATION_POINTS
    global _jointsControlData
    global _jointsList
    global PRINT_DEBUG

    global x_pedal_record
    global y_pedal_record
    global pedalTrajectoryRight

    global velocity_error_factor_hip
    global velocity_error_factor_knee
    global velocity_error_factor_ankle
    global velocity_error_factor_hip
    global velocity_error_factor_knee
    global velocity_error_factor_ankle

    _runFSM = True

    _currState = INIT
    current_angle_hip = _jointsStatusData[LEFT_HIP_JOINT]["Pos"]
    current_angle_knee = _jointsStatusData[LEFT_KNEE_JOINT]["Pos"]
    current_angle_ankle = _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"]

    _startTime = 0.0
    _endTime = 0.0
    _currTime = 0.0
    _prevTime = 0.0

    velocity = 1
    publisher = ros_left_hip_publisher

    begin_time = rospy.get_rostime()
    duration = rospy.Duration(10)
    end_time = duration + begin_time

    while rospy.get_rostime() < end_time:
        publisher.publish(velocity)
        print (_jointsStatusData[LEFT_HIP_JOINT]["Pos"])

    publisher.publish(0)
    final_angle_hip = _jointsStatusData[ LEFT_HIP_JOINT ][ "Pos" ]
    final_angle_knee = _jointsStatusData[ LEFT_KNEE_JOINT ][ "Pos" ]
    final_angle_ankle = _jointsStatusData[ LEFT_ANKLE_JOINT ][ "Pos" ]

    print ("velocity = ", velocity, "\ntime (ns) = ", duration,
           "\n\nangle 1 = ", current_angle_hip, "\nangle 2 = ", final_angle_hip, "\nd = ", final_angle_hip - current_angle_hip)


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
