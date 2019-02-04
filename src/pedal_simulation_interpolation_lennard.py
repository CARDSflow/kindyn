#!/usr/bin/env python
from __future__ import print_function

# roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='joint_hip_left joint_hip_right joint_wheel_right joint_wheel_back joint_pedal spine_joint joint_wheel_left joint_front joint_pedal_right joint_pedal_left elbow_right_rot1 joint_foot_left joint_knee_right joint_knee_left joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left_rot1 elbow_left_rot0 left_wrist_0 left_wrist_1 right_shoulder_axis0 right_shoulder_axis2 right_shoulder_axis1 elbow_right_rot0 right_wrist_0 right_wrist_1 head_axis0 head_axis1 head_axis2'
#

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

PRINT_DEBUG = True
SIMULATION_FACTOR = 100  # factor to slow down the motion for better simulation
NUMBER_CIRCULATION_POINTS = 30  # number of points for controlling


RECORDED_TRAJECTORY_FILENAME = "capture_trajectory/captured_trajectory_old.json"

BIKE_VELOCITY = 0

PEDAL_POSITION_ERROR_TOLERANCE = 0.02  # [meters]
JOINT_TRAJECTORY_ERROR_TOLERANCE = 0.02
PEDAL_SINGLE_ROTATION_DURATION = 0  # [seconds]
TRAJECTORY_POINT_DURATION = 0
CONTROLLER_FREQUENCY = 100  # [Hz]
MIN_JOINT_VEL = -500
MAX_JOINT_VEL = 500
JOINT_VELOCITY_FACTOR_SIMULATION = 0.01  # publish 1 => velocity = 0.01 rad/s  for Kp = 0.1 and simulation-step-length = 0.01

RADIUS_BACK_TIRE = 0.294398  # in m
RADIUS_GEAR_CLUSTER = 0.06  # in m
RADIUS_FRONT_CHAIN_RING = 0.075

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

joint_status_data = {
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
    global joint_status_data
    # Assert order of joints
    for stringIter in range(len(joint_data.names)):
        if joint_data.names[ stringIter ] == ROS_JOINT_HIP_RIGHT:
            joint_status_data[ RIGHT_HIP_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ RIGHT_HIP_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_HIP_LEFT:
            joint_status_data[ LEFT_HIP_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ LEFT_HIP_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_KNEE_RIGHT:
            joint_status_data[ RIGHT_KNEE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ RIGHT_KNEE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_KNEE_LEFT:
            joint_status_data[ LEFT_KNEE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ LEFT_KNEE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_ANKLE_RIGHT:
            joint_status_data[ RIGHT_ANKLE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ RIGHT_ANKLE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_ANKLE_LEFT:
            joint_status_data[ LEFT_ANKLE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ LEFT_ANKLE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]


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
        if ("point_" + str(pointIterator) in loaded_data):
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
            print("WARNING: No point_%s in trajectory" % (pointIterator))
            number_imported_trajectory_points -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(number_imported_trajectory_points)


def getJointPosition(jointName):
    global joint_status_data
    return joint_status_data[ jointName ][ "Pos" ]


def getJointVelocity(jointName):
    global joint_status_data
    return joint_status_data[ jointName ][ "Vel" ]


def getPosition(endeffector, frame):
    fkJointNamesList = [ ROS_JOINT_HIP_RIGHT, ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_KNEE_LEFT,
                         ROS_JOINT_ANKLE_RIGHT, ROS_JOINT_ANKLE_LEFT ]
    fkJointPositions = [ joint_status_data[ RIGHT_HIP_JOINT ][ "Pos" ], joint_status_data[ LEFT_HIP_JOINT ][ "Pos" ],
                         joint_status_data[ RIGHT_KNEE_JOINT ][ "Pos" ], joint_status_data[ LEFT_KNEE_JOINT ][ "Pos" ],
                         joint_status_data[ RIGHT_ANKLE_JOINT ][ "Pos" ],
                         joint_status_data[ LEFT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv(endeffector, frame, fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % (e))
    return [ 0.0, 0.0 ]  # [x, z]


def getPositionLeftFoot():
    fkJointNamesList = [ ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT ]
    fkJointPositions = [ joint_status_data[ LEFT_HIP_JOINT ][ "Pos" ], joint_status_data[ LEFT_KNEE_JOINT ][ "Pos" ],
                         joint_status_data[ LEFT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_left_tip", "foot_left_tip", fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % (e))

    print("ERROR fk foot_left failed")
    return [ 0.0, 0.0 ]  # [x, z]


def getPositionRightFoot():
    fkJointNamesList = [ ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT ]
    fkJointPositions = [ joint_status_data[ RIGHT_HIP_JOINT ][ "Pos" ], joint_status_data[ RIGHT_KNEE_JOINT ][ "Pos" ],
                         joint_status_data[ RIGHT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_right_tip", "foot_right_tip", fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % (e))

    print("ERROR fk foot_right failed")
    return [ 0.0, 0.0 ]  # [x, z]


def getDistance(point1, point2):
    x_diff = point2[ 0 ] - point1[ 0 ]
    y_diff = point2[ 1 ] - point1[ 1 ]

    return math.sqrt((x_diff * x_diff) + (y_diff * y_diff))



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


def compute_velocity(joint_name, next_joint_angle, current_joint_angle, end_time):

    joint_angle_difference = next_joint_angle - current_joint_angle

    current_time = time.time()
    publish_time = end_time - current_time

    ideal_velocity = joint_angle_difference / publish_time

    if PRINT_DEBUG:
        log_string = "\n"
        log_string += ("\n" + joint_name + ":")
        log_string += ("\ncurrent_joint_angle = " + str(current_joint_angle))
        log_string += ("\nnext_joint_angle = " + str(next_joint_angle))
        log_string += ("\nd = " + str(joint_angle_difference))
        log_string += ("\npublish_time = " + str(publish_time))
        log_string += ("\nideal_velocity = " + str(ideal_velocity))
        print (log_string)

    return ideal_velocity



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

    if (current_x > 0 and current_y > 0):
        return np.arctan(current_y / current_x)
    elif (current_x < 0 and current_y > 0):
        return np.arctan(current_y / current_x) + np.pi
    elif (current_x < 0 and current_y < 0):
        return np.arctan(current_y / current_x) + np.pi
    elif (current_x > 0 and current_y < 0):
        return np.arctan(current_y / current_x) + 2 * np.pi

    elif (current_x == 0 and current_y > 0):
        return np.pi / 2
    elif (current_x == 0 and current_y < 0):
        return np.pi * 3 / 2
    elif (current_x > 0 and current_y == 0):
        return 0
    elif (current_x < 0 and current_y == 0):
        return np.pi


def publish_velocity(joint_name, next_joint_angle, current_joint_angle, end_time):

    ideal_velocity = compute_velocity(joint_name, next_joint_angle, current_joint_angle, end_time)
    publisher = None
    error_factor = 1

    if joint_name == RIGHT_HIP_JOINT:
        publisher = ros_right_hip_publisher
        error_factor = velocity_error_factor_hip
        return
    elif joint_name == RIGHT_KNEE_JOINT:
        publisher = ros_right_knee_publisher
        error_factor = velocity_error_factor_knee
        return
    elif joint_name == RIGHT_ANKLE_JOINT:
        publisher = ros_right_ankle_publisher
        error_factor = velocity_error_factor_ankle
        return
    elif joint_name == LEFT_HIP_JOINT:
        publisher = ros_left_hip_publisher
        error_factor = velocity_error_factor_hip
    elif joint_name == LEFT_KNEE_JOINT:
        publisher = ros_left_knee_publisher
        error_factor = velocity_error_factor_knee
    elif joint_name == LEFT_ANKLE_JOINT:
        publisher = ros_left_ankle_publisher
        error_factor = velocity_error_factor_ankle

    published_velocity = ideal_velocity * error_factor / JOINT_VELOCITY_FACTOR_SIMULATION / SIMULATION_FACTOR

    if PRINT_DEBUG:
        log_msg = "publishing velocity "+str(published_velocity*JOINT_VELOCITY_FACTOR_SIMULATION)," rad/s to ", joint_name
        print(log_msg)

    duration = end_time - time.time()

    publisher.publish(published_velocity)
    time.sleep(duration*SIMULATION_FACTOR)

# used by listener to topic "rickshaw_velocity"
def update_velocity(velocity_F32):
    global PEDAL_SINGLE_ROTATION_DURATION
    global TRAJECTORY_POINT_DURATION
    global BIKE_VELOCITY

    velocity = velocity_F32.data

    BIKE_VELOCITY = velocity

    if velocity == 0:
        ros_right_hip_publisher.publish(0)
        ros_right_knee_publisher.publish(0)
        ros_right_ankle_publisher.publish(0)
        ros_left_hip_publisher.publish(0)
        ros_left_knee_publisher.publish(0)
        ros_left_ankle_publisher.publish(0)

    else:
        PEDAL_SINGLE_ROTATION_DURATION = 2 * np.pi * (RADIUS_FRONT_CHAIN_RING / RADIUS_GEAR_CLUSTER /
                                                          (velocity / RADIUS_BACK_TIRE))
        TRAJECTORY_POINT_DURATION = PEDAL_SINGLE_ROTATION_DURATION / NUMBER_CIRCULATION_POINTS


def get_angle_difference(angle_1, angle_2):
    return np.pi - np.abs(np.abs(angle_1 - angle_2) - np.pi)


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
    _currTrajectoryPoint = getPositionLeftFoot()
    _currTrajectoryAngle = evaluate_current_angle(_currTrajectoryPoint)
    _nextTrajectoryAngle = (_currTrajectoryAngle + (2 * np.pi / NUMBER_CIRCULATION_POINTS)) % (np.pi*2)


    _startTime = 0.0
    _endTime = 0.0
    _currTime = 0.0
    _prevTime = 0.0

    trajectory_points = 0


    while _runFSM:


        ##############################################
        if _currState == INIT:
            ##############################################

            importJointTrajectoryRecord()
            interpolate_functions()
            _currState = PEDAL


            while(PEDAL_SINGLE_ROTATION_DURATION == 0):
                pass
                # wait for velocity != 0

        ##############################################
        if _currState == PEDAL:
            ##############################################

            while (PEDAL_SINGLE_ROTATION_DURATION == 0):
                pass
                # wait for velocity != 0

            # Initialize state
            _startTime = time.time()
            _endTime = _startTime + TRAJECTORY_POINT_DURATION

            x_pedal_record.append(_currTrajectoryPoint[ 0 ])
            y_pedal_record.append(_currTrajectoryPoint[ 1 ])

            # Regulate update frequency
            _currTime = time.time()
            while float(float(_currTime) - float(_prevTime)) < (1 / CONTROLLER_FREQUENCY):
                time.sleep(1)
                x_pedal_record.append(_currTrajectoryPoint[ 0 ])
                y_pedal_record.append(_currTrajectoryPoint[ 1 ])
                _currTime = time.time()
            _prevTime = _currTime

            _currTrajectoryPoint = getPositionLeftFoot()
            _currTrajectoryAngle = evaluate_current_angle(_currTrajectoryPoint)

            _startTime = time.time()
            _endTime = _startTime + TRAJECTORY_POINT_DURATION
            _nextTrajectoryAngle = (_currTrajectoryAngle + (2 * np.pi / NUMBER_CIRCULATION_POINTS)) % (2*np.pi)

            trajectory_points += 1

            if PRINT_DEBUG:
                print("\n\ntrajectory_point = ", trajectory_points, ":")
                print("bike_velocity = ", BIKE_VELOCITY)
                print("rotation_duration = ", PEDAL_SINGLE_ROTATION_DURATION)
                print("trajectory_point_duration = ", TRAJECTORY_POINT_DURATION)
                print("current_pedal_angle = ", _currTrajectoryAngle)
                print("next_pedal_angle = ", _nextTrajectoryAngle)
                print("d = ", get_angle_difference(_currTrajectoryAngle, _nextTrajectoryAngle))

            # Iterate through joints and update setpoints
            publisher_threads = []
            i = 0
            for thisJointName in _jointsList:
                current_joint_angle = joint_status_data[thisJointName ][ "Pos" ]
                next_joint_angle = get_joint_angle(thisJointName, _nextTrajectoryAngle)

                _currTime = time.time()

                publisher_threads.append(Thread(target=publish_velocity, args=(thisJointName, next_joint_angle,
                                                                               current_joint_angle, _endTime)))
                publisher_threads[i].start()
                i += 1

            for thread in publisher_threads:
                thread.join()

            for joint in _jointsList:


                actual_joint_angle = get_joint_angle(joint, evaluate_current_angle(getPositionLeftFoot()))
                error = get_angle_difference(actual_joint_angle, next_joint_angle)

                new_factor = get_angle_difference(current_joint_angle, next_joint_angle) \
                             / get_angle_difference(current_joint_angle, actual_joint_angle)


                if np.abs(new_factor-1) <=0.1:

                    if thisJointName == RIGHT_HIP_JOINT:

                        velocity_error_factor_hip = new_factor
                    elif thisJointName == RIGHT_KNEE_JOINT:

                        velocity_error_factor_knee = new_factor
                    elif thisJointName == RIGHT_ANKLE_JOINT:

                        velocity_error_factor_ankle = new_factor
                    elif thisJointName == LEFT_HIP_JOINT:

                        velocity_error_factor_hip = new_factor
                    elif thisJointName == LEFT_KNEE_JOINT:

                        velocity_error_factor_knee = new_factor
                    elif thisJointName == LEFT_ANKLE_JOINT:

                        velocity_error_factor_ankle = new_factor






################
###   MAIN   ###
################


def main():
    rospy.init_node('pedal_simulation', anonymous=True)
    rospy.Subscriber("joint_state", JointState, jointStateCallback)
    rospy.Subscriber("/cmd_velocity_rickshaw", Float32, update_velocity)
    FSM()

    return 1


if __name__ == '__main__':
    main()

