## @package steering

from __future__ import print_function

import json
import math
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import numpy.polynomial.polynomial as poly
from scipy import interpolate
from scipy.misc import derivative

import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from roboy_control_msgs.srv import SetControllerParameters
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32

#roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='joint_hip_left joint_hip_right joint_wheel_right joint_wheel_back joint_pedal spine_joint joint_wheel_left joint_front joint_pedal_right joint_pedal_left elbow_right_rot1 joint_foot_left joint_knee_right joint_knee_left joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left_rot1 elbow_left_rot0 left_wrist_0 left_wrist_1 right_shoulder_axis0 right_shoulder_axis2 right_shoulder_axis1 elbow_right_rot0 right_wrist_0 right_wrist_1 head_axis0 head_axis1 head_axis2'


#############################
###   MODULE PARAMETERS   ###
#############################

MAX_TURNING_ANGLE = math.pi/15
NUM_STEERING_ANGLES = 61  # Should be odd number, symmetric about zero value

JOINT_ANGLE_TOLERANCE_FK = 0.01

ENDEFFECTOR_RIGHT = "right_hand"
FRAME_RIGHT       = "right_hand"
ENDEFFECTOR_LEFT  = "left_hand"
FRAME_LEFT        = "left_hand"


###############################
###   MEASURED PARAMETERS   ###
###############################

BIKE_OFFSET_X = 0  # -0.83471
BIKE_OFFSET_Y = 0  # 0.03437
BIKE_OFFSET_Z = 0  # 0.037

RIKSHAW_TURN_JOINT_X_OFFSET = 0.7163902600571725+0.23003546879794612  # [m]
RIKSHAW_TURN_JOINT_Y_OFFSET = -0.010388552466272516+0.010388308199859624  # [m]
RIKSHAW_TURN_JOINT_Z_OFFSET = 0.2164376942146126-0.20527069599791542  # [m]

YAW_RIGHT_HAND_OFFSET = math.pi / 2 + math.pi
YAW_LEFT_HAND_OFFSET  = 3 * math.pi / 2 + math.pi

HANDLEBAR_X_OFFSET = 0.728713  # [m]
HANDLEBAR_Z_OFFSET = 0.719269  # [m]

HAND_Y_OFFSET = 0.2  # [m]


############################
###   GLOBAL VARIABLES   ###
############################


_steeringAngles = []
_rightHandTrajectoryPlanned = []
_leftHandTrajectoryPlanned = []
_centerHandlebarTrajectoryPlanned = []

_rightHandTrajectoryActual = []
_leftHandTrajectoryActual = []

JOINT_SHOULDER_AXIS0_RIGHT = "right_shoulder_axis0"
JOINT_SHOULDER_AXIS1_RIGHT = "right_shoulder_axis1"
JOINT_SHOULDER_AXIS2_RIGHT = "right_shoulder_axis2"
JOINT_SHOULDER_AXIS0_LEFT  = "left_shoulder_axis0"
JOINT_SHOULDER_AXIS1_LEFT  = "left_shoulder_axis1"
JOINT_SHOULDER_AXIS2_LEFT  = "left_shoulder_axis2"
JOINT_ELBOW_ROT0_RIGHT     = "elbow_right_rot0"
JOINT_ELBOW_ROT1_RIGHT     = "elbow_right_rot1"
JOINT_ELBOW_ROT0_LEFT      = "elbow_left_rot0"
JOINT_ELBOW_ROT1_LEFT      = "elbow_left_rot1"
JOINT_WRIST_0_RIGHT        = "right_wrist_0"
JOINT_WRIST_1_RIGHT        = "right_wrist_1"
JOINT_WRIST_0_LEFT         = "left_wrist_0"
JOINT_WRIST_1_LEFT         = "left_wrist_1"

JOINT_LIST = [JOINT_SHOULDER_AXIS0_RIGHT, JOINT_SHOULDER_AXIS1_RIGHT, JOINT_SHOULDER_AXIS2_RIGHT, JOINT_SHOULDER_AXIS0_LEFT, JOINT_SHOULDER_AXIS1_LEFT, JOINT_SHOULDER_AXIS2_LEFT, JOINT_ELBOW_ROT0_RIGHT, JOINT_ELBOW_ROT1_RIGHT, JOINT_ELBOW_ROT0_LEFT, JOINT_ELBOW_ROT1_LEFT, JOINT_WRIST_0_RIGHT, JOINT_WRIST_1_RIGHT, JOINT_WRIST_0_LEFT, JOINT_WRIST_1_LEFT]

_regressedShoulder0Right  = None
_regressedShoulder1Right  = None
_regressedShoulder2Right  = None
_regressedShoulder0Left   = None
_regressedShoulder1Left   = None
_regressedShoulder2Left   = None
_regressedElbow0Right     = None
_regressedElbow1Right     = None
_regressedElbow0Left      = None
_regressedElbow1Left      = None
_regressedWrist0Right     = None
_regressedWrist1Right     = None
_regressedWrist0Left      = None
_regressedWrist1Left      = None

_jointsStatusData = {
    JOINT_SHOULDER_AXIS0_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS1_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS2_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS0_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS1_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS2_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_ELBOW_ROT0_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_ELBOW_ROT1_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_ELBOW_ROT0_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_ELBOW_ROT1_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_WRIST_0_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_WRIST_1_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_WRIST_0_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_WRIST_1_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    }
}

##############################
###   UTILITY FUNCTIONS   ###
##############################


def regressFunctionsFromFile(filename_coefficients):

    global _regressedShoulder0Right
    global _regressedShoulder1Right
    global _regressedShoulder2Right
    global _regressedShoulder0Left
    global _regressedShoulder1Left
    global _regressedShoulder2Left
    global _regressedElbow0Right
    global _regressedElbow1Right
    global _regressedElbow0Left
    global _regressedElbow1Left
    global _regressedWrist0Right
    global _regressedWrist1Right
    global _regressedWrist0Left
    global _regressedWrist1Left

    with open(filename_coefficients, "r") as read_file:
        loaded_data = json.load(read_file)

    _regressedShoulder0Right = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS0_RIGHT])
    _regressedShoulder1Right = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS1_RIGHT])
    _regressedShoulder2Right = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS2_RIGHT])
    _regressedElbow0Right    = poly.Polynomial(loaded_data[JOINT_ELBOW_ROT0_RIGHT])
    _regressedElbow1Right    = poly.Polynomial(loaded_data[JOINT_ELBOW_ROT1_RIGHT])
    _regressedWrist0Right    = poly.Polynomial(loaded_data[JOINT_WRIST_0_RIGHT])
    _regressedWrist1Right    = poly.Polynomial(loaded_data[JOINT_WRIST_1_RIGHT])

    _regressedShoulder0Left = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS0_LEFT])
    _regressedShoulder1Left = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS1_LEFT])
    _regressedShoulder2Left = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS2_LEFT])
    _regressedElbow0Left    = poly.Polynomial(loaded_data[JOINT_ELBOW_ROT0_LEFT])
    _regressedElbow1Left    = poly.Polynomial(loaded_data[JOINT_ELBOW_ROT1_LEFT])
    _regressedWrist0Left    = poly.Polynomial(loaded_data[JOINT_WRIST_0_LEFT])
    _regressedWrist1Left    = poly.Polynomial(loaded_data[JOINT_WRIST_1_LEFT])

    return 1


def setJointControllerParameters(proportionalVal, derivativeVal):

    for thisJointName in JOINT_LIST:
        rospy.wait_for_service(thisJointName + '/' + thisJointName + '/params')
        try:
            joint_srv = rospy.ServiceProxy(thisJointName + '/' + thisJointName + '/params', SetControllerParameters)
            joint_srv(proportionalVal, derivativeVal)
        except rospy.ServiceException, e:
            print("Service call change control parameter failed: ", e)


def computeSteeringAngles():

    global _steeringAngles
    global NUM_STEERING_ANGLES

    if NUM_STEERING_ANGLES % 2 == 0:
        print("ERROR: NUM_STEERING_ANGLES must be odd number")
        return

    numSymmetricSteeringAngles = (NUM_STEERING_ANGLES - 1) / 2

    for sizeIterator in range(NUM_STEERING_ANGLES):
        _steeringAngles.append(0.0)

    _steeringAngles[int(NUM_STEERING_ANGLES / 2)] = 0.0  # Redundant
    for steeringAngleIterator in range(1, numSymmetricSteeringAngles + 1, 1):
        thisAngle = steeringAngleIterator * (MAX_TURNING_ANGLE / numSymmetricSteeringAngles)
        _steeringAngles[numSymmetricSteeringAngles + steeringAngleIterator] = thisAngle
        _steeringAngles[numSymmetricSteeringAngles - steeringAngleIterator] = (-1) * thisAngle


def computeHandTrajectories():

    global _steeringAngles
    global _rightHandTrajectoryPlanned
    global _leftHandTrajectoryPlanned
    global _centerHandlebarTrajectoryPlanned

    for steeringAngleIterator in range(len(_steeringAngles)):

        thisCenterPointX = RIKSHAW_TURN_JOINT_X_OFFSET - (HANDLEBAR_X_OFFSET * math.cos(_steeringAngles[steeringAngleIterator]))
        thisCenterPointY = RIKSHAW_TURN_JOINT_Y_OFFSET - (HANDLEBAR_X_OFFSET * math.sin(_steeringAngles[steeringAngleIterator]))

        thisRightHandPointX = thisCenterPointX + (HAND_Y_OFFSET * math.cos(_steeringAngles[steeringAngleIterator] - (math.pi / 2)))
        thisRightHandPointY = thisCenterPointY + (HAND_Y_OFFSET * math.sin(_steeringAngles[steeringAngleIterator] - (math.pi / 2)))

        thisLeftHandPointX = thisCenterPointX + (HAND_Y_OFFSET * math.cos(_steeringAngles[steeringAngleIterator] + (math.pi / 2)))
        thisLeftHandPointY = thisCenterPointY + (HAND_Y_OFFSET * math.sin(_steeringAngles[steeringAngleIterator] + (math.pi / 2)))

        _centerHandlebarTrajectoryPlanned.append([thisCenterPointX, thisCenterPointY])
        _rightHandTrajectoryPlanned.append([thisRightHandPointX, thisRightHandPointY])
        _leftHandTrajectoryPlanned.append([thisLeftHandPointX, thisLeftHandPointY])


def getPositionLeftHand():
    fkJointNamesList = [JOINT_SHOULDER_AXIS0_LEFT, JOINT_SHOULDER_AXIS1_LEFT, JOINT_SHOULDER_AXIS2_LEFT, JOINT_ELBOW_ROT0_LEFT, JOINT_ELBOW_ROT1_LEFT, JOINT_WRIST_0_LEFT, JOINT_WRIST_1_LEFT]
    fkJointPositions = [_jointsStatusData[JOINT_SHOULDER_AXIS0_LEFT]["Pos"], _jointsStatusData[JOINT_SHOULDER_AXIS1_LEFT]["Pos"], _jointsStatusData[JOINT_SHOULDER_AXIS2_LEFT]["Pos"], _jointsStatusData[JOINT_ELBOW_ROT0_LEFT]["Pos"], _jointsStatusData[JOINT_ELBOW_ROT1_LEFT]["Pos"], _jointsStatusData[JOINT_WRIST_0_LEFT]["Pos"], _jointsStatusData[JOINT_WRIST_1_LEFT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv(ENDEFFECTOR_LEFT, FRAME_LEFT, fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.y, fk_result.pose.position.z]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    print("ERROR fk left_hand failed")
    return [0.0, 0.0, 0.0]  # [x, z]


def getPositionRightHand():
    fkJointNamesList = [JOINT_SHOULDER_AXIS0_RIGHT, JOINT_SHOULDER_AXIS1_RIGHT, JOINT_SHOULDER_AXIS2_RIGHT, JOINT_ELBOW_ROT0_RIGHT, JOINT_ELBOW_ROT1_RIGHT, JOINT_WRIST_0_RIGHT, JOINT_WRIST_1_RIGHT]
    fkJointPositions = [_jointsStatusData[JOINT_SHOULDER_AXIS0_RIGHT]["Pos"], _jointsStatusData[JOINT_SHOULDER_AXIS1_RIGHT]["Pos"], _jointsStatusData[JOINT_SHOULDER_AXIS2_RIGHT]["Pos"], _jointsStatusData[JOINT_ELBOW_ROT0_RIGHT]["Pos"], _jointsStatusData[JOINT_ELBOW_ROT1_RIGHT]["Pos"], _jointsStatusData[JOINT_WRIST_0_RIGHT]["Pos"], _jointsStatusData[JOINT_WRIST_1_RIGHT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv(ENDEFFECTOR_RIGHT, FRAME_RIGHT, fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.y, fk_result.pose.position.z]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    print("ERROR fk right_hand failed")
    return [0.0, 0.0, 0.0]  # [x, z]


def jointStateCallback(joint_data):
    global _jointsStatusData
    # Assert order of joints
    for stringIter in range(len(joint_data.names)):
        if joint_data.names[stringIter] in _jointsStatusData:
            _jointsStatusData[joint_data.names[stringIter]]["Pos"] = joint_data.q[stringIter]
            _jointsStatusData[joint_data.names[stringIter]]["Vel"] = joint_data.qd[stringIter]

def recordActualHandTrajectories(ros_right_shoulder0_publisher, ros_right_shoulder1_publisher, ros_right_shoulder2_publisher, ros_right_elbow0_publisher, ros_right_elbow1_publisher, ros_right_wrist0_publisher, ros_right_wrist1_publisher, ros_left_shoulder0_publisher, ros_left_shoulder1_publisher, ros_left_shoulder2_publisher, ros_left_elbow0_publisher, ros_left_elbow1_publisher, ros_left_wrist0_publisher, ros_left_wrist1_publisher):

    global _steeringAngles
    global _rightHandTrajectoryActual
    global _leftHandTrajectoryActual

    global _regressedShoulder0Right
    global _regressedShoulder1Right
    global _regressedShoulder2Right
    global _regressedShoulder0Left
    global _regressedShoulder1Left
    global _regressedShoulder2Left
    global _regressedElbow0Right
    global _regressedElbow1Right
    global _regressedElbow0Left
    global _regressedElbow1Left
    global _regressedWrist0Right
    global _regressedWrist1Right
    global _regressedWrist0Left
    global _regressedWrist1Left

    for thisSteeringAngle in _steeringAngles:

        print("Recording steering angle: ", thisSteeringAngle)

        ros_left_shoulder0_publisher.publish(_regressedShoulder0Left(thisSteeringAngle).astype(float))
        ros_left_shoulder1_publisher.publish(_regressedShoulder1Left(thisSteeringAngle).astype(float))
        ros_left_shoulder2_publisher.publish(_regressedShoulder2Left(thisSteeringAngle).astype(float))
        ros_right_shoulder0_publisher.publish(_regressedShoulder0Right(thisSteeringAngle).astype(float))
        ros_right_shoulder1_publisher.publish(_regressedShoulder1Right(thisSteeringAngle).astype(float))
        ros_right_shoulder2_publisher.publish(_regressedShoulder2Right(thisSteeringAngle).astype(float))
        ros_left_elbow0_publisher.publish(_regressedElbow0Left(thisSteeringAngle).astype(float))
        ros_left_elbow1_publisher.publish(_regressedElbow1Left(thisSteeringAngle).astype(float))
        ros_right_elbow0_publisher.publish(_regressedElbow0Right(thisSteeringAngle).astype(float))
        ros_right_elbow1_publisher.publish(_regressedElbow1Right(thisSteeringAngle).astype(float))
        ros_left_wrist0_publisher.publish(_regressedWrist0Left(thisSteeringAngle).astype(float))
        ros_left_wrist1_publisher.publish(_regressedWrist1Left(thisSteeringAngle).astype(float))
        ros_right_wrist0_publisher.publish(_regressedWrist0Right(thisSteeringAngle).astype(float))
        ros_right_wrist1_publisher.publish(_regressedWrist1Right(thisSteeringAngle).astype(float))

        while abs(_jointsStatusData[JOINT_SHOULDER_AXIS0_LEFT]["Pos"] - _regressedShoulder0Left(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_SHOULDER_AXIS0_LEFT moved to new position")
        while abs(_jointsStatusData[JOINT_SHOULDER_AXIS1_LEFT]["Pos"] - _regressedShoulder1Left(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_SHOULDER_AXIS1_LEFT moved to new position")
        while abs(_jointsStatusData[JOINT_SHOULDER_AXIS2_LEFT]["Pos"] - _regressedShoulder2Left(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_SHOULDER_AXIS2_LEFT moved to new position")

        while abs(_jointsStatusData[JOINT_SHOULDER_AXIS0_RIGHT]["Pos"] - _regressedShoulder0Right(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_SHOULDER_AXIS0_RIGHT moved to new position")
        while abs(_jointsStatusData[JOINT_SHOULDER_AXIS1_RIGHT]["Pos"] - _regressedShoulder1Right(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_SHOULDER_AXIS1_RIGHT moved to new position")
        while abs(_jointsStatusData[JOINT_SHOULDER_AXIS2_RIGHT]["Pos"] - _regressedShoulder2Right(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_SHOULDER_AXIS2_RIGHT moved to new position")

        while abs(_jointsStatusData[JOINT_ELBOW_ROT0_LEFT]["Pos"] - _regressedElbow0Left(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_ELBOW_ROT0_LEFT moved to new position")
        while abs(_jointsStatusData[JOINT_ELBOW_ROT1_LEFT]["Pos"] - _regressedElbow1Left(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_ELBOW_ROT1_LEFT moved to new position")

        while abs(_jointsStatusData[JOINT_ELBOW_ROT0_RIGHT]["Pos"] - _regressedElbow0Right(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_ELBOW_ROT0_RIGHT moved to new position")
        while abs(_jointsStatusData[JOINT_ELBOW_ROT1_RIGHT]["Pos"] - _regressedElbow1Right(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_ELBOW_ROT1_RIGHT moved to new position")

        while abs(_jointsStatusData[JOINT_WRIST_0_LEFT]["Pos"] - _regressedWrist0Left(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_WRIST_0_LEFT moved to new position")
        while abs(_jointsStatusData[JOINT_WRIST_1_LEFT]["Pos"] - _regressedWrist1Left(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_WRIST_1_LEFT moved to new position")

        while abs(_jointsStatusData[JOINT_WRIST_0_RIGHT]["Pos"] - _regressedWrist0Right(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_WRIST_0_RIGHT moved to new position")
        while abs(_jointsStatusData[JOINT_WRIST_1_RIGHT]["Pos"] - _regressedWrist1Right(thisSteeringAngle).astype(float)) > JOINT_ANGLE_TOLERANCE_FK:
            time.sleep(0.1)
        print("JOINT_WRIST_1_RIGHT moved to new position")


        _rightHandTrajectoryActual.append(getPositionRightHand())
        _leftHandTrajectoryActual.append(getPositionLeftHand())

        print("Moving on...")

def printPlannedActualHandTrajectories():

    global _rightHandTrajectoryPlanned
    global _leftHandTrajectoryPlanned

    global _rightHandTrajectoryActual
    global _leftHandTrajectoryActual

    rightHandTrajectoryPlannedX = []
    rightHandTrajectoryPlannedY = []
    leftHandTrajectoryPlannedX = []
    leftHandTrajectoryPlannedY = []

    rightHandTrajectoryActualX = []
    rightHandTrajectoryActualY = []
    leftHandTrajectoryActualX = []
    leftHandTrajectoryActualY = []

    for pointIterator in range(len(_rightHandTrajectoryPlanned)):
        rightHandTrajectoryPlannedX.append(_rightHandTrajectoryPlanned[pointIterator][0])
        rightHandTrajectoryPlannedY.append(_rightHandTrajectoryPlanned[pointIterator][1])

    for pointIterator in range(len(_leftHandTrajectoryPlanned)):
        leftHandTrajectoryPlannedX.append(_leftHandTrajectoryPlanned[pointIterator][0])
        leftHandTrajectoryPlannedY.append(_leftHandTrajectoryPlanned[pointIterator][1])

    for pointIterator in range(len(_rightHandTrajectoryActual)):
        rightHandTrajectoryActualX.append(_rightHandTrajectoryActual[pointIterator][0])
        rightHandTrajectoryActualY.append(_rightHandTrajectoryActual[pointIterator][1])

    for pointIterator in range(len(_leftHandTrajectoryActual)):
        leftHandTrajectoryActualX.append(_leftHandTrajectoryActual[pointIterator][0])
        leftHandTrajectoryActualY.append(_leftHandTrajectoryActual[pointIterator][1])

    print("Right hand trajectory planned x: ", rightHandTrajectoryPlannedX)
    print("Right hand trajectory planned y: ", rightHandTrajectoryPlannedY)

    plt.figure(1)
    plt.plot(rightHandTrajectoryPlannedX, rightHandTrajectoryPlannedY, 'b-', label = 'Planned')
    plt.plot(rightHandTrajectoryActualX, rightHandTrajectoryActualY, 'r*', label = 'Actual')
    plt.title("Right hand planned vs. actual path")

    plt.figure(2)
    plt.plot(leftHandTrajectoryPlannedX, leftHandTrajectoryPlannedY, 'b-', label = 'Planned')
    plt.plot(leftHandTrajectoryActualX, leftHandTrajectoryActualY, 'r*', label = 'Actual')
    plt.title("Left hand planned vs. actual path")

    plt.show()


################
###   MAIN   ###
################


def main():

    filename_coefficients = "saved_coefficients.json"

    rospy.init_node('steering_trajectory_test', anonymous=True)
    rospy.Subscriber("joint_state", JointState, jointStateCallback)

    ros_right_shoulder0_publisher = rospy.Publisher('/' + JOINT_SHOULDER_AXIS0_RIGHT + '/' + JOINT_SHOULDER_AXIS0_RIGHT + '/target', Float32, queue_size=1)
    ros_right_shoulder1_publisher = rospy.Publisher('/' + JOINT_SHOULDER_AXIS1_RIGHT + '/' + JOINT_SHOULDER_AXIS1_RIGHT + '/target', Float32, queue_size=1)
    ros_right_shoulder2_publisher = rospy.Publisher('/' + JOINT_SHOULDER_AXIS2_RIGHT + '/' + JOINT_SHOULDER_AXIS2_RIGHT + '/target', Float32, queue_size=1)
    ros_right_elbow0_publisher    = rospy.Publisher('/' + JOINT_ELBOW_ROT0_RIGHT + '/' + JOINT_ELBOW_ROT0_RIGHT + '/target', Float32, queue_size=1)
    ros_right_elbow1_publisher    = rospy.Publisher('/' + JOINT_ELBOW_ROT1_RIGHT + '/' + JOINT_ELBOW_ROT1_RIGHT + '/target', Float32, queue_size=1)
    ros_right_wrist0_publisher    = rospy.Publisher('/' + JOINT_WRIST_0_RIGHT + '/' + JOINT_WRIST_0_RIGHT + '/target', Float32, queue_size=1)
    ros_right_wrist1_publisher    = rospy.Publisher('/' + JOINT_WRIST_1_RIGHT + '/' + JOINT_WRIST_1_RIGHT + '/target', Float32, queue_size=1)

    ros_left_shoulder0_publisher  = rospy.Publisher('/' + JOINT_SHOULDER_AXIS0_LEFT + '/' + JOINT_SHOULDER_AXIS0_LEFT + '/target', Float32, queue_size=1)
    ros_left_shoulder1_publisher  = rospy.Publisher('/' + JOINT_SHOULDER_AXIS1_LEFT + '/' + JOINT_SHOULDER_AXIS1_LEFT + '/target', Float32, queue_size=1)
    ros_left_shoulder2_publisher  = rospy.Publisher('/' + JOINT_SHOULDER_AXIS2_LEFT + '/' + JOINT_SHOULDER_AXIS2_LEFT + '/target', Float32, queue_size=1)
    ros_left_elbow0_publisher     = rospy.Publisher('/' + JOINT_ELBOW_ROT0_LEFT + '/' + JOINT_ELBOW_ROT0_LEFT + '/target', Float32, queue_size=1)
    ros_left_elbow1_publisher     = rospy.Publisher('/' + JOINT_ELBOW_ROT1_LEFT + '/' + JOINT_ELBOW_ROT1_LEFT + '/target', Float32, queue_size=1)
    ros_left_wrist0_publisher     = rospy.Publisher('/' + JOINT_WRIST_0_LEFT + '/' + JOINT_WRIST_0_LEFT + '/target', Float32, queue_size=1)
    ros_left_wrist1_publisher     = rospy.Publisher('/' + JOINT_WRIST_1_LEFT + '/' + JOINT_WRIST_1_LEFT + '/target', Float32, queue_size=1)

    time.sleep(3)
    print("Initialized")
    regressFunctionsFromFile(filename_coefficients)
    print("Regressed based on coefficients from file")
    setJointControllerParameters(proportionalVal = 2500, derivativeVal = 0)
    computeSteeringAngles()
    computeHandTrajectories()
    print("Planned hand trajectories calculated")
    recordActualHandTrajectories(ros_right_shoulder0_publisher, ros_right_shoulder1_publisher, ros_right_shoulder2_publisher, ros_right_elbow0_publisher, ros_right_elbow1_publisher, ros_right_wrist0_publisher, ros_right_wrist1_publisher, ros_left_shoulder0_publisher, ros_left_shoulder1_publisher, ros_left_shoulder2_publisher, ros_left_elbow0_publisher, ros_left_elbow1_publisher, ros_left_wrist0_publisher, ros_left_wrist1_publisher)
    print("Actual hand trajectories recorded")
    printPlannedActualHandTrajectories()

    return 1


if __name__ == '__main__':
    main()
