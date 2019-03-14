## @package steering

import json
import math
import time

import matplotlib.pyplot as plt
import rospy
import tf
from geometry_msgs.msg import Pose, Point
from roboy_control_msgs.srv import SetControllerParameters
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from std_msgs.msg import Float32

###############################
###   FUNCTION PARAMETERS   ###
###############################

MAX_TURNING_ANGLE = math.pi / 15  # [rad]
NUM_STEERING_ANGLES = 61  # Should be odd number, symmetric about zero value

RIKSHAW_TURN_JOINT_X_OFFSET = -0.23003546880974085 + (-0.716390260045379) # [m]
RIKSHAW_TURN_JOINT_Y_OFFSET = -0.010388308215364166 + (0.010388552481776845) # [m]
RIKSHAW_TURN_JOINT_Z_OFFSET = -0.2052706960113988 + (0.21643769422809594) # [m]

YAW_RIGHT_HAND_OFFSET = math.pi / 2 + math.pi
YAW_LEFT_HAND_OFFSET = 3 * math.pi / 2 + math.pi

HANDLEBAR_X_OFFSET = 0.728713  # [m]
HANDLEBAR_Z_OFFSET = 0.719269  # [m]

HAND_Y_OFFSET = 0.2  # [m]

JSON_FILENAME = "new_hand_steering_trajectory.json"
JOINT_ANGLE_TOLERANCE_FK = 0.01

ENDEFFECTOR_RIGHT = "right_arm"
FRAME_RIGHT = "wrist_right_sphere_link1"
ENDEFFECTOR_LEFT = "left_arm"
FRAME_LEFT = "wrist_left_sphere_link1"

###############################
###   MEASURED PARAMETERS   ###
###############################

BIKE_OFFSET_X = 0  # -0.83471
BIKE_OFFSET_Y = 0  # 0.03437
BIKE_OFFSET_Z = 0  # 0.037

############################
###   GLOBAL VARIABLES   ###
############################


_steeringAngles = []
_rightHandTrajectory = []
_leftHandTrajectory = []
_centerHandlebarTrajectory = []

JOINT_SHOULDER_AXIS0_RIGHT = "right_shoulder_axis0"
JOINT_SHOULDER_AXIS1_RIGHT = "right_shoulder_axis1"
JOINT_SHOULDER_AXIS2_RIGHT = "right_shoulder_axis2"
JOINT_SHOULDER_AXIS0_LEFT = "left_shoulder_axis0"
JOINT_SHOULDER_AXIS1_LEFT = "left_shoulder_axis1"
JOINT_SHOULDER_AXIS2_LEFT = "left_shoulder_axis2"
JOINT_ELBOW_RIGHT = "elbow_right"
JOINT_ELBOW_LEFT = "elbow_left"
JOINT_WRIST_RIGHT_SPHERE_AXIS0 = "wrist_right_sphere_axis0"
#JOINT_WRIST_RIGHT_SPHERE_AXIS1 = "wrist_right_sphere_axis1"
#JOINT_WRIST_RIGHT_SPHERE_AXIS2 = "wrist_right_sphere_axis2"
JOINT_WRIST_LEFT_SPHERE_AXIS0 = "wrist_left_sphere_axis0"
#JOINT_WRIST_LEFT_SPHERE_AXIS1 = "wrist_left_sphere_axis1"
#JOINT_WRIST_LEFT_SPHERE_AXIS2 = "wrist_left_sphere_axis2"

JOINT_LIST = [JOINT_SHOULDER_AXIS0_RIGHT, JOINT_SHOULDER_AXIS1_RIGHT, JOINT_SHOULDER_AXIS2_RIGHT,
              JOINT_SHOULDER_AXIS0_LEFT, JOINT_SHOULDER_AXIS1_LEFT, JOINT_SHOULDER_AXIS2_LEFT, JOINT_ELBOW_RIGHT, JOINT_ELBOW_LEFT, JOINT_WRIST_RIGHT_SPHERE_AXIS0,
              JOINT_WRIST_LEFT_SPHERE_AXIS0] # JOINT_WRIST_RIGHT_SPHERE_AXIS1, JOINT_WRIST_RIGHT_SPHERE_AXIS2, JOINT_WRIST_LEFT_SPHERE_AXIS1, JOINT_WRIST_LEFT_SPHERE_AXIS2

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
    JOINT_ELBOW_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_ELBOW_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
        JOINT_WRIST_RIGHT_SPHERE_AXIS0: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    # JOINT_WRIST_RIGHT_SPHERE_AXIS1: {
    #     "Pos": 0.0,
    #     "Vel": 0.0
    # },
    # JOINT_WRIST_RIGHT_SPHERE_AXIS2: {
    #     "Pos": 0.0,
    #     "Vel": 0.0
    # },
    JOINT_WRIST_LEFT_SPHERE_AXIS0: {
        "Pos": 0.0,
        "Vel": 0.0
    } #,
    # JOINT_WRIST_LEFT_SPHERE_AXIS1: {
    #     "Pos": 0.0,
    #     "Vel": 0.0
    # },
    # JOINT_WRIST_LEFT_SPHERE_AXIS2: {
    #     "Pos": 0.0,
    #     "Vel": 0.0
    # }
}


##############################
###   UTILITY FUNCTIONS   ###
##############################


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
    global _rightHandTrajectory
    global _leftHandTrajectory
    global _centerHandlebarTrajectory

    for steeringAngleIterator in range(len(_steeringAngles)):
        thisCenterPointX = RIKSHAW_TURN_JOINT_X_OFFSET + (
                    HANDLEBAR_X_OFFSET * math.cos(_steeringAngles[steeringAngleIterator]))
        thisCenterPointY = RIKSHAW_TURN_JOINT_Y_OFFSET - (
                    HANDLEBAR_X_OFFSET * math.sin(_steeringAngles[steeringAngleIterator]))

        thisRightHandPointX = thisCenterPointX + (
                    HAND_Y_OFFSET * math.cos(_steeringAngles[steeringAngleIterator] - (math.pi / 2)))
        thisRightHandPointY = thisCenterPointY + (
                    HAND_Y_OFFSET * math.sin(_steeringAngles[steeringAngleIterator] - (math.pi / 2)))

        thisLeftHandPointX = thisCenterPointX + (
                    HAND_Y_OFFSET * math.cos(_steeringAngles[steeringAngleIterator] + (math.pi / 2)))
        thisLeftHandPointY = thisCenterPointY + (
                    HAND_Y_OFFSET * math.sin(_steeringAngles[steeringAngleIterator] + (math.pi / 2)))

        _centerHandlebarTrajectory.append([thisCenterPointX, thisCenterPointY])
        _rightHandTrajectory.append([thisRightHandPointX, thisRightHandPointY])
        _leftHandTrajectory.append([thisLeftHandPointX, thisLeftHandPointY])


def getPositionLeftHand():
    fkJointNamesList = [JOINT_SHOULDER_AXIS0_LEFT, JOINT_SHOULDER_AXIS1_LEFT, JOINT_SHOULDER_AXIS2_LEFT, JOINT_ELBOW_LEFT, JOINT_WRIST_LEFT_SPHERE_AXIS0] #JOINT_WRIST_LEFT_SPHERE_AXIS1, JOINT_WRIST_LEFT_SPHERE_AXIS2
    fkJointPositions = [_jointsStatusData[JOINT_SHOULDER_AXIS0_LEFT]["Pos"],
                        _jointsStatusData[JOINT_SHOULDER_AXIS1_LEFT]["Pos"],
                        _jointsStatusData[JOINT_SHOULDER_AXIS2_LEFT]["Pos"],
                        _jointsStatusData[JOINT_ELBOW_LEFT]["Pos"],
                        _jointsStatusData[JOINT_WRIST_LEFT_SPHERE_AXIS0]["Pos"] #,
                        #_jointsStatusData[JOINT_WRIST_LEFT_SPHERE_AXIS1]["Pos"],
                        #_jointsStatusData[JOINT_WRIST_LEFT_SPHERE_AXIS2]["Pos"]
                        ]

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
    fkJointNamesList = [JOINT_SHOULDER_AXIS0_RIGHT, JOINT_SHOULDER_AXIS1_RIGHT, JOINT_SHOULDER_AXIS2_RIGHT,
                  JOINT_ELBOW_RIGHT, JOINT_WRIST_RIGHT_SPHERE_AXIS0] # JOINT_WRIST_RIGHT_SPHERE_AXIS1, JOINT_WRIST_RIGHT_SPHERE_AXIS2
    fkJointPositions = [_jointsStatusData[JOINT_SHOULDER_AXIS0_RIGHT]["Pos"],
                        _jointsStatusData[JOINT_SHOULDER_AXIS1_RIGHT]["Pos"],
                        _jointsStatusData[JOINT_SHOULDER_AXIS2_RIGHT]["Pos"],
                        _jointsStatusData[JOINT_ELBOW_RIGHT]["Pos"],
                        _jointsStatusData[JOINT_WRIST_RIGHT_SPHERE_AXIS0]["Pos"],
                        #_jointsStatusData[JOINT_WRIST_RIGHT_SPHERE_AXIS1]["Pos"],
                        #_jointsStatusData[JOINT_WRIST_RIGHT_SPHERE_AXIS2]["Pos"]
                        ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv(ENDEFFECTOR_RIGHT, FRAME_RIGHT, fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.y, fk_result.pose.position.z]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    print("ERROR fk right_hand failed")
    return [0.0, 0.0, 0.0]  # [x, z]


def setJointControllerParameters(proportionalVal, derivativeVal):
    for thisJointName in JOINT_LIST:
        rospy.wait_for_service(thisJointName + '/' + thisJointName + '/params')
        try:
            param_srv = rospy.ServiceProxy(thisJointName + '/' + thisJointName + '/params', SetControllerParameters)
            param_srv(proportionalVal, derivativeVal)
        except rospy.ServiceException, e:
            print
            "Service call for " + thisJointName + "failed: %s" % e

    print("Controller paramters updated")


def jointStateCallback(joint_data):
    global _jointsStatusData
    # Assert order of joints
    for stringIter in range(len(joint_data.names)):
        if joint_data.names[stringIter] in _jointsStatusData:
            _jointsStatusData[joint_data.names[stringIter]]["Pos"] = joint_data.q[stringIter]
            _jointsStatusData[joint_data.names[stringIter]]["Vel"] = joint_data.qd[stringIter]


def inverse_kinematics_client(endeffector, frame, x, y, z, roll, pitch, yaw):
    rospy.wait_for_service('ik')
    try:
        ik_srv = rospy.ServiceProxy('ik', InverseKinematics)
        requested_position = Point(x, y, z)
        requested_pose = Pose(position=requested_position)
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        requested_pose.orientation.x = quaternion[0]
        requested_pose.orientation.y = quaternion[1]
        requested_pose.orientation.z = quaternion[2]
        requested_pose.orientation.w = quaternion[3]
        requested_ik_type = 1 # Pos only #0 Position and orientation
        ik_result = ik_srv(endeffector, requested_ik_type, frame, requested_pose)

        jointDict = {}
        for thisJoint in range(len(ik_result.angles)):
            jointDict[ik_result.joint_names[thisJoint]] = ik_result.angles[thisJoint]

        return jointDict

    except rospy.ServiceException, e:
        print
        "Service call failed: %s" % e


################
###   MAIN   ###
################

def main():
    global _steeringAngles
    global _rightHandTrajectory
    global _leftHandTrajectory
    global _centerHandlebarTrajectory

    computeSteeringAngles()
    computeHandTrajectories()

    plt.figure(1)
    for steeringAngleIterator in range(len(_steeringAngles)):
        plt.plot([RIKSHAW_TURN_JOINT_X_OFFSET, _centerHandlebarTrajectory[steeringAngleIterator][0]],
                 [RIKSHAW_TURN_JOINT_Y_OFFSET, _centerHandlebarTrajectory[steeringAngleIterator][1]])
        plt.plot([_rightHandTrajectory[steeringAngleIterator][0], _leftHandTrajectory[steeringAngleIterator][0]],
                 [_rightHandTrajectory[steeringAngleIterator][1], _leftHandTrajectory[steeringAngleIterator][1]])
        plt.plot([_rightHandTrajectory[steeringAngleIterator][0], _leftHandTrajectory[steeringAngleIterator][0]],
                 [_rightHandTrajectory[steeringAngleIterator][1], _leftHandTrajectory[steeringAngleIterator][1]], '*')
    plt.show()

    global JOINT_ANGLE_TOLERANCE_FK

    rospy.init_node('steering_capture', anonymous=True)
    rospy.Subscriber("joint_state", JointState, jointStateCallback)

    ros_right_shoulder_axis0_pub = rospy.Publisher('/right_shoulder_axis0/right_shoulder_axis0/target', Float32,
                                                   queue_size=2)
    ros_right_shoulder_axis1_pub = rospy.Publisher('/right_shoulder_axis1/right_shoulder_axis1/target', Float32,
                                                   queue_size=2)
    ros_right_shoulder_axis2_pub = rospy.Publisher('/right_shoulder_axis2/right_shoulder_axis2/target', Float32,
                                                   queue_size=2)
    ros_left_shoulder_axis0_pub = rospy.Publisher('/left_shoulder_axis0/left_shoulder_axis0/target', Float32, queue_size=2)
    ros_left_shoulder_axis1_pub = rospy.Publisher('/left_shoulder_axis1/left_shoulder_axis1/target', Float32, queue_size=2)
    ros_left_shoulder_axis2_pub = rospy.Publisher('/left_shoulder_axis2/left_shoulder_axis2/target', Float32, queue_size=2)
    ros_elbow_right_pub = rospy.Publisher('/elbow_right/elbow_right/target', Float32, queue_size=2)
    ros_elbow_left_pub = rospy.Publisher('/elbow_left/elbow_left/target', Float32, queue_size=2)
    ros_right_wrist_0_pub = rospy.Publisher('/wrist_right_sphere_axis0/wrist_right_sphere_axis0/target', Float32, queue_size=2)
    #ros_right_wrist_1_pub = rospy.Publisher('/wrist_right_sphere_axis1/wrist_right_sphere_axis1/target', Float32, queue_size=2)
    #ros_right_wrist_2_pub = rospy.Publisher('/wrist_right_sphere_axis2/wrist_right_sphere_axis2/target', Float32, queue_size=2)
    ros_left_wrist_0_pub = rospy.Publisher('/wrist_left_sphere_axis0/wrist_left_sphere_axis0/target', Float32, queue_size=2)
    #ros_left_wrist_1_pub = rospy.Publisher('/wrist_left_sphere_axis1/wrist_left_sphere_axis1/target', Float32, queue_size=2)
    #ros_left_wrist_2_pub = rospy.Publisher('/wrist_left_sphere_axis2/wrist_left_sphere_axis2/target', Float32, queue_size=2)

    setJointControllerParameters(100, 0)

    jointAngleDict = {}
    jointAngleDict["num_points"] = NUM_STEERING_ANGLES

    for pointIter in range(NUM_STEERING_ANGLES):
        print("Capturing point number ", pointIter)
        thisRightHandX = _rightHandTrajectory[pointIter][0]
        thisRightHandY = _rightHandTrajectory[pointIter][1]
        thisRightHandZ = RIKSHAW_TURN_JOINT_Z_OFFSET + HANDLEBAR_Z_OFFSET
        thisLeftHandX = _leftHandTrajectory[pointIter][0]
        thisLeftHandY = _leftHandTrajectory[pointIter][1]
        thisLeftHandZ = RIKSHAW_TURN_JOINT_Z_OFFSET + HANDLEBAR_Z_OFFSET
        thisSteeringAngle = _steeringAngles[pointIter]
        thisRoll = 0
        thisPitch = 0
        thisYaw = thisSteeringAngle
        jointAngleResult_right = inverse_kinematics_client(ENDEFFECTOR_RIGHT, FRAME_RIGHT, thisRightHandX,
                                                           thisRightHandY, thisRightHandZ, thisRoll, thisPitch,
                                                           thisYaw - YAW_RIGHT_HAND_OFFSET)
        print("ik result fetched for right hand")
        jointAngleResult_left = inverse_kinematics_client(ENDEFFECTOR_LEFT, FRAME_LEFT, thisLeftHandX, thisLeftHandY,
                                                          thisLeftHandZ, thisRoll, thisPitch,
                                                          thisYaw - YAW_LEFT_HAND_OFFSET)
        print("ik result fetched for left hand")
        if jointAngleResult_right and jointAngleResult_left:
            jointAngleDict["point_" + str(pointIter)] = {}
            jointAngleDict["point_" + str(pointIter)]["Left"] = {}
            jointAngleDict["point_" + str(pointIter)]["Right"] = {}
            jointAngleDict["point_" + str(pointIter)]["Left"]["Steering_angle"] = thisSteeringAngle
            jointAngleDict["point_" + str(pointIter)]["Left"][JOINT_SHOULDER_AXIS0_LEFT] = jointAngleResult_left[
                JOINT_SHOULDER_AXIS0_LEFT]
            jointAngleDict["point_" + str(pointIter)]["Left"][JOINT_SHOULDER_AXIS1_LEFT] = jointAngleResult_left[
                JOINT_SHOULDER_AXIS1_LEFT]
            jointAngleDict["point_" + str(pointIter)]["Left"][JOINT_SHOULDER_AXIS2_LEFT] = jointAngleResult_left[
                JOINT_SHOULDER_AXIS2_LEFT]
            jointAngleDict["point_" + str(pointIter)]["Left"][JOINT_ELBOW_LEFT] = jointAngleResult_left[
                JOINT_ELBOW_LEFT]
            jointAngleDict["point_" + str(pointIter)]["Left"][JOINT_WRIST_LEFT_SPHERE_AXIS0] = jointAngleResult_left[
                JOINT_WRIST_LEFT_SPHERE_AXIS0]
            #jointAngleDict["point_" + str(pointIter)]["Left"][JOINT_WRIST_LEFT_SPHERE_AXIS1] = jointAngleResult_left[
            #    JOINT_WRIST_LEFT_SPHERE_AXIS1]
            #jointAngleDict["point_" + str(pointIter)]["Left"][JOINT_WRIST_LEFT_SPHERE_AXIS2] = jointAngleResult_left[
            #    JOINT_WRIST_LEFT_SPHERE_AXIS2]
            jointAngleDict["point_" + str(pointIter)]["Right"]["Steering_angle"] = thisSteeringAngle
            jointAngleDict["point_" + str(pointIter)]["Right"][JOINT_SHOULDER_AXIS0_RIGHT] = jointAngleResult_right[
                JOINT_SHOULDER_AXIS0_RIGHT]
            jointAngleDict["point_" + str(pointIter)]["Right"][JOINT_SHOULDER_AXIS1_RIGHT] = jointAngleResult_right[
                JOINT_SHOULDER_AXIS1_RIGHT]
            jointAngleDict["point_" + str(pointIter)]["Right"][JOINT_SHOULDER_AXIS2_RIGHT] = jointAngleResult_right[
                JOINT_SHOULDER_AXIS2_RIGHT]
            jointAngleDict["point_" + str(pointIter)]["Right"][JOINT_ELBOW_RIGHT] = jointAngleResult_right[
                JOINT_ELBOW_RIGHT]
            jointAngleDict["point_" + str(pointIter)]["Right"][JOINT_WRIST_RIGHT_SPHERE_AXIS0] = jointAngleResult_right[
                JOINT_WRIST_RIGHT_SPHERE_AXIS0]
            #jointAngleDict["point_" + str(pointIter)]["Right"][JOINT_WRIST_RIGHT_SPHERE_AXIS1] = jointAngleResult_right[
            #    JOINT_WRIST_RIGHT_SPHERE_AXIS1]
            #jointAngleDict["point_" + str(pointIter)]["Right"][JOINT_WRIST_RIGHT_SPHERE_AXIS2] = jointAngleResult_right[
            #    JOINT_WRIST_RIGHT_SPHERE_AXIS2]

            ros_left_shoulder_axis0_pub.publish(jointAngleResult_left[JOINT_SHOULDER_AXIS0_LEFT])
            ros_left_shoulder_axis1_pub.publish(jointAngleResult_left[JOINT_SHOULDER_AXIS1_LEFT])
            ros_left_shoulder_axis2_pub.publish(jointAngleResult_left[JOINT_SHOULDER_AXIS2_LEFT])
            ros_right_shoulder_axis0_pub.publish(jointAngleResult_right[JOINT_SHOULDER_AXIS0_RIGHT])
            ros_right_shoulder_axis1_pub.publish(jointAngleResult_right[JOINT_SHOULDER_AXIS1_RIGHT])
            ros_right_shoulder_axis2_pub.publish(jointAngleResult_right[JOINT_SHOULDER_AXIS2_RIGHT])
            ros_elbow_left_pub.publish(jointAngleResult_left[JOINT_ELBOW_LEFT])
            ros_elbow_right_pub.publish(jointAngleResult_right[JOINT_ELBOW_RIGHT])
            ros_left_wrist_0_pub.publish(jointAngleResult_left[JOINT_WRIST_LEFT_SPHERE_AXIS0])
            #ros_left_wrist_1_pub.publish(jointAngleResult_left[JOINT_WRIST_LEFT_SPHERE_AXIS1])
            #ros_left_wrist_2_pub.publish(jointAngleResult_left[JOINT_WRIST_LEFT_SPHERE_AXIS2])
            ros_right_wrist_0_pub.publish(jointAngleResult_right[JOINT_WRIST_RIGHT_SPHERE_AXIS0])
            #ros_right_wrist_1_pub.publish(jointAngleResult_right[JOINT_WRIST_RIGHT_SPHERE_AXIS1])
            #ros_right_wrist_2_pub.publish(jointAngleResult_right[JOINT_WRIST_RIGHT_SPHERE_AXIS2])

            while abs(_jointsStatusData[JOINT_SHOULDER_AXIS0_LEFT]["Pos"] - jointAngleResult_left[
                JOINT_SHOULDER_AXIS0_LEFT]) > JOINT_ANGLE_TOLERANCE_FK:
                time.sleep(0.1)
            print("JOINT_SHOULDER_AXIS0_LEFT moved to new position")
            while abs(_jointsStatusData[JOINT_SHOULDER_AXIS1_LEFT]["Pos"] - jointAngleResult_left[
                JOINT_SHOULDER_AXIS1_LEFT]) > JOINT_ANGLE_TOLERANCE_FK:
                time.sleep(0.1)
            print("JOINT_SHOULDER_AXIS1_LEFT moved to new position")
            while abs(_jointsStatusData[JOINT_SHOULDER_AXIS2_LEFT]["Pos"] - jointAngleResult_left[
                JOINT_SHOULDER_AXIS2_LEFT]) > JOINT_ANGLE_TOLERANCE_FK:
                time.sleep(0.1)
            print("JOINT_SHOULDER_AXIS2_LEFT moved to new position")

            while abs(_jointsStatusData[JOINT_SHOULDER_AXIS0_RIGHT]["Pos"] - jointAngleResult_right[
                JOINT_SHOULDER_AXIS0_RIGHT]) > JOINT_ANGLE_TOLERANCE_FK:
                time.sleep(0.1)
            print("JOINT_SHOULDER_AXIS0_RIGHT moved to new position")
            while abs(_jointsStatusData[JOINT_SHOULDER_AXIS1_RIGHT]["Pos"] - jointAngleResult_right[
                JOINT_SHOULDER_AXIS1_RIGHT]) > JOINT_ANGLE_TOLERANCE_FK:
                time.sleep(0.1)
            print("JOINT_SHOULDER_AXIS1_RIGHT moved to new position")
            while abs(_jointsStatusData[JOINT_SHOULDER_AXIS2_RIGHT]["Pos"] - jointAngleResult_right[
                JOINT_SHOULDER_AXIS2_RIGHT]) > JOINT_ANGLE_TOLERANCE_FK:
                time.sleep(0.1)
            print("JOINT_SHOULDER_AXIS2_RIGHT moved to new position")

            while abs(_jointsStatusData[JOINT_ELBOW_LEFT]["Pos"] - jointAngleResult_left[
                JOINT_ELBOW_LEFT]) > JOINT_ANGLE_TOLERANCE_FK:
                time.sleep(0.1)
            print("JOINT_ELBOW_LEFT moved to new position")
            while abs(_jointsStatusData[JOINT_ELBOW_RIGHT]["Pos"] - jointAngleResult_right[
                JOINT_ELBOW_RIGHT]) > JOINT_ANGLE_TOLERANCE_FK:
                time.sleep(0.1)
            print("JOINT_ELBOW_RIGHT moved to new position")

            # while abs(_jointsStatusData[JOINT_WRIST_LEFT_SPHERE_AXIS0]["Pos"] - jointAngleResult_left[
            #     JOINT_WRIST_LEFT_SPHERE_AXIS0]) > JOINT_ANGLE_TOLERANCE_FK:
            #     time.sleep(0.1)
            # print("JOINT_WRIST_LEFT_SPHERE_AXIS0 moved to new position")
            #while abs(_jointsStatusData[JOINT_WRIST_LEFT_SPHERE_AXIS1]["Pos"] - jointAngleResult_left[
            #    JOINT_WRIST_LEFT_SPHERE_AXIS1]) > JOINT_ANGLE_TOLERANCE_FK:
            #    time.sleep(0.1)
            #print("JOINT_WRIST_LEFT_SPHERE_AXIS1 moved to new position")
            #while abs(_jointsStatusData[JOINT_WRIST_LEFT_SPHERE_AXIS2]["Pos"] - jointAngleResult_left[
            #    JOINT_WRIST_LEFT_SPHERE_AXIS2]) > JOINT_ANGLE_TOLERANCE_FK:
            #    time.sleep(0.1)
            #print("JOINT_WRIST_LEFT_SPHERE_AXIS2 moved to new position")

            # while abs(_jointsStatusData[JOINT_WRIST_RIGHT_SPHERE_AXIS0]["Pos"] - jointAngleResult_right[
            #     JOINT_WRIST_RIGHT_SPHERE_AXIS0]) > JOINT_ANGLE_TOLERANCE_FK:
            #     time.sleep(0.1)
            # print("JOINT_WRIST_RIGHT_SPHERE_AXIS0 moved to new position")
            # while abs(_jointsStatusData[JOINT_WRIST_RIGHT_SPHERE_AXIS1]["Pos"] - jointAngleResult_right[
            #     JOINT_WRIST_RIGHT_SPHERE_AXIS1]) > JOINT_ANGLE_TOLERANCE_FK:
            #     time.sleep(0.1)
            # print("JOINT_WRIST_RIGHT_SPHERE_AXIS1 moved to new position")
            # while abs(_jointsStatusData[JOINT_WRIST_RIGHT_SPHERE_AXIS2]["Pos"] - jointAngleResult_right[
            #     JOINT_WRIST_RIGHT_SPHERE_AXIS2]) > JOINT_ANGLE_TOLERANCE_FK:
            #     time.sleep(0.1)
            # print("JOINT_WRIST_RIGHT_SPHERE_AXIS2 moved to new position")

            jointAngleDict["point_" + str(pointIter)]["Right"]["Hand_actual"] = getPositionRightHand()
            jointAngleDict["point_" + str(pointIter)]["Left"]["Hand_actual"] = getPositionLeftHand()
            print("Moving on...")

        else:
            jointAngleDict["num_points"] = jointAngleDict["num_points"] - 1

        print("Finished point ", pointIter)

    # print(jointAngleDict)
    with open(JSON_FILENAME, "w") as write_file:
        json.dump(jointAngleDict, write_file, indent=4, sort_keys=True)

    return 1


if __name__ == '__main__':
    main()
