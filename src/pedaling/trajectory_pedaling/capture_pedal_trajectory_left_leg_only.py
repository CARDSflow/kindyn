## @package pedaling

import json
import math
import sys
import time

import numpy as np

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Pose, Point
from roboy_control_msgs.srv import SetControllerParameters
from roboy_middleware_msgs.srv import InverseKinematics, InverseKinematicsMultipleFrames, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from std_msgs.msg import Float32

JSON_FILENAME = "captured_pedal_trajectory_09mar_with_joint_limits.json"  # "captured_trajectory_two_frames.json"
TEMP_READFILE = "captured_pedal_trajectory_WHATmar_with_joint_limits.json"

JOINT_ANGLE_TOLERANCE_FK = 0.002
JOINT_KP = 200
JOINT_KD = 0
POINT_REACHED_TOLERANCE = 0.02

LEFT_HIP_JOINT_TEST_CONFIGURATIONS = np.linspace(-0.9, -0.8, 20)
LEFT_KNEE_JOINT_TEST_CONFIGURATIONS = np.linspace(1.9, 1.8, 5)
LEFT_ANKLE_JOINT_TEST_CONFIGURATIONS = np.linspace(0.2, 0.0, 5)

LEFT_HIP_JOINT_LOWER_LIMIT = -1.9
LEFT_KNEE_JOINT_LOWER_LIMIT = 0
LEFT_ANKLE_JOINT_LOWER_LIMIT = -0.7

LEFT_HIP_JOINT_UPPER_LIMIT = 0.5
LEFT_KNEE_JOINT_UPPER_LIMIT = 2.0
LEFT_ANKLE_JOINT_UPPER_LIMIT = 0.7

###############################
###   MEASURED PARAMETERS   ###
###############################

PEDAL_CENTER_OFFSET_X = 0.09222872619138603  # 0.20421
PEDAL_CENTER_OFFSET_Y = 0.00013171801209023543  # -0.00062
PEDAL_CENTER_OFFSET_Z = 0.0042772834965418725  # 0.2101

BIKE_OFFSET_X = 0  # -0.83471
BIKE_OFFSET_Y = 0  # 0.03437
BIKE_OFFSET_Z = 0  # 0.037

PEDAL_RADIUS = 0.16924  # [meters]

RIGHT_LEG_OFFSET_Y = 0.1579606226770175 - 0.00013171801209023543
LEFT_LEG_OFFSET_Y = -0.1873561275007122 - 0.00013171801209023543

############################
###   GLOBAL VARIABLES   ###
############################

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


##############################
###   UTILITY FUNCTIONS   ###
##############################

## Documentation for a function
#
#  Return the distance between two points where points are a list of two coordinates.
def get_distance(point1, point2):
    x_diff = point2[ 0 ] - point1[ 0 ]
    y_diff = point2[ 1 ] - point1[ 1 ]

    return math.sqrt((x_diff * x_diff) + (y_diff * y_diff))

def getPositionLeftFoot():
    fkJointNamesList = [ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT]
    fkJointPositions = [_jointsStatusData[LEFT_HIP_JOINT]["Pos"], _jointsStatusData[LEFT_KNEE_JOINT]["Pos"],
                        _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("left_leg", "foot_left_tip", fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    print("ERROR fk foot_left failed")
    return [0.0, 0.0]  # [x, z]


def getPositionRightFoot():
    fkJointNamesList = [ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT]
    fkJointPositions = [_jointsStatusData[RIGHT_HIP_JOINT]["Pos"], _jointsStatusData[RIGHT_KNEE_JOINT]["Pos"],
                        _jointsStatusData[RIGHT_ANKLE_JOINT]["Pos"]]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("right_leg", "foot_right_tip", fkJointNamesList, fkJointPositions)
        return [fk_result.pose.position.x, fk_result.pose.position.z]

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    print("ERROR fk foot_right failed")
    return [0.0, 0.0]  # [x, z]


def setJointControllerParameters(proportionalVal, derivativeVal):
    rospy.wait_for_service('joint_foot_left/joint_foot_left/params')
    try:
        foot_left_srv = rospy.ServiceProxy('joint_foot_left/joint_foot_left/params', SetControllerParameters)
        foot_left_srv(proportionalVal, derivativeVal)
    except rospy.ServiceException as e:
        print("Service call joint_foot_left failed:", e)

    rospy.wait_for_service('joint_foot_right/joint_foot_right/params')
    try:
        foot_right_srv = rospy.ServiceProxy('joint_foot_right/joint_foot_right/params', SetControllerParameters)
        foot_right_srv(proportionalVal, derivativeVal)
    except rospy.ServiceException as e:
        print("Service call joint_foot_right failed:", e)

    rospy.wait_for_service('joint_knee_left/joint_knee_left/params')
    try:
        knee_left_srv = rospy.ServiceProxy('joint_knee_left/joint_knee_left/params', SetControllerParameters)
        knee_left_srv(proportionalVal, derivativeVal)
    except rospy.ServiceException as e:
        print("Service call joint_knee_left failed:", e)

    rospy.wait_for_service('joint_knee_right/joint_knee_right/params')
    try:
        knee_right_srv = rospy.ServiceProxy('joint_knee_right/joint_knee_right/params', SetControllerParameters)
        knee_right_srv(proportionalVal, derivativeVal)
    except rospy.ServiceException as e:
        print("Service call joint_knee_right failed:", e)

    rospy.wait_for_service('joint_hip_left/joint_hip_left/params')
    try:
        hip_left_srv = rospy.ServiceProxy('joint_hip_left/joint_hip_left/params', SetControllerParameters)
        hip_left_srv(proportionalVal, derivativeVal)
    except rospy.ServiceException as e:
        print("Service call joint_hip_left failed:", e)

    rospy.wait_for_service('joint_hip_right/joint_hip_right/params')
    try:
        hip_right_srv = rospy.ServiceProxy('joint_hip_right/joint_hip_right/params', SetControllerParameters)
        hip_right_srv(proportionalVal, derivativeVal)
    except rospy.ServiceException as e:
        print("Service call joint_hip_right failed:", e)

    print("Controller paramters updated")


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


def getPedalPositions(numSamples):
    # Format: [[pedal_angle_1, x, y, z], [pedal_angle_2, x, y, z], ...]
    capturedPositions = []
    angularStep = 2.0 * math.pi / numSamples
    for sampleIterator in range(numSamples + 1):
        thisPedalAngle = sampleIterator * angularStep
        thisXVal = PEDAL_CENTER_OFFSET_X + math.cos(thisPedalAngle) * PEDAL_RADIUS
        thisYVal = PEDAL_CENTER_OFFSET_Y
        thisZVal = PEDAL_CENTER_OFFSET_Z + math.sin(thisPedalAngle) * PEDAL_RADIUS
        capturedPositions.append([thisPedalAngle, thisXVal, thisYVal, thisZVal])

    return capturedPositions


def plotPedalTrajectories():
    numSamples = []
    if len(sys.argv) > 1:
        for argIterator in range(1, len(sys.argv)):
            numSamples.append(int(sys.argv[argIterator]))

    plt.title('Pedal trajectory estimation by number of intermediate points')

    for thisSample in numSamples:
        capturedPositions = getPedalPositions(thisSample)
        x_values = []
        z_values = []
        for pointIterator in capturedPositions:
            x_values.append(pointIterator[1])
            z_values.append(pointIterator[3])
        plt.plot(x_values, z_values, label=str(thisSample))
    plt.legend()
    plt.show()


def plotEverything(numSamples, jointAngleDict):
    plt.figure(1)
    plt.title('Pedal trajectory planned and ik result')
    capturedPositions = getPedalPositions(numSamples)
    x_values = []
    z_values = []
    for pointIterator in capturedPositions:
        x_values.append(pointIterator[1])
        z_values.append(pointIterator[3])
    plt.plot(x_values, z_values, label="Ideal")
    for pointIter in range(jointAngleDict["num_points"]):
        if "point_" + str(pointIter) in jointAngleDict:
            if "Pedal" in jointAngleDict["point_" + str(pointIter)]["Left"]:
                plt.plot(jointAngleDict["point_" + str(pointIter)]["Left"]["Pedal"][0],
                         jointAngleDict["point_" + str(pointIter)]["Left"]["Pedal"][1], 'gs', label="IK recorded left")
                plt.plot(jointAngleDict["point_" + str(pointIter)]["Left"]["Pedal_actual"][0],
                         jointAngleDict["point_" + str(pointIter)]["Left"]["Pedal_actual"][1], 'rs',
                         label="Actual left")


    plt.figure(2)
    plt.title('Hip positions left')
    x_values = []
    z_values = []
    tot_points = jointAngleDict["num_points"]
    for pointIter in range(tot_points):
        if "point_" + str(pointIter) in jointAngleDict:
            if "Hip" in jointAngleDict["point_" + str(pointIter)]["Left"]:
                x_values.append(pointIter * (2 * math.pi / tot_points))
                z_values.append(jointAngleDict["point_" + str(pointIter)]["Left"]["Hip"])
    plt.plot(x_values, z_values)


    plt.figure(4)
    plt.title('Knee positions left')
    x_values = []
    z_values = []
    tot_points = jointAngleDict["num_points"]
    for pointIter in range(tot_points):
        if "point_" + str(pointIter) in jointAngleDict:
            if "Knee" in jointAngleDict["point_" + str(pointIter)]["Left"]:
                x_values.append(pointIter * (2 * math.pi / tot_points))
                z_values.append(jointAngleDict["point_" + str(pointIter)]["Left"]["Knee"])
    plt.plot(x_values, z_values)


    plt.figure(6)
    plt.title('Ankle positions left')
    x_values = []
    z_values = []
    tot_points = jointAngleDict["num_points"]
    for pointIter in range(tot_points):
        if "point_" + str(pointIter) in jointAngleDict:
            if "Ankle" in jointAngleDict["point_" + str(pointIter)]["Left"]:
                x_values.append(pointIter * (2 * math.pi / tot_points))
                z_values.append(jointAngleDict["point_" + str(pointIter)]["Left"]["Ankle"])
    plt.plot(x_values, z_values)


    plt.show()


def inverse_kinematics_client(endeffector, frame, x, y, z):
    rospy.wait_for_service('ik')
    try:
        ik_srv = rospy.ServiceProxy('ik', InverseKinematics)
        requested_position = Point(x, y, z)
        requested_pose = Pose(position=requested_position)
        requested_ik_type = 1  # Position only
        ik_result = ik_srv(endeffector, requested_ik_type, frame, requested_pose)

        jointDict = {}
        for thisJoint in range(len(ik_result.angles)):
            jointDict[ik_result.joint_names[thisJoint]] = ik_result.angles[thisJoint]

        return jointDict

    except rospy.ServiceException as e:
        print("Service call failed:", e)


def inverse_kinematics_multiple_frames_client(endeffector, frames, x, y, z, weights):
    rospy.wait_for_service('ik_multiple_frames')
    try:
        ik_srv = rospy.ServiceProxy('ik_multiple_frames', InverseKinematicsMultipleFrames)
        requested_poses = []
        for requestIterator in range(len(frames)):
            this_position = Point(x[requestIterator], y[requestIterator], z[requestIterator])
            requested_poses.append(Pose(position=this_position))
        requested_ik_type = 1  # Position only

        print("****** ik multiple frames: ******")
        print("endeffector:", endeffector, "frames:", frames, "poses:", requested_poses, "weights:", weights)
        print("*********************************")
        ik_result = ik_srv(endeffector, requested_ik_type, frames, requested_poses, weights)

        jointDict = {}
        for thisJoint in range(len(ik_result.angles)):
            jointDict[ik_result.joint_names[thisJoint]] = ik_result.angles[thisJoint]

        return jointDict


    except rospy.ServiceException as e:
        print("Service call failed:", e)


################
###   MAIN   ###
################

def main_notnow():
    with open(TEMP_READFILE, "r") as read_file:
        jointAngleDict = json.load(read_file)

    plotEverything(72, jointAngleDict)







def main():
    if len(sys.argv) > 1:
        num_requested_points = int(sys.argv[1])
    else:
        num_requested_points = 72

    global JOINT_ANGLE_TOLERANCE_FK

    rospy.init_node('pedal_simulation', anonymous=True)
    rospy.Subscriber("joint_state", JointState, jointStateCallback)
    #ros_right_hip_publisher = rospy.Publisher('joint_hip_right/joint_hip_right/target', Float32, queue_size=1)
    #ros_right_knee_publisher = rospy.Publisher('joint_knee_right/joint_knee_right/target', Float32, queue_size=1)
    #ros_right_ankle_publisher = rospy.Publisher('joint_foot_right/joint_foot_right/target', Float32, queue_size=1)
    ros_left_hip_publisher = rospy.Publisher('joint_hip_left/joint_hip_left/target', Float32, queue_size=1)
    ros_left_knee_publisher = rospy.Publisher('joint_knee_left/joint_knee_left/target', Float32, queue_size=1)
    ros_left_ankle_publisher = rospy.Publisher('joint_foot_left/joint_foot_left/target', Float32, queue_size=1)
    time.sleep(2)

    #setJointControllerParameters(JOINT_KP, JOINT_KD)

    capturedPositions = getPedalPositions(num_requested_points)

    endeffector_right = "right_leg"
    frame_right = "foot_right_tip"
    y_offset_right = RIGHT_LEG_OFFSET_Y
    frame_right_1 = "foot_right_tip"
    weight_right_1 = 1
    frame_right_2 = "thigh_right"
    weight_right_2 = 1
    frame_right_2_x_offset = -0.1
    frame_right_2_y_offset = 0
    frame_right_2_z_offset = 0.3

    endeffector_left = "left_leg"
    frame_left = "foot_left_tip"
    y_offset_left = LEFT_LEG_OFFSET_Y
    frame_left_1 = "foot_left_tip"
    weight_left_1 = 1
    frame_left_2 = "thigh_left"
    weight_left_2 = 1
    frame_left_2_x_offset = -0.1
    frame_left_2_y_offset = 0
    frame_left_2_z_offset = 0.3

    jointAngleDict = {}
    jointAngleDict["num_points"] = num_requested_points

    firstPointReached = False
    for pointIter in range(num_requested_points):
        print("Capturing point number ", pointIter)
        thisX = capturedPositions[pointIter][1]
        thisZ = capturedPositions[pointIter][3]
        thisPedalAngle = capturedPositions[pointIter][0]

        reached_point = False
        currBestDistance = 999
        while (not reached_point):

            hip_test_position_iterator = 0
            knee_test_position_iterator = 0
            ankle_test_position_iterator = 0

            if firstPointReached:

                lastCorrectHipAngle = _jointsStatusData[LEFT_HIP_JOINT]["Pos"]
                lastCorrectKneeAngle = _jointsStatusData[LEFT_KNEE_JOINT]["Pos"]
                lastCorrectAnkleAngle = _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"]

                upperTestHipAngle = lastCorrectHipAngle+0.15
                upperTestKneeAngle = lastCorrectKneeAngle+0.15
                upperTestAnkleAngle = lastCorrectAnkleAngle+0.2

                lowerTestAnkleAngle = lastCorrectAnkleAngle-0.2
                lowerTestKneeAngle = lastCorrectKneeAngle-0.15
                lowerTestHipAngle = lastCorrectHipAngle-0.15

                if (upperTestHipAngle > LEFT_HIP_JOINT_UPPER_LIMIT):
                    upperTestHipAngle = LEFT_HIP_JOINT_UPPER_LIMIT
                elif (upperTestHipAngle < LEFT_HIP_JOINT_LOWER_LIMIT):
                    upperTestHipAngle = LEFT_HIP_JOINT_LOWER_LIMIT

                if (lowerTestHipAngle > LEFT_HIP_JOINT_UPPER_LIMIT):
                    lowerTestHipAngle = LEFT_HIP_JOINT_UPPER_LIMIT
                elif (lowerTestHipAngle < LEFT_HIP_JOINT_LOWER_LIMIT):
                    lowerTestHipAngle = LEFT_HIP_JOINT_LOWER_LIMIT

                if (upperTestKneeAngle > LEFT_KNEE_JOINT_UPPER_LIMIT):
                    upperTestKneeAngle = LEFT_KNEE_JOINT_UPPER_LIMIT
                elif (upperTestKneeAngle < LEFT_KNEE_JOINT_LOWER_LIMIT):
                    upperTestKneeAngle = LEFT_KNEE_JOINT_LOWER_LIMIT

                if (lowerTestKneeAngle > LEFT_KNEE_JOINT_UPPER_LIMIT):
                    lowerTestKneeAngle = LEFT_KNEE_JOINT_UPPER_LIMIT
                elif (lowerTestKneeAngle < LEFT_KNEE_JOINT_LOWER_LIMIT):
                    lowerTestKneeAngle = LEFT_KNEE_JOINT_LOWER_LIMIT

                if (upperTestAnkleAngle > LEFT_ANKLE_JOINT_UPPER_LIMIT):
                    upperTestAnkleAngle = LEFT_ANKLE_JOINT_UPPER_LIMIT
                elif (upperTestAnkleAngle < LEFT_ANKLE_JOINT_LOWER_LIMIT):
                    upperTestAnkleAngle = LEFT_ANKLE_JOINT_LOWER_LIMIT

                if (lowerTestAnkleAngle > LEFT_ANKLE_JOINT_UPPER_LIMIT):
                    lowerTestAnkleAngle = LEFT_ANKLE_JOINT_UPPER_LIMIT
                elif (lowerTestAnkleAngle < LEFT_ANKLE_JOINT_LOWER_LIMIT):
                    lowerTestAnkleAngle = LEFT_ANKLE_JOINT_LOWER_LIMIT

                ankleTestPositionsList = np.linspace(lowerTestAnkleAngle, upperTestAnkleAngle, 10)
                kneeTestPositionsList = np.linspace(lowerTestKneeAngle, upperTestKneeAngle, 20)
                hipTestPositionsList = np.linspace(lowerTestHipAngle, upperTestHipAngle, 20)

                print("Inside firstPointReached, lastCorrectHipAngle:", lastCorrectHipAngle, "upperTestHipAngle", upperTestHipAngle, "lowerTestHipAngle", lowerTestHipAngle, "hipTestPositionsList:", hipTestPositionsList)
                print("Inside firstPointReached, lastCorrectKneeAngle:", lastCorrectKneeAngle, "upperTestKneeAngle", upperTestKneeAngle, "lowerTestKneeAngle", lowerTestKneeAngle, "kneeTestPositionsList:", kneeTestPositionsList)
                print("Inside firstPointReached, lastCorrectAnkleAngle:", lastCorrectAnkleAngle, "upperTestAnkleAngle", upperTestAnkleAngle, "lowerTestAnkleAngle", lowerTestAnkleAngle, "ankleTestPositionsList:", ankleTestPositionsList)

                currBestDistance = 999
                currBestHipAngle = 0
                currBestKneeAngle = 0
                currBestAnkleAngle = 0

                for ankle_test_position_iterator in range(len(ankleTestPositionsList)):
                    if reached_point:
                        break

                    if ankle_test_position_iterator % 2 is 0:
                        ankle_test_position = ankleTestPositionsList[len(ankleTestPositionsList)/2 + ankle_test_position_iterator/2]
                    else:
                        ankle_test_position = ankleTestPositionsList[len(ankleTestPositionsList)/2 - int(ankle_test_position_iterator/2)]

                    for knee_test_position_iterator in range(len(kneeTestPositionsList)):
                        if reached_point:
                            break

                        if knee_test_position_iterator % 2 is 0:
                            knee_test_position = kneeTestPositionsList[len(kneeTestPositionsList)/2 + knee_test_position_iterator/2]
                        else:
                            knee_test_position = kneeTestPositionsList[len(kneeTestPositionsList)/2 - int(knee_test_position_iterator/2)]


                        for hip_test_position_iterator in range(len(hipTestPositionsList)):
                            if reached_point:
                                break

                            if hip_test_position_iterator % 2 is 0:
                                hip_test_position = hipTestPositionsList[len(hipTestPositionsList)/2 + hip_test_position_iterator/2]
                            else:
                                hip_test_position = hipTestPositionsList[len(hipTestPositionsList)/2 - int(hip_test_position_iterator/2)]

                            ros_left_hip_publisher.publish(hip_test_position)
                            ros_left_knee_publisher.publish(knee_test_position)
                            ros_left_ankle_publisher.publish(ankle_test_position)
                            print("Target angles for point number", pointIter, "out of", num_requested_points, "published: hip=", hip_test_position, "knee=", knee_test_position, "ankle=", ankle_test_position, "worst case positions left to try: ", (len(hipTestPositionsList)-hip_test_position_iterator)+ (len(hipTestPositionsList))*(len(kneeTestPositionsList)-knee_test_position_iterator) +(len(kneeTestPositionsList))*(len(hipTestPositionsList))*(len(ankleTestPositionsList)-ankle_test_position_iterator) )
                            while abs(_jointsStatusData[LEFT_HIP_JOINT]["Pos"] - hip_test_position) > JOINT_ANGLE_TOLERANCE_FK:
                                time.sleep(0.001)
                            while abs(_jointsStatusData[LEFT_KNEE_JOINT]["Pos"] - knee_test_position) > JOINT_ANGLE_TOLERANCE_FK:
                                time.sleep(0.001)
                            while abs(_jointsStatusData[LEFT_ANKLE_JOINT]["Pos"] - ankle_test_position) > JOINT_ANGLE_TOLERANCE_FK:
                                time.sleep(0.001)

                            currDistanceToGoalPoint = get_distance(getPositionLeftFoot(), [thisX, thisZ])
                            if currDistanceToGoalPoint < currBestDistance:
                                currBestDistance = currDistanceToGoalPoint
                                currBestHipAngle = hip_test_position
                                currBestKneeAngle = knee_test_position
                                currBestAnkleAngle = ankle_test_position
                            print("best distance to goal point:", currBestDistance,"this distance:", currDistanceToGoalPoint, "tolerance:", POINT_REACHED_TOLERANCE)
                            if (currDistanceToGoalPoint < POINT_REACHED_TOLERANCE):
                                reached_point = True

                ros_left_hip_publisher.publish(currBestHipAngle)
                ros_left_knee_publisher.publish(currBestKneeAngle)
                ros_left_ankle_publisher.publish(currBestAnkleAngle)
                print("Target angles for point number", pointIter, "published: hip=", currBestHipAngle, "knee=", currBestKneeAngle, "ankle=", currBestAnkleAngle)
                while abs(_jointsStatusData[LEFT_HIP_JOINT]["Pos"] - currBestHipAngle) > JOINT_ANGLE_TOLERANCE_FK:
                    time.sleep(0.001)
                while abs(_jointsStatusData[LEFT_KNEE_JOINT]["Pos"] - currBestKneeAngle) > JOINT_ANGLE_TOLERANCE_FK:
                    time.sleep(0.001)
                while abs(_jointsStatusData[LEFT_ANKLE_JOINT]["Pos"] - currBestAnkleAngle) > JOINT_ANGLE_TOLERANCE_FK:
                    time.sleep(0.001)
                reached_point = True
            else:

                currBestDistance = 999

                for ankle_test_position in LEFT_ANKLE_JOINT_TEST_CONFIGURATIONS:
                    if reached_point:
                        break
                    for knee_test_position in LEFT_KNEE_JOINT_TEST_CONFIGURATIONS:
                        if reached_point:
                            break
                        for hip_test_position in LEFT_HIP_JOINT_TEST_CONFIGURATIONS:
                            if reached_point:
                                break

                            ros_left_hip_publisher.publish(hip_test_position)
                            ros_left_knee_publisher.publish(knee_test_position)
                            ros_left_ankle_publisher.publish(ankle_test_position)
                            print("Target angles for point number", pointIter, "published: hip=", hip_test_position, "knee=", knee_test_position, "ankle=", ankle_test_position)
                            while abs(_jointsStatusData[LEFT_HIP_JOINT]["Pos"] - hip_test_position) > JOINT_ANGLE_TOLERANCE_FK:
                                time.sleep(0.01)
                            while abs(_jointsStatusData[LEFT_KNEE_JOINT]["Pos"] - knee_test_position) > JOINT_ANGLE_TOLERANCE_FK:
                                time.sleep(0.01)
                            while abs(_jointsStatusData[LEFT_ANKLE_JOINT]["Pos"] - ankle_test_position) > JOINT_ANGLE_TOLERANCE_FK:
                                time.sleep(0.01)

                            currDistanceToGoalPoint = get_distance(getPositionLeftFoot(), [thisX, thisZ])
                            if currDistanceToGoalPoint < currBestDistance:
                                currBestDistance = currDistanceToGoalPoint
                            print("best distance to goal point:", currBestDistance,"this distance:", currDistanceToGoalPoint, "tolerance:", POINT_REACHED_TOLERANCE)

                            if (get_distance(getPositionLeftFoot(), [thisX, thisZ]) < POINT_REACHED_TOLERANCE):
                                reached_point = True
                                firstPointReached = True
                                currBestDistance = get_distance(getPositionLeftFoot(), [thisX, thisZ])

        jointAngleDict["point_" + str(pointIter)] = {}
        jointAngleDict["point_" + str(pointIter)]["Left"] = {}
        jointAngleDict["point_" + str(pointIter)]["Left"]["Pedal"] = [thisX, thisZ]
        jointAngleDict["point_" + str(pointIter)]["Left"]["Pedal_angle"] = thisPedalAngle
        jointAngleDict["point_" + str(pointIter)]["Left"]["Hip"] = _jointsStatusData[LEFT_HIP_JOINT]["Pos"]
        jointAngleDict["point_" + str(pointIter)]["Left"]["Knee"] = _jointsStatusData[LEFT_KNEE_JOINT]["Pos"]
        jointAngleDict["point_" + str(pointIter)]["Left"]["Ankle"] = _jointsStatusData[LEFT_ANKLE_JOINT]["Pos"]
        jointAngleDict["point_" + str(pointIter)]["Left"]["Pedal_actual"] = getPositionLeftFoot()
        jointAngleDict["point_" + str(pointIter)]["Left"]["Error"] = currBestDistance

        with open(JSON_FILENAME, "w") as write_file:
            json.dump(jointAngleDict, write_file, indent=4, sort_keys=True)

        print("Finished point ", pointIter)

    # print(jointAngleDict)
    with open(JSON_FILENAME, "w") as write_file:
        json.dump(jointAngleDict, write_file, indent=4, sort_keys=True)

    plotEverything(jointAngleDict["num_points"], jointAngleDict)

    return 1


if __name__ == '__main__':
    main()
