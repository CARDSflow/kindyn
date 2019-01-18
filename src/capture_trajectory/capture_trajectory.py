#! /usr/bin/env python
import math
import time
import sys
import json
import matplotlib.pyplot as plt

import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32

JSON_FILENAME = "captured_trajectory.json"

JOINT_ANGLE_TOLERANCE_FK = 0.05

###############################
###   MEASURED PARAMETERS   ###
###############################

PEDAL_CENTER_OFFSET_X = 0.20421
PEDAL_CENTER_OFFSET_Y = -0.00062
PEDAL_CENTER_OFFSET_Z = 0.2101

PEDAL_RADIUS = 0.16924  # [millimeters]

RIGHT_LEG_OFFSET_Y = 0.2095
LEFT_LEG_OFFSET_Y = -0.13815


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
    angularStep = 2.0*math.pi/numSamples
    for sampleIterator in range(numSamples+1):
        thisPedalAngle = sampleIterator*angularStep
        thisXVal = PEDAL_CENTER_OFFSET_X + math.cos(thisPedalAngle)*PEDAL_RADIUS
        thisYVal = PEDAL_CENTER_OFFSET_Y
        thisZVal = PEDAL_CENTER_OFFSET_Z + math.sin(thisPedalAngle)*PEDAL_RADIUS
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
        plt.plot(x_values, z_values,label=str(thisSample))
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
    plt.plot(x_values, z_values,label="Ideal")
    for pointIter in range(jointAngleDict["num_points"]):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Pedal" in jointAngleDict["point_"+str(pointIter)]["Left"]:
                plt.plot(jointAngleDict["point_"+str(pointIter)]["Left"]["Pedal"][0], jointAngleDict["point_"+str(pointIter)]["Left"]["Pedal"][1], 'rs',label="IK recorded left")
            if "Pedal" in jointAngleDict["point_"+str(pointIter)]["Right"]:
                plt.plot(jointAngleDict["point_"+str(pointIter)]["Right"]["Pedal"][0], jointAngleDict["point_"+str(pointIter)]["Right"]["Pedal"][1], 'gs',label="IK recorded right")


    plt.figure(2)
    plt.title('Hip positions left')
    x_values = []
    z_values = []
    tot_points = jointAngleDict["num_points"]
    for pointIter in range(tot_points):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Hip" in jointAngleDict["point_"+str(pointIter)]["Left"]:
                x_values.append(pointIter * (2 * math.pi / tot_points))
                z_values.append(jointAngleDict["point_"+str(pointIter)]["Left"]["Hip"])
    plt.plot(x_values, z_values)

    plt.figure(3)
    plt.title('Hip positions right')
    x_values = []
    z_values = []
    tot_points = jointAngleDict["num_points"]
    for pointIter in range(tot_points):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Hip" in jointAngleDict["point_"+str(pointIter)]["Right"]:
                x_values.append(pointIter * (2 * math.pi / tot_points))
                z_values.append(jointAngleDict["point_"+str(pointIter)]["Right"]["Hip"])
    plt.plot(x_values, z_values)


    plt.figure(4)
    plt.title('Knee positions left')
    x_values = []
    z_values = []
    tot_points = jointAngleDict["num_points"]
    for pointIter in range(tot_points):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Knee" in jointAngleDict["point_"+str(pointIter)]["Left"]:
                x_values.append(pointIter * (2 * math.pi / tot_points))
                z_values.append(jointAngleDict["point_"+str(pointIter)]["Left"]["Knee"])
    plt.plot(x_values, z_values)

    plt.figure(5)
    plt.title('Knee positions right')
    x_values = []
    z_values = []
    tot_points = jointAngleDict["num_points"]
    for pointIter in range(tot_points):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Knee" in jointAngleDict["point_"+str(pointIter)]["Right"]:
                x_values.append(pointIter * (2 * math.pi / tot_points))
                z_values.append(jointAngleDict["point_"+str(pointIter)]["Right"]["Knee"])
    plt.plot(x_values, z_values)

    plt.figure(6)
    plt.title('Ankle positions left')
    x_values = []
    z_values = []
    tot_points = jointAngleDict["num_points"]
    for pointIter in range(tot_points):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Ankle" in jointAngleDict["point_"+str(pointIter)]["Left"]:
                x_values.append(pointIter * (2 * math.pi / tot_points))
                z_values.append(jointAngleDict["point_"+str(pointIter)]["Left"]["Ankle"])
    plt.plot(x_values, z_values)

    plt.figure(7)
    plt.title('Ankle positions right')
    x_values = []
    z_values = []
    tot_points = jointAngleDict["num_points"]
    for pointIter in range(tot_points):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Ankle" in jointAngleDict["point_"+str(pointIter)]["Right"]:
                x_values.append(pointIter * (2 * math.pi / tot_points))
                z_values.append(jointAngleDict["point_"+str(pointIter)]["Right"]["Ankle"])
    plt.plot(x_values, z_values)

    plt.show()



def inverse_kinematics_client(endeffector, frame, x, y, z):
    rospy.wait_for_service('ik')
    try:
        ik_srv = rospy.ServiceProxy('ik', InverseKinematics)
        requested_position = Point(x, y, z)
        requested_pose = Pose(position=requested_position)
        requested_ik_type = 1  #Position only
        ik_result = ik_srv(endeffector, requested_ik_type, frame, requested_pose)

        jointDict = {}
        for thisJoint in range(len(ik_result.angles)):
            jointDict[ik_result.joint_names[thisJoint]] = ik_result.angles[thisJoint]

        return jointDict

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


################
###   MAIN   ###
################

def main():

    if len(sys.argv) > 1:
        num_requested_points = int(sys.argv[1])
    else:
        num_requested_points = 72

    global JOINT_ANGLE_TOLERANCE_FK

    rospy.init_node('pedal_simulation', anonymous=True)
    rospy.Subscriber("joint_state", JointState, jointStateCallback)
    ros_right_hip_publisher   = rospy.Publisher('/joint_hip_right/joint_hip_right/target', Float32, queue_size=2)
    ros_right_knee_publisher  = rospy.Publisher('/joint_knee_right/joint_knee_right/target', Float32, queue_size=2)
    ros_right_ankle_publisher = rospy.Publisher('/joint_foot_right/joint_foot_right/target', Float32, queue_size=2)
    ros_left_hip_publisher   = rospy.Publisher('/joint_hip_left/joint_hip_left/target', Float32, queue_size=2)
    ros_left_knee_publisher  = rospy.Publisher('/joint_knee_left/joint_knee_left/target', Float32, queue_size=2)
    ros_left_ankle_publisher = rospy.Publisher('/joint_foot_left/joint_foot_left/target', Float32, queue_size=2)

    capturedPositions = getPedalPositions(num_requested_points)

    endeffector_right = "right_leg"
    frame_right = "foot_right_tip"
    y_offset_right = RIGHT_LEG_OFFSET_Y

    endeffector_left = "left_leg"
    frame_left = "foot_left_tip"
    y_offset_left = LEFT_LEG_OFFSET_Y

    jointAngleDict = {}
    jointAngleDict["num_points"] = num_requested_points

    for pointIter in range(num_requested_points):
        print("Capturing point number ", pointIter)
        thisX = capturedPositions[pointIter][1]
        thisZ = capturedPositions[pointIter][3]
        thisPedalAngle = capturedPositions[pointIter][0]
        jointAngleResult_right = inverse_kinematics_client(endeffector_right, frame_right, thisX, y_offset_right, thisZ)
        jointAngleResult_left = inverse_kinematics_client(endeffector_left, frame_left, thisX, y_offset_left, thisZ)
        if (jointAngleResult_right and jointAngleResult_left):
            jointAngleDict["point_"+str(pointIter)] = {}
            jointAngleDict["point_"+str(pointIter)]["Left"] = {}
            jointAngleDict["point_"+str(pointIter)]["Right"] = {}
            jointAngleDict["point_"+str(pointIter)]["Left"]["Pedal"] = [thisX, thisZ]
            jointAngleDict["point_"+str(pointIter)]["Left"]["Pedal_angle"] = thisPedalAngle
            jointAngleDict["point_"+str(pointIter)]["Left"]["Hip"] = jointAngleResult_left["joint_hip_left"]
            jointAngleDict["point_"+str(pointIter)]["Left"]["Knee"] = jointAngleResult_left["joint_knee_left"]
            jointAngleDict["point_"+str(pointIter)]["Left"]["Ankle"] = jointAngleResult_left["joint_foot_left"]
            jointAngleDict["point_"+str(pointIter)]["Right"]["Pedal"] = [thisX, thisZ]
            jointAngleDict["point_"+str(pointIter)]["Right"]["Pedal_angle"] = thisPedalAngle
            jointAngleDict["point_"+str(pointIter)]["Right"]["Hip"] = jointAngleResult_right["joint_hip_right"]
            jointAngleDict["point_"+str(pointIter)]["Right"]["Knee"] = jointAngleResult_right["joint_knee_right"]
            jointAngleDict["point_"+str(pointIter)]["Right"]["Ankle"] = jointAngleResult_right["joint_foot_right"]
        else:
            jointAngleDict["num_points"] = jointAngleDict["num_points"] - 1

        ros_right_hip_publisher.publish(jointAngleResult_right["joint_hip_right"])
        ros_right_knee_publisher.publish(jointAngleResult_right["joint_knee_right"])
        ros_right_ankle_publisher.publish(jointAngleResult_right["joint_foot_right"])
        ros_left_hip_publisher.publish(jointAngleResult_left["joint_hip_left"])
        ros_left_knee_publisher.publish(jointAngleResult_left["joint_knee_left"])
        ros_left_ankle_publisher.publish(jointAngleResult_left["joint_foot_left"])

        while ( abs(_jointsStatusData[RIGHT_HIP_JOINT]["Pos"] - jointAngleResult_right["joint_hip_right"]) > JOINT_ANGLE_TOLERANCE_FK):
            time.sleep(0.1)
        print("Right hip moved to new position")
        while ( abs(_jointsStatusData[RIGHT_KNEE_JOINT]["Pos"] - jointAngleResult_right["joint_knee_right"]) > JOINT_ANGLE_TOLERANCE_FK ):
            time.sleep(0.1)
        print("Right knee moved to new position")
        while ( abs(_jointsStatusData[RIGHT_ANKLE_JOINT]["Pos"] - jointAngleResult_right["joint_foot_right"]) > JOINT_ANGLE_TOLERANCE_FK ):
            time.sleep(0.1)
        print("Right ankle moved to new position")
        while ( abs(_jointsStatusData[LEFT_HIP_JOINT]["Pos"] - jointAngleResult_left["joint_hip_left"]) > JOINT_ANGLE_TOLERANCE_FK ):
            time.sleep(0.1)
        print("Left hip moved to new position")
        while ( abs(_jointsStatusData[LEFT_KNEE_JOINT]["Pos"] - jointAngleResult_left["joint_knee_left"]) > JOINT_ANGLE_TOLERANCE_FK ):
            time.sleep(0.1)
        print("Left knee moved to new position")
        while ( abs(_jointsStatusData[LEFT_ANKLE_JOINT]["Pos"] - jointAngleResult_left["joint_foot_left"]) > JOINT_ANGLE_TOLERANCE_FK ):
            time.sleep(0.1)
        print("Left ankle moved to new position")

        print("Finished point ", pointIter)

    #print(jointAngleDict)
    with open(JSON_FILENAME, "w") as write_file:
        json.dump(jointAngleDict, write_file, indent=4, sort_keys=True)

    plotEverything(jointAngleDict["num_points"], jointAngleDict)

    return 1

if __name__ == '__main__':
    main()
