#! /usr/bin/env python
import math
import sys
import json
import matplotlib.pyplot as plt

import rospy
from roboy_middleware_msgs.srv import InverseKinematics
from geometry_msgs.msg import Pose, Point, Quaternion

JSON_FILENAME = "captured_trajectory_ik.json"

###############################
###   MEASURED PARAMETERS   ###
###############################

PEDAL_CENTER_OFFSET_X = 0.20421
PEDAL_CENTER_OFFSET_Y = -0.00062
PEDAL_CENTER_OFFSET_Z = 0.2101

PEDAL_RADIUS = 0.16924  # [millimeters]

RIGHT_LEG_OFFSET_Y = 0.2095
LEFT_LEG_OFFSET_Y = -0.13815

##############################
###   UTILITY FUNCTIONS   ###
##############################

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

def plotIdealAndActualPedalTrajectories(numSamples, ):
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
        num_points = int(sys.argv[1])
    else:
        num_points = 36

    #plotPedalTrajectories()

    capturedPositions = getPedalPositions(num_points)
    #for thisLine in capturedPositions:
    #    print(thisLine)

    endeffector = "pedal_right"
    frame = "foot_right"
    y_offset = 0.2095

    jointAngleDict = {}
    jointAngleDict["num_points"] = num_points

    for pointIter in range(num_points):
        thisX = capturedPositions[pointIter][1]
        thisZ = capturedPositions[pointIter][3]
        jointAngleResult = inverse_kinematics_client(endeffector, frame, thisX, y_offset, thisZ)
        if (jointAngleResult and ("joint_hip_right" in jointAngleResult) and ("joint_knee_right" in jointAngleResult) and ("joint_foot_right" in jointAngleResult)):
		jointAngleDict["point_"+str(pointIter)] = {}
		jointAngleDict["point_"+str(pointIter)]["Pedal"] = [thisX, thisZ]
		jointAngleDict["point_"+str(pointIter)]["Hip"] = jointAngleResult["joint_hip_right"]
		jointAngleDict["point_"+str(pointIter)]["Knee"] = jointAngleResult["joint_knee_right"]
		jointAngleDict["point_"+str(pointIter)]["Ankle"] = jointAngleResult["joint_foot_right"]

    print(jointAngleDict)
    with open(JSON_FILENAME, "w") as write_file:
        json.dump(jointAngleDict, write_file, indent=4, sort_keys=True)

    return 1

if __name__ == '__main__':
    main()
