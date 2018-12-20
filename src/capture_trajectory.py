#! /usr/bin/env python
import math
import sys
import json
import matplotlib.pyplot as plt

import rospy
from roboy_middleware_msgs.srv import InverseKinematics
from geometry_msgs.msg import Pose, Point, Quaternion

JSON_FILENAME = "captured_trajectory_2_ik.json"

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
        thisRightPedalAngle = sampleIterator*angularStep
        thisRightXVal = PEDAL_CENTER_OFFSET_X + math.cos(thisRightPedalAngle)*PEDAL_RADIUS
        thisRightYVal = PEDAL_CENTER_OFFSET_Y
        thisRightZVal = PEDAL_CENTER_OFFSET_Z + math.sin(thisRightPedalAngle)*PEDAL_RADIUS
	thisLeftPedalAngle = thisRightPedalAngle + 3.14159265359
        thisLeftXVal = PEDAL_CENTER_OFFSET_X + math.cos(thisLeftPedalAngle)*PEDAL_RADIUS
        thisLeftYVal = PEDAL_CENTER_OFFSET_Y
        thisLeftZVal = PEDAL_CENTER_OFFSET_Z + math.sin(thisLeftPedalAngle)*PEDAL_RADIUS
        capturedPositions.append([thisRightPedalAngle, thisRightXVal, thisRightZVal, thisLeftPedalAngle, thisLeftXVal, thisLeftZVal])

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
            if "Pedal" in jointAngleDict["point_"+str(pointIter)]:
                plt.plot(jointAngleDict["point_"+str(pointIter)]["Pedal"][0], jointAngleDict["point_"+str(pointIter)]["Pedal"][1], 'rs',label="IK recorded")

    
    plt.figure(2)
    plt.title('Hip positions')
    x_values = []
    z_values = []
    for pointIter in range(jointAngleDict["num_points"]):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Hip" in jointAngleDict["point_"+str(pointIter)]:
                x_values.append(pointIter)
                z_values.append(jointAngleDict["point_"+str(pointIter)]["Hip"])
    plt.plot(x_values, z_values)

    plt.figure(3)
    plt.title('Knee positions')
    x_values = []
    z_values = []
    for pointIter in range(jointAngleDict["num_points"]):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Knee" in jointAngleDict["point_"+str(pointIter)]:
                x_values.append(pointIter)
                z_values.append(jointAngleDict["point_"+str(pointIter)]["Knee"])
    plt.plot(x_values, z_values)

    plt.figure(4)
    plt.title('Ankle positions')
    x_values = []
    z_values = []
    for pointIter in range(jointAngleDict["num_points"]):
        if "point_"+str(pointIter) in jointAngleDict:
            if "Ankle" in jointAngleDict["point_"+str(pointIter)]:
                x_values.append(pointIter)
                z_values.append(jointAngleDict["point_"+str(pointIter)]["Ankle"])
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
        num_points = int(sys.argv[1])
    else:
        num_points = 36

    #plotPedalTrajectories()

    capturedPositions = getPedalPositions(num_points)
    #for thisLine in capturedPositions:
    #    print(thisLine)

    right_endeffector = "pedal_right"
    right_frame = "foot_right"
    right_y_offset = RIGHT_LEG_OFFSET_Y

    left_endeffector = "pedal_left"
    left_frame = "foot_left"
    left_y_offset = LEFT_LEG_OFFSET_Y

    jointAngleDict = {}
    

    for pointIter in range(num_points):
        thisRightX = capturedPositions[pointIter][1]
        thisRightZ = capturedPositions[pointIter][2]
        thisLeftX = capturedPositions[pointIter][4]
        thisLeftZ = capturedPositions[pointIter][5]
        rightJointAngleResult = inverse_kinematics_client(right_endeffector, right_frame, thisRightX, right_y_offset, thisRightZ)
	leftJointAngleResult = inverse_kinematics_client(left_endeffector, left_frame, thisLeftX, left_y_offset, thisLeftZ)
        if (rightJointAngleResult and leftJointAngleResult):
            jointAngleDict["point_"+str(pointIter)] = {}
            jointAngleDict["point_"+str(pointIter)]["Right"] = {}
            jointAngleDict["point_"+str(pointIter)]["Right"]["Hip"] = rightJointAngleResult["joint_hip_right"]
            jointAngleDict["point_"+str(pointIter)]["Right"]["Knee"] = rightJointAngleResult["joint_knee_right"]
            jointAngleDict["point_"+str(pointIter)]["Right"]["Ankle"] = rightJointAngleResult["joint_foot_right"]
            jointAngleDict["point_"+str(pointIter)]["Left"] = {}
            jointAngleDict["point_"+str(pointIter)]["Left"]["Hip"] = leftJointAngleResult["joint_hip_left"]
            jointAngleDict["point_"+str(pointIter)]["Left"]["Knee"] = leftJointAngleResult["joint_knee_left"]
            jointAngleDict["point_"+str(pointIter)]["Left"]["Ankle"] = leftJointAngleResult["joint_foot_left"]
            jointAngleDict["point_"+str(pointIter)]["Right"]["Pedal"] = [thisRightX, thisRightZ]
            jointAngleDict["point_"+str(pointIter)]["Left"]["Pedal"] = [thisLeftX, thisLeftZ]
	else:
	    num_points -= 1

    jointAngleDict["num_points"] = num_points

    #print(jointAngleDict)
    with open(JSON_FILENAME, "w") as write_file:
        json.dump(jointAngleDict, write_file, indent=4, sort_keys=True)

    #plotEverything(num_points, jointAngleDict)

    return 1

if __name__ == '__main__':
    main()



