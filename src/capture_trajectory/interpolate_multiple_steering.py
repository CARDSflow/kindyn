#!/usr/bin/env python
from __future__ import print_function

import json
import math
import time

import matplotlib.pyplot as plt
import numpy as np
import numpy.polynomial.polynomial as poly
from scipy import interpolate
from scipy.misc import derivative



#############################
###   MODULE PARAMETERS   ###
#############################

RECORDED_TRAJECTORY_FILENAME = "multiple_steering_trajectory_v1_610_points.json"
PRINT_DEBUG = True

MAX_TURNING_ANGLE = math.pi/15
NUM_TEMP_ANGLES   = 31

############################
###   GLOBAL VARIABLES   ###
############################

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

_numTrajectoryPoints       = 0

_trajectorySteering        = []
_trajectoryShoulder0Right  = []
_trajectoryShoulder1Right  = []
_trajectoryShoulder2Right  = []
_trajectoryShoulder0Left   = []
_trajectoryShoulder1Left   = []
_trajectoryShoulder2Left   = []
_trajectoryElbow0Right     = []
_trajectoryElbow1Right     = []
_trajectoryElbow0Left      = []
_trajectoryElbow1Left      = []
_trajectoryWrist0Right     = []
_trajectoryWrist1Right     = []
_trajectoryWrist0Left      = []
_trajectoryWrist1Left      = []

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


##############################
###   UTILITY FUNCTIONS   ###
##############################


def importJointTrajectoryRecord():

    global _trajectorySteering
    global _trajectoryShoulder0Right
    global _trajectoryShoulder1Right
    global _trajectoryShoulder2Right
    global _trajectoryShoulder0Left
    global _trajectoryShoulder1Left
    global _trajectoryShoulder2Left
    global _trajectoryElbow0Right
    global _trajectoryElbow1Right
    global _trajectoryElbow0Left
    global _trajectoryElbow1Left
    global _trajectoryWrist0Right
    global _trajectoryWrist1Right
    global _trajectoryWrist0Left
    global _trajectoryWrist1Left

    global PRINT_DEBUG

    with open(RECORDED_TRAJECTORY_FILENAME, "r") as read_file:
        loaded_data = json.load(read_file)

    if (loaded_data["num_points"] is None) or (loaded_data["num_steering_angles"] is None):
        return 0
    else:
        _numTrajectoryPoints = loaded_data["num_points"]
        _numSteeringAngles   = loaded_data["num_steering_angles"]

#    for steeringAngleIterator in range(_numSteeringAngles):
#        _trajectorySteering.append([])
#        _trajectoryShoulder0Right.append([])
#        _trajectoryShoulder1Right.append([])
#        _trajectoryShoulder2Right.append([])
#        _trajectoryShoulder0Left.append([])
#        _trajectoryShoulder1Left.append([])
#        _trajectoryShoulder2Left.append([])
#        _trajectoryElbow0Right.append([])
#        _trajectoryElbow1Right.append([])
#        _trajectoryElbow0Left.append([])
#        _trajectoryElbow1Left.append([])
#        _trajectoryWrist0Right.append([])
#        _trajectoryWrist1Right.append([])
#        _trajectoryWrist0Left.append([])
#        _trajectoryWrist1Left.append([])

    for pointIterator in range(1, _numTrajectoryPoints + 1):
        if ("point_"+str(pointIterator) in loaded_data):
            _trajectorySteering.append(loaded_data["point_"+str(pointIterator)]["Steering_angle"])
            _trajectoryShoulder0Right.append(loaded_data["point_"+str(pointIterator)]["Right"][JOINT_SHOULDER_AXIS0_RIGHT])
            _trajectoryShoulder1Right.append(loaded_data["point_"+str(pointIterator)]["Right"][JOINT_SHOULDER_AXIS1_RIGHT])
            _trajectoryShoulder2Right.append(loaded_data["point_"+str(pointIterator)]["Right"][JOINT_SHOULDER_AXIS2_RIGHT])
            _trajectoryElbow0Right.append(loaded_data["point_"+str(pointIterator)]["Right"][JOINT_ELBOW_ROT0_RIGHT])
            _trajectoryElbow1Right.append(loaded_data["point_"+str(pointIterator)]["Right"][JOINT_ELBOW_ROT1_RIGHT])
            _trajectoryWrist0Right.append(loaded_data["point_"+str(pointIterator)]["Right"][JOINT_WRIST_0_RIGHT])
            _trajectoryWrist1Right.append(loaded_data["point_"+str(pointIterator)]["Right"][JOINT_WRIST_1_RIGHT])

            _trajectoryShoulder0Left.append(loaded_data["point_"+str(pointIterator)]["Left"][JOINT_SHOULDER_AXIS0_LEFT])
            _trajectoryShoulder1Left.append(loaded_data["point_"+str(pointIterator)]["Left"][JOINT_SHOULDER_AXIS1_LEFT])
            _trajectoryShoulder2Left.append(loaded_data["point_"+str(pointIterator)]["Left"][JOINT_SHOULDER_AXIS2_LEFT])
            _trajectoryElbow0Left.append(loaded_data["point_"+str(pointIterator)]["Left"][JOINT_ELBOW_ROT0_LEFT])
            _trajectoryElbow1Left.append(loaded_data["point_"+str(pointIterator)]["Left"][JOINT_ELBOW_ROT1_LEFT])
            _trajectoryWrist0Left.append(loaded_data["point_"+str(pointIterator)]["Left"][JOINT_WRIST_0_LEFT])
            _trajectoryWrist1Left.append(loaded_data["point_"+str(pointIterator)]["Left"][JOINT_WRIST_1_LEFT])

        else:
            print("WARNING: No point_%s in trajectory" % (pointIterator))
            _numTrajectoryPoints -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(_numTrajectoryPoints)



def regressAllJointPositions():

    global _trajectorySteering
    global _trajectoryShoulder0Right
    global _trajectoryShoulder1Right
    global _trajectoryShoulder2Right
    global _trajectoryShoulder0Left
    global _trajectoryShoulder1Left
    global _trajectoryShoulder2Left
    global _trajectoryElbow0Right
    global _trajectoryElbow1Right
    global _trajectoryElbow0Left
    global _trajectoryElbow1Left
    global _trajectoryWrist0Right
    global _trajectoryWrist1Right
    global _trajectoryWrist0Left
    global _trajectoryWrist1Left

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

    _regressedShoulder0Right = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryShoulder0Right, 4))
    _regressedShoulder1Right = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryShoulder1Right, 4))
    _regressedShoulder2Right = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryShoulder2Right, 4))
    _regressedElbow0Right    = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryElbow0Right, 4))
    _regressedElbow1Right    = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryElbow1Right, 4))
    _regressedWrist0Right    = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryWrist0Right, 4))
    _regressedWrist1Right    = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryWrist1Right, 4))

    _regressedShoulder0Left = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryShoulder0Left, 4))
    _regressedShoulder1Left = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryShoulder1Left, 4))
    _regressedShoulder2Left = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryShoulder2Left, 4))
    _regressedElbow0Left    = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryElbow0Left, 4))
    _regressedElbow1Left    = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryElbow1Left, 4))
    _regressedWrist0Left    = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryWrist0Left, 4))
    _regressedWrist1Left    = poly.Polynomial(poly.polyfit(_trajectorySteering, _trajectoryWrist1Left, 4))

    return 1


def printRegressedFunctions():

    global _trajectorySteering
    global _trajectoryShoulder0Right
    global _trajectoryShoulder1Right
    global _trajectoryShoulder2Right
    global _trajectoryShoulder0Left
    global _trajectoryShoulder1Left
    global _trajectoryShoulder2Left
    global _trajectoryElbow0Right
    global _trajectoryElbow1Right
    global _trajectoryElbow0Left
    global _trajectoryElbow1Left
    global _trajectoryWrist0Right
    global _trajectoryWrist1Right
    global _trajectoryWrist0Left
    global _trajectoryWrist1Left

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

    highDefPlotRange = np.linspace(-1 * math.pi / 12, math.pi / 12, 100)

    plt.figure(1)
    plt.plot(_trajectorySteering, _trajectoryShoulder0Right, '*')
    plt.plot(highDefPlotRange, _regressedShoulder0Right(highDefPlotRange), '-')

    plt.figure(2)
    plt.plot(_trajectorySteering, _trajectoryShoulder1Right, '*')
    plt.plot(highDefPlotRange, _regressedShoulder1Right(highDefPlotRange), '-')

    plt.figure(3)
    plt.plot(_trajectorySteering, _trajectoryShoulder2Right, '*')
    plt.plot(highDefPlotRange, _regressedShoulder2Right(highDefPlotRange), '-')

    plt.figure(4)
    plt.plot(_trajectorySteering, _trajectoryShoulder0Left, '*')
    plt.plot(highDefPlotRange, _regressedShoulder0Left(highDefPlotRange), '-')

    plt.figure(5)
    plt.plot(_trajectorySteering, _trajectoryShoulder1Left, '*')
    plt.plot(highDefPlotRange, _regressedShoulder1Left(highDefPlotRange), '-')

    plt.figure(6)
    plt.plot(_trajectorySteering, _trajectoryShoulder2Left, '*')
    plt.plot(highDefPlotRange, _regressedShoulder2Left(highDefPlotRange), '-')

    plt.figure(7)
    plt.plot(_trajectorySteering, _trajectoryElbow0Right, '*')
    plt.plot(highDefPlotRange, _regressedElbow0Right(highDefPlotRange), '-')

    plt.figure(8)
    plt.plot(_trajectorySteering, _trajectoryElbow1Right, '*')
    plt.plot(highDefPlotRange, _regressedElbow1Right(highDefPlotRange), '-')

    plt.figure(9)
    plt.plot(_trajectorySteering, _trajectoryWrist0Right, '*')
    plt.plot(highDefPlotRange, _regressedWrist0Right(highDefPlotRange), '-')

    plt.figure(10)
    plt.plot(_trajectorySteering, _trajectoryWrist1Right, '*')
    plt.plot(highDefPlotRange, _regressedWrist1Right(highDefPlotRange), '-')

    plt.figure(11)
    plt.plot(_trajectorySteering, _trajectoryElbow0Left, '*')
    plt.plot(highDefPlotRange, _regressedElbow0Left(highDefPlotRange), '-')

    plt.figure(12)
    plt.plot(_trajectorySteering, _trajectoryElbow1Left, '*')
    plt.plot(highDefPlotRange, _regressedElbow1Left(highDefPlotRange), '-')

    plt.figure(13)
    plt.plot(_trajectorySteering, _trajectoryWrist0Left, '*')
    plt.plot(highDefPlotRange, _regressedWrist0Left(highDefPlotRange), '-')

    plt.figure(14)
    plt.plot(_trajectorySteering, _trajectoryWrist1Left, '*')
    plt.plot(highDefPlotRange, _regressedWrist1Left(highDefPlotRange), '-')

    plt.show()


def printAllTrajectories():

    global _trajectorySteering
    global _trajectoryShoulder0Right
    global _trajectoryShoulder1Right
    global _trajectoryShoulder2Right
    global _trajectoryShoulder0Left
    global _trajectoryShoulder1Left
    global _trajectoryShoulder2Left
    global _trajectoryElbow0Right
    global _trajectoryElbow1Right
    global _trajectoryElbow0Left
    global _trajectoryElbow1Left
    global _trajectoryWrist0Right
    global _trajectoryWrist1Right
    global _trajectoryWrist0Left
    global _trajectoryWrist1Left

    plt.figure(1)
    plt.plot(_trajectorySteering, _trajectoryShoulder0Right, '*')
    plt.title("Shoulder0Right")

    plt.figure(2)
    plt.plot(_trajectorySteering, _trajectoryShoulder1Right, '*')
    plt.title("Shoulder1Right")

    plt.figure(3)
    plt.plot(_trajectorySteering, _trajectoryShoulder2Right, '*')
    plt.title("Shoulder2Right")

    plt.figure(4)
    plt.plot(_trajectorySteering, _trajectoryShoulder0Left, '*')
    plt.title("Shoulder0Left")

    plt.figure(5)
    plt.plot(_trajectorySteering, _trajectoryShoulder1Left, '*')
    plt.title("Shoulder1Left")

    plt.figure(6)
    plt.plot(_trajectorySteering, _trajectoryShoulder2Left, '*')
    plt.title("Shoulder2Left")

    plt.figure(7)
    plt.plot(_trajectorySteering, _trajectoryElbow0Right, '*')
    plt.title("Elbow0Right")

    plt.figure(8)
    plt.plot(_trajectorySteering, _trajectoryElbow1Right, '*')
    plt.title("Elbow1Right")

    plt.figure(9)
    plt.plot(_trajectorySteering, _trajectoryWrist0Right, '*')
    plt.title("Wrist0Right")

    plt.figure(10)
    plt.plot(_trajectorySteering, _trajectoryWrist1Right, '*')
    plt.title("Wrist1Right")

    plt.figure(11)
    plt.plot(_trajectorySteering, _trajectoryElbow0Left, '*')
    plt.title("Elbow0Left")

    plt.figure(12)
    plt.plot(_trajectorySteering, _trajectoryElbow1Left, '*')
    plt.title("Elbow1Left")

    plt.figure(13)
    plt.plot(_trajectorySteering, _trajectoryWrist0Left, '*')
    plt.title("Wrist0Left")

    plt.figure(14)
    plt.plot(_trajectorySteering, _trajectoryWrist1Left, '*')
    plt.title("Wrist1Left")

    plt.show()


################
###   MAIN   ###
################


def main():

    importJointTrajectoryRecord()
#    printAllTrajectories()
    regressAllJointPositions()
    printRegressedFunctions()

    return 1


if __name__ == '__main__':
    main()
