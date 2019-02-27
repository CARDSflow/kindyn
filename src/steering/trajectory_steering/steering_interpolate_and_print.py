## @package steering

from __future__ import print_function

import json
import math
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
from scipy.misc import derivative



#############################
###   MODULE PARAMETERS   ###
#############################

RECORDED_TRAJECTORY_FILENAME = "steering_trajectory.json"
PRINT_DEBUG = True

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

_interpolatedShoulder0Right  = None
_interpolatedShoulder1Right  = None
_interpolatedShoulder2Right  = None
_interpolatedShoulder0Left   = None
_interpolatedShoulder1Left   = None
_interpolatedShoulder2Left   = None
_interpolatedElbow0Right     = None
_interpolatedElbow1Right     = None
_interpolatedElbow0Left      = None
_interpolatedElbow1Left      = None
_interpolatedWrist0Right     = None
_interpolatedWrist1Right     = None
_interpolatedWrist0Left      = None
_interpolatedWrist1Left      = None


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

    if loaded_data["num_points"] is None:
        return 0
    else:
        _numTrajectoryPoints = loaded_data["num_points"]


    for pointIterator in range(_numTrajectoryPoints):
        if "point_"+str(pointIterator) in loaded_data:
            _trajectorySteering.append(loaded_data["point_"+str(pointIterator)]["Right"]["Steering_angle"])

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
            print("WARNING: No point_%s in trajectory" % pointIterator)
            _numTrajectoryPoints -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(_numTrajectoryPoints)



def interpolateAllJointPositions():

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

    global _interpolatedShoulder0Right
    global _interpolatedShoulder1Right
    global _interpolatedShoulder2Right
    global _interpolatedShoulder0Left
    global _interpolatedShoulder1Left
    global _interpolatedShoulder2Left
    global _interpolatedElbow0Right
    global _interpolatedElbow1Right
    global _interpolatedElbow0Left
    global _interpolatedElbow1Left
    global _interpolatedWrist0Right
    global _interpolatedWrist1Right
    global _interpolatedWrist0Left
    global _interpolatedWrist1Left

    _interpolatedShoulder0Right = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder0Right, kind = "cubic")
    _interpolatedShoulder1Right = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder1Right, kind = "cubic")
    _interpolatedShoulder2Right = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder2Right, kind = "cubic")
    _interpolatedElbow0Right    = interpolate.interp1d(_trajectorySteering, _trajectoryElbow0Right, kind = "cubic")
    _interpolatedElbow1Right    = interpolate.interp1d(_trajectorySteering, _trajectoryElbow1Right, kind = "cubic")
    _interpolatedWrist0Right    = interpolate.interp1d(_trajectorySteering, _trajectoryWrist0Right, kind = "cubic")
    _interpolatedWrist1Right    = interpolate.interp1d(_trajectorySteering, _trajectoryWrist1Right, kind = "cubic")

    _interpolatedShoulder0Left = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder0Left, kind = "cubic")
    _interpolatedShoulder1Left = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder1Left, kind = "cubic")
    _interpolatedShoulder2Left = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder2Left, kind = "cubic")
    _interpolatedElbow0Left    = interpolate.interp1d(_trajectorySteering, _trajectoryElbow0Left, kind = "cubic")
    _interpolatedElbow1Left    = interpolate.interp1d(_trajectorySteering, _trajectoryElbow1Left, kind = "cubic")
    _interpolatedWrist0Left    = interpolate.interp1d(_trajectorySteering, _trajectoryWrist0Left, kind = "cubic")
    _interpolatedWrist1Left    = interpolate.interp1d(_trajectorySteering, _trajectoryWrist1Left, kind = "cubic")


    return 1


def printInterpolatedFunctions():

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

    global _interpolatedShoulder0Right
    global _interpolatedShoulder1Right
    global _interpolatedShoulder2Right
    global _interpolatedShoulder0Left
    global _interpolatedShoulder1Left
    global _interpolatedShoulder2Left
    global _interpolatedElbow0Right
    global _interpolatedElbow1Right
    global _interpolatedElbow0Left
    global _interpolatedElbow1Left
    global _interpolatedWrist0Right
    global _interpolatedWrist1Right
    global _interpolatedWrist0Left
    global _interpolatedWrist1Left

    highDefPlotRange = np.linspace(_trajectorySteering[0], _trajectorySteering[len(_trajectorySteering)-1], 500)

    plt.figure(1)
    plt.plot(_trajectorySteering, _trajectoryShoulder0Right, '*')
    plt.plot(highDefPlotRange, _interpolatedShoulder0Right(highDefPlotRange), '-')

    plt.figure(2)
    plt.plot(_trajectorySteering, _trajectoryShoulder1Right, '*')
    plt.plot(highDefPlotRange, _interpolatedShoulder1Right(highDefPlotRange), '-')

    plt.figure(3)
    plt.plot(_trajectorySteering, _trajectoryShoulder2Right, '*')
    plt.plot(highDefPlotRange, _interpolatedShoulder2Right(highDefPlotRange), '-')

    plt.figure(4)
    plt.plot(_trajectorySteering, _trajectoryShoulder0Left, '*')
    plt.plot(highDefPlotRange, _interpolatedShoulder0Left(highDefPlotRange), '-')

    plt.figure(5)
    plt.plot(_trajectorySteering, _trajectoryShoulder1Left, '*')
    plt.plot(highDefPlotRange, _interpolatedShoulder1Left(highDefPlotRange), '-')

    plt.figure(6)
    plt.plot(_trajectorySteering, _trajectoryShoulder2Left, '*')
    plt.plot(highDefPlotRange, _interpolatedShoulder2Left(highDefPlotRange), '-')

    plt.figure(7)
    plt.plot(_trajectorySteering, _trajectoryElbow0Right, '*')
    plt.plot(highDefPlotRange, _interpolatedElbow0Right(highDefPlotRange), '-')

    plt.figure(8)
    plt.plot(_trajectorySteering, _trajectoryElbow1Right, '*')
    plt.plot(highDefPlotRange, _interpolatedElbow1Right(highDefPlotRange), '-')

    plt.figure(9)
    plt.plot(_trajectorySteering, _trajectoryWrist0Right, '*')
    plt.plot(highDefPlotRange, _interpolatedWrist0Right(highDefPlotRange), '-')

    plt.figure(10)
    plt.plot(_trajectorySteering, _trajectoryWrist1Right, '*')
    plt.plot(highDefPlotRange, _interpolatedWrist1Right(highDefPlotRange), '-')

    plt.figure(11)
    plt.plot(_trajectorySteering, _trajectoryElbow0Left, '*')
    plt.plot(highDefPlotRange, _interpolatedElbow0Left(highDefPlotRange), '-')

    plt.figure(12)
    plt.plot(_trajectorySteering, _trajectoryElbow1Left, '*')
    plt.plot(highDefPlotRange, _interpolatedElbow1Left(highDefPlotRange), '-')

    plt.figure(13)
    plt.plot(_trajectorySteering, _trajectoryWrist0Left, '*')
    plt.plot(highDefPlotRange, _interpolatedWrist0Left(highDefPlotRange), '-')

    plt.figure(14)
    plt.plot(_trajectorySteering, _trajectoryWrist1Left, '*')
    plt.plot(highDefPlotRange, _interpolatedWrist1Left(highDefPlotRange), '-')

    plt.show()


################
###   MAIN   ###
################


def main():

    importJointTrajectoryRecord()
    interpolateAllJointPositions()
    printInterpolatedFunctions()

    return 1


if __name__ == '__main__':
    main()
