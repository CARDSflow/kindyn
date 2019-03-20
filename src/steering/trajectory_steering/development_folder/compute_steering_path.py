## @package steering

import math

import matplotlib.pyplot as plt

# Program tham computes [x, y, z] coordinates of left and right endeffector for steering movements
# Alternatively take steering angle as input, produce [x, y, z] coordinates for endeffectors as output

###############################
###   FUNCTION PARAMETERS   ###
###############################

MAX_TURNING_ANGLE = math.pi / 6  # [rad]
NUM_STEERING_ANGLES = 61  # Should be odd number, symmetric about zero value

RIKSHAW_TURN_JOINT_X_OFFSET = 0  # [m]
RIKSHAW_TURN_JOINT_Y_OFFSET = 0  # [m]
RIKSHAW_TURN_JOINT_Z_OFFSET = 0  # [m]

HANDLEBAR_X_OFFSET = 0.728713  # [m]
HANDLEBAR_Z_OFFSET = 0.719269  # [m]

HAND_Y_OFFSET = 0.125  # [m]

############################
###   GLOBAL VARIABLES   ###
############################

_steeringAngles = []

_rightHandTrajectory = []
_leftHandTrajectory = []
_centerHandlebarTrajectory = []


##############################
###   UTILITY FUNCTIONS   ###
##############################

def computeSteeringAngles():
    global _steeringAngles

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
        thisCenterPointY = RIKSHAW_TURN_JOINT_Y_OFFSET + (
                HANDLEBAR_X_OFFSET * math.sin(_steeringAngles[steeringAngleIterator]))

        thisRightHandPointX = thisCenterPointX + (
                HAND_Y_OFFSET * math.cos(_steeringAngles[steeringAngleIterator] + (math.pi / 2)))
        thisRightHandPointY = thisCenterPointY + (
                HAND_Y_OFFSET * math.sin(_steeringAngles[steeringAngleIterator] + (math.pi / 2)))

        thisLeftHandPointX = thisCenterPointX + (
                HAND_Y_OFFSET * math.cos(_steeringAngles[steeringAngleIterator] - (math.pi / 2)))
        thisLeftHandPointY = thisCenterPointY + (
                HAND_Y_OFFSET * math.sin(_steeringAngles[steeringAngleIterator] - (math.pi / 2)))

        _centerHandlebarTrajectory.append([thisCenterPointX, thisCenterPointY])
        _rightHandTrajectory.append([thisRightHandPointX, thisRightHandPointY])
        _leftHandTrajectory.append([thisLeftHandPointX, thisLeftHandPointY])


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

    return 1


if __name__ == '__main__':
    main()
