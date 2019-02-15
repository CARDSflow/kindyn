import json
import math
import time

# TODO: define coordinate system

#############################
###   MODULE PARAMETERS   ###
#############################

CONTROL_PARAMETERS_FILENAME = "control_parameters_ddr.json"
RECORDED_TRAJECTORY_FILENAME = "recorded_trajectory_ddr.json"

PEDAL_POSITION_ERROR_TOLERANCE = 10  # [millimeters]
PEDAL_SINGLE_ROTATION_DURATION = 10  # [seconds]
TRAJECTORY_POINT_DURATION      = 0
CONTROLLER_FREQUENCY           = 20  # [Hz]

############################
###   GLOBAL VARIABLES   ###
############################

TESTING_VELOCITY1 = 0.0

RIGHT_HIP_JOINT   = "right_hip"
RIGHT_KNEE_JOINT  = "right_knee"
RIGHT_ANKLE_JOINT = "right_ankle"

LEFT_HIP_JOINT    = "left_hip"
LEFT_KNEE_JOINT   = "left_knee"
LEFT_ANKLE_JOINT  = "left_ankle"

_jointsList = [RIGHT_HIP_JOINT, RIGHT_KNEE_JOINT, RIGHT_ANKLE_JOINT, LEFT_HIP_JOINT, LEFT_KNEE_JOINT, LEFT_ANKLE_JOINT]

_parametersRightHip = {
    "param_p":            1.0,
    "param_i"             0.0,
    "param_d"             0.0,
    "prev_pos"            0.0,
    "prev_vel"            0.0,
    "prev_time"           0.0,
    "prev_error"          0.0,
    "pos_error_integral"  0.0,
    "trajectory_startpoint"     0.0
}

_parametersRightKnee = {
    "param_p":            1.0,
    "param_i"             0.0,
    "param_d"             0.0,
    "prev_pos"            0.0,
    "prev_vel"            0.0,
    "prev_time"           0.0,
    "prev_error"          0.0,
    "pos_error_integral"  0.0,
    "trajectory_startpoint"     0.0
}

_parametersRightAnkle = {
    "param_p":            1.0,
    "param_i"             0.0,
    "param_d"             0.0,
    "prev_pos"            0.0,
    "prev_vel"            0.0,
    "prev_time"           0.0,
    "prev_error"          0.0,
    "pos_error_integral"  0.0,
    "trajectory_startpoint"     0.0
}

_parametersLeftHip = {
    "param_p":            1.0,
    "param_i"             0.0,
    "param_d"             0.0,
    "prev_pos"            0.0,
    "prev_vel"            0.0,
    "prev_time"           0.0,
    "prev_error"          0.0,
    "pos_error_integral"  0.0,
    "trajectory_startpoint"     0.0
}

_parametersLeftKnee = {
    "param_p":            1.0,
    "param_i"             0.0,
    "param_d"             0.0,
    "prev_pos"            0.0,
    "prev_vel"            0.0,
    "prev_time"           0.0,
    "prev_error"          0.0,
    "pos_error_integral"  0.0,
    "trajectory_startpoint"     0.0
}

_parametersLeftAnkle = {
    "param_p":            1.0,
    "param_i"             0.0,
    "param_d"             0.0,
    "prev_pos"            0.0,
    "prev_vel"            0.0,
    "prev_time"           0.0,
    "prev_error"          0.0,
    "pos_error_integral"  0.0,
    "trajectory_startpoint"     0.0
}

_jointsControlData = {
    RIGHT_HIP_JOINT:    _parametersRightHip,
    RIGHT_KNEE_JOINT:   _parametersRightKnee,
    RIGHT_ANKLE_JOINT:  _parametersRightAnkle,
    LEFT_HIP_JOINT:     _parametersLeftHip,
    LEFT_KNEE_JOINT:    _parametersLeftKnee,
    LEFT_ANKLE_JOINT:   _parametersLeftAnkle
}

_numTrajectoryPoints = 0
_trajectoryStartingPoint = 0

_pedalTrajectory = []
_hipTrajectory   = []
_kneeTrajectory  = []
_ankleTrajectory = []

##############################
###   UTILITY FUNCTIONS   ###
##############################

def importJointTrajectoryRecord():

    with open(RECORDED_TRAJECTORY_FILENAME, "r") as read_file:
        loaded_data = json.load(read_file)

    if loaded_data["num_points"] == None:
        return 0
    else _numTrajectoryPoints = loaded_data["num_points"]

    # Deleting previous trajectory before loading new
    _pedalTrajectory = None
    _hipTrajectory   = None
    _kneeTrajectory  = None
    _ankleTrajectory = None
    for pointIterator in range (_numTrajectoryPoints):
        _pedalTrajectory.append(loaded_data["point_"+str(pointIterator)]["Pedal"])
        _hipTrajectory.append(loaded_data["point_"+str(pointIterator)]["Hip"])
        _kneeTrajectory.append(loaded_data["point_"+str(pointIterator)]["Knee"])
        _ankleTrajectory.append(loaded_data["point_"+str(pointIterator)]["Ankle"])

    return 1

def importJointPIDParameters():

    with open(CONTROL_PARAMETERS_FILENAME, "r") as read_file:
        loaded_data = json.load(read_file)

    return 0


def setJointVelocity(jointName, jointVel):

    if jointName == RIGHT_HIP_JOINT:
        TESTING_VELOCITY1 = jointVel

    return 1

def getJointVelocity(jointName):

    if jointName == RIGHT_HIP_JOINT:
        return TESTING_VELOCITY1

    return 1

def getJointPosition(jointName):

    return 1

def getPedalPosition():

    return [0.0, 0.0] #[x, y]

def getDistance(point1,point2):

    x_diff = point2[0] - point1[0]
    y_diff = point2[1] - point1[1]

    return math.sqrt(x_diff*x_diff + y_diff*y_diff)

def setPedalSingleRotationDuration(new_duration_seconds):
    PEDAL_SINGLE_ROTATION_DURATION = new_duration_seconds
    setTrajectoryPointDuration()
    return 1

def setTrajectoryPointDuration():
    if _numTrajectoryPoints != 0:
        TRAJECTORY_POINT_DURATION = double(PEDAL_SINGLE_ROTATION_DURATION) / _numTrajectoryPoints
    else:
        TRAJECTORY_POINT_DURATION = 0
        return 0
    return 1

def interpolateTrajectoryPoints(jointName, value1, value2, startTime, currTime, endTime):
    if currTime > endTime:
        return value2
    return value1 + (value2 - value1)*(double(currTime - startTime)/(endTime - startTime))


#########################
###   STATE MACHINE   ###
#########################

INIT              = "INIT"
PEDAL             = "PEDAL"
UPDATE_PARAMETERS = "UPDATE_PARAMETERS"

def FSM():

    _runFSM = 1

    _currState = INIT
    _currTrajectoryPoint = None

    _startTime = None
    _endTime = None
    _currTime = None
    _prevTime = None

    while _runFSM:

        ##############################################
        if _currState == INIT:
        ##############################################

            importJointTrajectoryRecord()
            importJointPIDParameters()  # Just a placeholder function for now!
            setTrajectoryPointDuration()

            # Find starting point on the trajectory

            _currState = PEDAL

        ##############################################
        if _currState == PEDAL:
        ##############################################

            # Initialize state
            if _currTrajectoryPoint == None:
                _currTrajectoryPoint = _trajectoryStartingPoint
            if _startTime == None:
                _startTime = time.time()
            if _endTime == None:
                _endTime = _startTime + TRAJECTORY_POINT_DURATION
            if _prevTime == None:
                _prevTime = time.time()

            # Regulate update frequency
            currTime = time.time()
            while (currTime - prevTime) < (1 / CONTROLLER_FREQUENCY):
                sleep(1)
                prevTime = currTime
                currTime = time.time()

            # Check if trajectory point reached and act accordingly
            if getDistance(getPedalPosition, _pedalTrajectory[_currTrajectoryPoint]) <= PEDAL_POSITION_ERROR_TOLERANCE:
                if _currTrajectoryPoint < _numTrajectoryPoints:
                    _currTrajectoryPoint += 1
                else:
                    _currTrajectoryPoint = 0
                for thisJointName in _jointsList:
                    _jointsControlData[thisJointName]["trajectory_startpoint"] = getJointPosition(thisJointName)
                _startTime = time.time()
                _endTime = _startTime + TRAJECTORY_POINT_DURATION

            # Iterate through joints and update setpoints
            for thisJointName in _jointsList:
                thisJointPos = getJointPosition(thisJointName)

                thisJointPositionGoalpoint = None
                if (thisJointName == RIGHT_HIP_JOINT) || (thisJointName == LEFT_HIP_JOINT):
                    thisJointPositionGoalpoint = _hipTrajectory[_currTrajectoryPoint]
                else if ((thisJointName == RIGHT_KNEE_JOINT) || (thisJointName == LEFT_KNEE_JOINT)):
                    thisJointPositionGoalpoint = _kneeTrajectory[_currTrajectoryPoint]
                else if (thisJointName == RIGHT_ANKLE_JOINT) || (thisJointName == LEFT_ANKLE_JOINT):
                    thisJointPositionGoalpoint = _ankleTrajectory[_currTrajectoryPoint]

                currTime = time.time()
                thisJointPositionSetpoint = interpolateTrajectoryPoints(thisJointName, _jointsControlData[thisJointName][trajectory_startpoint], thisJointPositionGoalpoint, startTime, currTime, endTime)
                thisJointVelocitySetpoint = computeVelocitySetpoint(thisJointName, thisJointPos, currTime)

                setJointVelocity(thisJointName, thisJointVelocitySetpoint)

        ##############################################
        if _currState == UPDATE_PARAMETERS:
        ##############################################

            # Reload trajectory and PID parameters

    return 1


#############################
###   CONTROL FUNCTIONS   ###
#############################

# PID Controller
def computeVelocitySetpoint(jointName, currPos, currTime, goalPos):

    thisPosError = currPos - goalPos
    thisPosErrorDerivative = thisPosError - prev_error

    _jointsControlData[jointName]["error_integral"] += thisPosError

    _jointsControlData[jointName]["prev_time"] = currTime
    _jointsControlData[jointName]["prev_pos"] = currPos
    _jointsControlData[jointName]["prev_error"] = thisPosError

    return _jointsControlData[jointName]["param_p"]*thisPosError + _jointsControlData[jointName]["param_i"]*_jointsControlData[jointName]["error_integral"] + _jointsControlData[jointName]["param_d"]*thisPosErrorDerivative


################
###   MAIN   ###
################

def main():

    return 1


if __name__ == '__main__':
    main()
