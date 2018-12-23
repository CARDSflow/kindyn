 #!/usr/bin/env python
import json
import math
import time

import rospy
from roboy_middleware_msgs.srv import InverseKinematics
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32


#############################
###   MODULE PARAMETERS   ###
#############################

RECORDED_TRAJECTORY_FILENAME = "captured_trajectory_ik.json"

PEDAL_POSITION_ERROR_TOLERANCE = 10  # [millimeters]
PEDAL_SINGLE_ROTATION_DURATION = 10  # [seconds]
TRAJECTORY_POINT_DURATION      = 1
CONTROLLER_FREQUENCY           = 20  # [Hz]

############################
###   GLOBAL VARIABLES   ###
############################

RIGHT_HIP_JOINT   = "right_hip"
RIGHT_KNEE_JOINT  = "right_knee"
RIGHT_ANKLE_JOINT = "right_ankle"

LEFT_HIP_JOINT    = "left_hip"
LEFT_KNEE_JOINT   = "left_knee"
LEFT_ANKLE_JOINT  = "left_ankle"

_jointsList = [RIGHT_HIP_JOINT, RIGHT_KNEE_JOINT, RIGHT_ANKLE_JOINT, LEFT_HIP_JOINT, LEFT_KNEE_JOINT, LEFT_ANKLE_JOINT]


_parametersRightHip = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersRightKnee = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersRightAnkle = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersLeftHip = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersLeftKnee = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_parametersLeftAnkle = {
    "param_p":               1.0,
    "param_i":               0.0,
    "param_d":               0.0,
    "prev_pos":              0.0,
    "prev_vel":              0.0,
    "prev_time":             0.0,
    "prev_error":            0.0,
    "pos_error_integral":    0.0,
    "trajectory_startpoint": 0.0
}

_jointsControlData = {
    RIGHT_HIP_JOINT:    _parametersRightHip,
    RIGHT_KNEE_JOINT:   _parametersRightKnee,
    RIGHT_ANKLE_JOINT:  _parametersRightAnkle,
    LEFT_HIP_JOINT:     _parametersLeftHip,
    LEFT_KNEE_JOINT:    _parametersLeftKnee,
    LEFT_ANKLE_JOINT:   _parametersLeftAnkle
}

numTrajectoryPoints = 0
trajectoryStartingPoint = 0

pedalTrajectory = []
hipTrajectory   = []
kneeTrajectory  = []
ankleTrajectory = []

##############################
###   UTILITY FUNCTIONS   ###
##############################

def importJointTrajectoryRecord():

    with open(RECORDED_TRAJECTORY_FILENAME, "r") as read_file:
        loaded_data = json.load(read_file)

    if loaded_data["num_points"] == None:
        return 0
    else:
        numTrajectoryPoints = loaded_data["num_points"]

    # Deleting previous trajectory before loading new
    del pedalTrajectory[:]
    del hipTrajectory[:]
    del kneeTrajectory[:]
    del ankleTrajectory[:]
    for pointIterator in range (numTrajectoryPoints):
	if ("point_"+str(pointIterator) in loaded_data):
		pedalTrajectory.append(loaded_data["point_"+str(pointIterator)]["Pedal"])
		hipTrajectory.append(loaded_data["point_"+str(pointIterator)]["Hip"])
		kneeTrajectory.append(loaded_data["point_"+str(pointIterator)]["Knee"])
		ankleTrajectory.append(loaded_data["point_"+str(pointIterator)]["Ankle"])
	else:
		numTrajectoryPoints -= 1

    print("--------- Num trajectory points:")
    print(numTrajectoryPoints)
    print("--------- Hip trajectory:")
    print(hipTrajectory)
    print("--------- Knee trajectory:")
    print(kneeTrajectory)
    print("--------- Ankle trajectory:")
    print(ankleTrajectory)

    return numTrajectoryPoints

def getJointPosition(jointName):

    return 1

def setJointPosition(jointName):

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
    #if numTrajectoryPoints != 0:
    #    TRAJECTORY_POINT_DURATION = float(PEDAL_SINGLE_ROTATION_DURATION) / numTrajectoryPoints
    #else:
    #    TRAJECTORY_POINT_DURATION = 1
    #    return 0
    TRAJECTORY_POINT_DURATION = 3
    return 1

def interpolateTrajectoryPoints(jointName, value1, value2, startTime, currTime, endTime):
    if currTime > endTime:
        return value2
    return value1 + (value2 - value1)*(float(currTime - startTime)/(endTime - startTime))


#########################
###   STATE MACHINE   ###
#########################

INIT              = "INIT"
PEDAL             = "PEDAL"
UPDATE_PARAMETERS = "UPDATE_PARAMETERS"

def FSM():

    _runFSM = 1

    _currState = INIT
    _currTrajectoryPoint = 0

    _startTime = 0.0
    _endTime = 0.0
    _currTime = 0.0
    _prevTime = 0.0

    ros_right_hip_publisher = rospy.Publisher('/joint_hip_right/joint_hip_right/target', Float32, queue_size=2)
    ros_right_knee_publisher = rospy.Publisher('/joint_knee_right/joint_knee_right/target', Float32, queue_size=2)
    ros_right_ankle_publisher = rospy.Publisher('/joint_foot_right/joint_foot_right/target', Float32, queue_size=2)
    rospy.init_node('pedal_simulation_ddr', anonymous=True)

    while _runFSM:

        ##############################################
        if _currState == INIT:
        ##############################################

            numTrajectoryPoints = importJointTrajectoryRecord()
            setTrajectoryPointDuration()

            # Find starting point on the trajectory

            _currState = PEDAL

        ##############################################
        if _currState == PEDAL:
        ##############################################

            # Initialize state
            if _currTrajectoryPoint == 0:
                _currTrajectoryPoint = trajectoryStartingPoint
            if _startTime == 0.0:
                _startTime = time.time()
            if _endTime == 0.0:
                _endTime = _startTime + TRAJECTORY_POINT_DURATION
            if _prevTime == 0.0:
                _prevTime = time.time()

            # Regulate update frequency
            _currTime = time.time()
            while float(float(_currTime) - float(_prevTime)) < (1 / CONTROLLER_FREQUENCY):
                time.sleep(1)
                _currTime = time.time()
            _prevTime = _currTime

            # Check if trajectory point reached and act accordingly
            #if getDistance(getPedalPosition, pedalTrajectory[_currTrajectoryPoint]) <= PEDAL_POSITION_ERROR_TOLERANCE:
            if float(float(_currTime) - float(_startTime)) > TRAJECTORY_POINT_DURATION:
                print("CURR TRAJECTORY POINT: %s. NUM TRAJECTORY POINTS (-1): %s " % (_currTrajectoryPoint, (numTrajectoryPoints-1)))
                for thisJointName in _jointsList:
                    _jointsControlData[thisJointName]["trajectory_startpoint"] = hipTrajectory[_currTrajectoryPoint]
                if (_currTrajectoryPoint < (numTrajectoryPoints-1)):
                    _currTrajectoryPoint += 1
                elif (_currTrajectoryPoint >= (numTrajectoryPoints-1)):
                    _currTrajectoryPoint = 0
                print("UPDATING TRAJECTORY POINT. NEW POINT: %s" % (_currTrajectoryPoint))
                _startTime = time.time()
                _endTime = _startTime + TRAJECTORY_POINT_DURATION

            # Iterate through joints and update setpoints
            for thisJointName in _jointsList:

                thisJointPositionGoalpoint = 0.0
                if ((thisJointName == RIGHT_HIP_JOINT) or (thisJointName == LEFT_HIP_JOINT)):
                    thisJointPositionGoalpoint = hipTrajectory[_currTrajectoryPoint]
                elif ((thisJointName == RIGHT_KNEE_JOINT) or (thisJointName == LEFT_KNEE_JOINT)):
                    thisJointPositionGoalpoint = kneeTrajectory[_currTrajectoryPoint]
                elif ((thisJointName == RIGHT_ANKLE_JOINT) or (thisJointName == LEFT_ANKLE_JOINT)):
                    thisJointPositionGoalpoint = ankleTrajectory[_currTrajectoryPoint]

                #_currTime = time.time()
                thisJointPositionSetpoint = thisJointPositionGoalpoint  #interpolateTrajectoryPoints(thisJointName, _jointsControlData[thisJointName]["trajectory_startpoint"], thisJointPositionGoalpoint, _startTime, _currTime, _endTime)

                if thisJointName == RIGHT_HIP_JOINT:
                    ros_right_hip_publisher.publish(thisJointPositionSetpoint)
                elif thisJointName == RIGHT_KNEE_JOINT:
                    ros_right_knee_publisher.publish(thisJointPositionSetpoint)
                elif thisJointName == RIGHT_ANKLE_JOINT:
                    ros_right_ankle_publisher.publish(thisJointPositionSetpoint)

        ##############################################
        #if _currState == UPDATE_PARAMETERS:
        ##############################################

            # Reload trajectory and PID parameters

    return 1

################
###   MAIN   ###
################

def main():

    FSM()

    return 1


if __name__ == '__main__':
    main()

