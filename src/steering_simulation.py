#!/usr/bin/env python
from __future__ import print_function

# roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='joint_hip_left joint_hip_right joint_wheel_right joint_wheel_back joint_pedal spine_joint joint_wheel_left joint_front joint_pedal_right joint_pedal_left elbow_right_rot1 joint_foot_left joint_knee_right joint_knee_left joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left_rot1 elbow_left_rot0 left_wrist_0 left_wrist_1 right_shoulder_axis0 right_shoulder_axis2 right_shoulder_axis1 elbow_right_rot0 right_wrist_0 right_wrist_1 head_axis0 head_axis1 head_axis2'


import json
import time
from threading import Thread

from scipy import interpolate
import numpy as np
import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from std_msgs.msg import Float32, String

#############################
###   MODULE PARAMETERS   ###
#############################

PRINT_DEBUG = False

RECORDED_TRAJECTORY_FILENAME = "capture_trajectory/old_captured_trajectory.json"

############################
###   GLOBAL VARIABLES   ###
############################

UPDATE_FREQUENCY = 0.001
MAX_ANGLE_CHANGE =  np.pi/60

JOINT_SHOULDER_AXIS0_RIGHT = "right_shoulder_axis0"
JOINT_SHOULDER_AXIS1_RIGHT = "right_shoulder_axis1"
JOINT_SHOULDER_AXIS2_RIGHT = "right_shoulder_axis2"
JOINT_SHOULDER_AXIS0_LEFT = "left_shoulder_axis0"
JOINT_SHOULDER_AXIS1_LEFT = "left_shoulder_axis1"
JOINT_SHOULDER_AXIS2_LEFT = "left_shoulder_axis2"
JOINT_ELBOW_ROT0_RIGHT = "elbow_right_rot0"
JOINT_ELBOW_ROT1_RIGHT = "elbow_right_rot1"
JOINT_ELBOW_ROT0_LEFT = "elbow_left_rot0"
JOINT_ELBOW_ROT1_LEFT = "elbow_left_rot1"
JOINT_WRIST_0_RIGHT = "right_wrist_0"
JOINT_WRIST_1_RIGHT = "right_wrist_1"
JOINT_WRIST_0_LEFT = "left_wrist_0"
JOINT_WRIST_1_LEFT = "left_wrist_1"

_joints_list = [JOINT_SHOULDER_AXIS0_RIGHT, JOINT_SHOULDER_AXIS1_RIGHT, JOINT_SHOULDER_AXIS2_RIGHT,
                JOINT_SHOULDER_AXIS0_LEFT, JOINT_SHOULDER_AXIS1_LEFT, JOINT_SHOULDER_AXIS2_LEFT,
                JOINT_ELBOW_ROT0_RIGHT, JOINT_ELBOW_ROT1_RIGHT, JOINT_ELBOW_ROT0_LEFT, JOINT_ELBOW_ROT1_LEFT,
                JOINT_WRIST_0_RIGHT, JOINT_WRIST_1_RIGHT, JOINT_WRIST_0_LEFT, JOINT_WRIST_1_LEFT]


_numTrajectoryPoints = 0

_trajectorySteering = [ ]
_trajectoryShoulder0Right = [ ]
_trajectoryShoulder1Right = [ ]
_trajectoryShoulder2Right = [ ]
_trajectoryShoulder0Left = [ ]
_trajectoryShoulder1Left = [ ]
_trajectoryShoulder2Left = [ ]
_trajectoryElbow0Right = [ ]
_trajectoryElbow1Right = [ ]
_trajectoryElbow0Left = [ ]
_trajectoryElbow1Left = [ ]
_trajectoryWrist0Right = [ ]
_trajectoryWrist1Right = [ ]
_trajectoryWrist0Left = [ ]
_trajectoryWrist1Left = [ ]

_interpolatedShoulder0Right = None
_interpolatedShoulder1Right = None
_interpolatedShoulder2Right = None
_interpolatedShoulder0Left = None
_interpolatedShoulder1Left = None
_interpolatedShoulder2Left = None
_interpolatedElbow0Right = None
_interpolatedElbow1Right = None
_interpolatedElbow0Left = None
_interpolatedElbow1Left = None
_interpolatedWrist0Right = None
_interpolatedWrist1Right = None
_interpolatedWrist0Left = None
_interpolatedWrist1Left = None

ros_right_shoulder_axis0_pub = rospy.Publisher('/right_shoulder_axis0/right_shoulder_axis0/target', Float32, queue_size=2)
ros_right_shoulder_axis1_pub = rospy.Publisher('/right_shoulder_axis1/right_shoulder_axis1/target', Float32, queue_size=2)
ros_right_shoulder_axis2_pub = rospy.Publisher('/right_shoulder_axis2/right_shoulder_axis2/target', Float32, queue_size=2)
ros_left_shoulder_axis0_pub = rospy.Publisher('/left_shoulder_axis0/left_shoulder_axis0/target', Float32, queue_size=2)
ros_left_shoulder_axis1_pub = rospy.Publisher('/left_shoulder_axis1/left_shoulder_axis1/target', Float32, queue_size=2)
ros_left_shoulder_axis2_pub = rospy.Publisher('/left_shoulder_axis2/left_shoulder_axis2/target', Float32, queue_size=2)
ros_elbow_right_rot0_pub = rospy.Publisher('/elbow_right_rot0/elbow_right_rot0/target', Float32, queue_size=2)
ros_elbow_right_rot1_pub = rospy.Publisher('/elbow_right_rot1/elbow_right_rot1/target', Float32, queue_size=2)
ros_elbow_left_rot0_pub = rospy.Publisher('/elbow_left_rot0/elbow_left_rot0/target', Float32, queue_size=2)
ros_elbow_left_rot1_pub = rospy.Publisher('/elbow_left_rot1/elbow_left_rot1/target', Float32, queue_size=2)
ros_right_wrist_0_pub = rospy.Publisher('/right_wrist_0/right_wrist_0/target', Float32, queue_size=2)
ros_right_wrist_1_pub = rospy.Publisher('/right_wrist_1/right_wrist_1/target', Float32, queue_size=2)
ros_left_wrist_0_pub = rospy.Publisher('/left_wrist_0/left_wrist_0/target', Float32, queue_size=2)
ros_left_wrist_1_pub = rospy.Publisher('/left_wrist_1/left_wrist_1/target', Float32, queue_size=2)

ros_log_error_pub = rospy.Publisher('chatter', String)

requested_steering_angle = 0
angle_change_successful = True

##############################
###   UTILITY FUNCTIONS   ###
##############################


def import_joint_trajectory_record():
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

    if loaded_data[ "num_points" ] is None:
        return 0
    else:
        _numTrajectoryPoints = loaded_data[ "num_points" ]

    for pointIterator in range(_numTrajectoryPoints):
        if ("point_" + str(pointIterator) in loaded_data):
            _trajectorySteering.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Steering_angle" ])

            _trajectoryShoulder0Right.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ JOINT_SHOULDER_AXIS0_RIGHT ])
            _trajectoryShoulder1Right.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ JOINT_SHOULDER_AXIS1_RIGHT ])
            _trajectoryShoulder2Right.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ JOINT_SHOULDER_AXIS2_RIGHT ])
            _trajectoryElbow0Right.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ JOINT_ELBOW_ROT0_RIGHT ])
            _trajectoryElbow1Right.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ JOINT_ELBOW_ROT1_RIGHT ])
            _trajectoryWrist0Right.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ JOINT_WRIST_0_RIGHT ])
            _trajectoryWrist1Right.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ JOINT_WRIST_1_RIGHT ])

            _trajectoryShoulder0Left.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ JOINT_SHOULDER_AXIS0_LEFT ])
            _trajectoryShoulder1Left.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ JOINT_SHOULDER_AXIS1_LEFT ])
            _trajectoryShoulder2Left.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ JOINT_SHOULDER_AXIS2_LEFT ])
            _trajectoryElbow0Left.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ JOINT_ELBOW_ROT0_LEFT ])
            _trajectoryElbow1Left.append(
                loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ JOINT_ELBOW_ROT1_LEFT ])
            _trajectoryWrist0Left.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ JOINT_WRIST_0_LEFT ])
            _trajectoryWrist1Left.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ JOINT_WRIST_1_LEFT ])

        else:
            print("WARNING: No point_%s in trajectory" % (pointIterator))
            _numTrajectoryPoints -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(_numTrajectoryPoints)


def interpolate_joint_angles():
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

    _interpolatedShoulder0Right = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder0Right, kind="cubic")
    _interpolatedShoulder1Right = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder1Right, kind="cubic")
    _interpolatedShoulder2Right = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder2Right, kind="cubic")
    _interpolatedElbow0Right = interpolate.interp1d(_trajectorySteering, _trajectoryElbow0Right, kind="cubic")
    _interpolatedElbow1Right = interpolate.interp1d(_trajectorySteering, _trajectoryElbow1Right, kind="cubic")
    _interpolatedWrist0Right = interpolate.interp1d(_trajectorySteering, _trajectoryWrist0Right, kind="cubic")
    _interpolatedWrist1Right = interpolate.interp1d(_trajectorySteering, _trajectoryWrist1Right, kind="cubic")

    _interpolatedShoulder0Left = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder0Left, kind="cubic")
    _interpolatedShoulder1Left = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder1Left, kind="cubic")
    _interpolatedShoulder2Left = interpolate.interp1d(_trajectorySteering, _trajectoryShoulder2Left, kind="cubic")
    _interpolatedElbow0Left = interpolate.interp1d(_trajectorySteering, _trajectoryElbow0Left, kind="cubic")
    _interpolatedElbow1Left = interpolate.interp1d(_trajectorySteering, _trajectoryElbow1Left, kind="cubic")
    _interpolatedWrist0Left = interpolate.interp1d(_trajectorySteering, _trajectoryWrist0Left, kind="cubic")
    _interpolatedWrist1Left = interpolate.interp1d(_trajectorySteering, _trajectoryWrist1Left, kind="cubic")


def get_angle_difference(angle_1, angle_2):
    return np.pi - np.abs(np.abs(angle_1 - angle_2) - np.pi)

#############################
###   CONTROL FUNCTIONS   ###
#############################

def update_steering_angle(steering_angle):
    global requested_steering_angle
    requested_steering_angle = steering_angle


def check_steering_angle_range(steering_angle):
    min_angle = min(_trajectorySteering)
    max_angle = max(_trajectorySteering)
    if min_angle <= steering_angle <= max_angle:
        return True
    else:
        ros_log_error_pub.publish("requested steering_angle (", steering_angle,") out of range "
                                                                               "[", min_angle,";",max_angle,"]")
        return False

def publish_joint_angle(joint_name, steering_angle):
    if check_steering_angle_range():

        pub = None
        f_interpolated = None

        if joint_name == JOINT_SHOULDER_AXIS0_LEFT:
            pub = ros_left_shoulder_axis0_pub
            f_interpolated = _interpolatedShoulder0Left
        elif joint_name == JOINT_SHOULDER_AXIS1_LEFT:
            pub = ros_left_shoulder_axis1_pub
            f_interpolated = _interpolatedShoulder1Left
        elif joint_name == JOINT_SHOULDER_AXIS2_LEFT:
            pub = ros_left_shoulder_axis2_pub
            f_interpolated = _interpolatedShoulder2Left
        elif joint_name == JOINT_SHOULDER_AXIS0_RIGHT:
            pub = ros_right_shoulder_axis0_pub
            f_interpolated = _interpolatedShoulder0Right
        elif joint_name == JOINT_SHOULDER_AXIS1_RIGHT:
            pub = ros_right_shoulder_axis1_pub
            f_interpolated = _interpolatedShoulder1Right
        elif joint_name == JOINT_SHOULDER_AXIS2_RIGHT:
            pub = ros_right_shoulder_axis2_pub
            f_interpolated = _interpolatedShoulder2Right
        elif joint_name == JOINT_ELBOW_ROT0_LEFT:
            pub = ros_elbow_left_rot0_pub
            f_interpolated = _interpolatedElbow0Left
        elif joint_name == JOINT_ELBOW_ROT1_LEFT:
            pub = ros_elbow_left_rot1_pub
            f_interpolated = _interpolatedElbow1Left
        elif joint_name == JOINT_ELBOW_ROT0_RIGHT:
            pub = ros_elbow_right_rot0_pub
            f_interpolated = _interpolatedElbow0Right
        elif joint_name == JOINT_ELBOW_ROT1_RIGHT:
            pub = ros_elbow_right_rot1_pub
            f_interpolated = _interpolatedElbow1Right
        elif joint_name == JOINT_WRIST_0_LEFT:
            pub = ros_left_wrist_0_pub
            f_interpolated = _interpolatedWrist0Left
        elif joint_name == JOINT_WRIST_1_LEFT:
            pub = ros_left_wrist_1_pub
            f_interpolated = _interpolatedWrist1Left
        elif joint_name == JOINT_WRIST_0_RIGHT:
            pub = ros_right_wrist_0_pub
            f_interpolated = _interpolatedWrist0Right
        elif joint_name == JOINT_WRIST_1_RIGHT:
            pub = ros_right_wrist_1_pub
            f_interpolated = _interpolatedWrist1Right
        else:
            ros_log_error_pub.publish("Didn't catch joint_name in publish_joint_angle()")

        target_joint_angle = f_interpolated(steering_angle)
        pub.publish(target_joint_angle)

    else:
        global angle_change_successful
        angle_change_successful = False
        pass


def steering_control():
    rospy.Subscriber("/cmd_steering_angle_rickshaw", Float32, update_steering_angle)
    thread_ros = Thread(target=rospy.spin)
    thread_ros.start()

    current_steering_angle = 0

    while not rospy.is_shutdown():

        while requested_steering_angle == current_steering_angle:
            time.sleep(UPDATE_FREQUENCY)


        if get_angle_difference(current_steering_angle, requested_steering_angle) > MAX_ANGLE_CHANGE:

            target_steering_angle = 0

            if current_steering_angle < requested_steering_angle:
                target_steering_angle = current_steering_angle + MAX_ANGLE_CHANGE
            else:
                target_steering_angle = current_steering_angle - MAX_ANGLE_CHANGE


            publisher_threads = []
            i = 0
            for joint in _joints_list:
                publisher_threads.append(Thread(target=publish_joint_angle, args=(joint, target_steering_angle)))
                publisher_threads[i].start()
                i += 1

            for thread in publisher_threads:
                thread.join()

            if angle_change_successful:
                current_steering_angle = target_steering_angle
            else:
                print ("Steering angle out of range: ", target_steering_angle)
                global angle_change_successful
                angle_change_successful = True

        else:
            publisher_threads = [ ]
            i = 0
            for joint in _joints_list:
                publisher_threads.append(Thread(target=publish_joint_angle, args=(joint, requested_steering_angle)))
                publisher_threads[ i ].start()
                i += 1

            for thread in publisher_threads:
                thread.join()

            if angle_change_successful:
                current_steering_angle = requested_steering_angle
            else:
                print ("Steering angle out of range: ", target_steering_angle)
                global angle_change_successful
                angle_change_successful = True


################
###   MAIN   ###
################


def main():
    rospy.init_node('steering_simulation', anonymous=True)
    import_joint_trajectory_record()
    interpolate_joint_angles()

    steering_control()


if __name__ == '__main__':
    main()
