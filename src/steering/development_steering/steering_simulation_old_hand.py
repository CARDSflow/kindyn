
## @package steering
#  Documentation for this module.
#
#  Control of Roboys' shoulders, elbows and wrists for steering.
#  In order to reach the requested steering-angle we target intermediate points
#  between the current and the requested steering-angle to ensure Roboys' hands
#  are following the captured steering-trajectory.
#
#  In order to target a point on the steering-trajectory for the hands,
#  we use an interpolation-function for the joint angles with steering-angle as input
#  that uses precomputed set-points of all joint-angles
#  according to a certain steering-angle.

from __future__ import print_function
import json
import time
from threading import Thread
import numpy as np
import numpy.polynomial.polynomial as poly
import rospy
from roboy_control_msgs.srv import SetControllerParameters
from roboy_simulation_msgs.msg import JointState
from scipy import interpolate
from std_msgs.msg import Float32, String, Float64


PRINT_DEBUG = True
RECORDED_TRAJECTORY_FILENAME = "trajectory_steering/steering_trajectory.json"
JOINT_TARGET_ERROR_TOLERANCE = 0.01
UPDATE_FREQUENCY = 0.001
MAX_ANGLE_CHANGE = np.pi / 72
STEP_TRANSITION_TIME = 2.5


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
JOINT_BIKE_FRONT = "joint_front"

_joints_list = [JOINT_SHOULDER_AXIS0_RIGHT, JOINT_SHOULDER_AXIS1_RIGHT, JOINT_SHOULDER_AXIS2_RIGHT,
                JOINT_SHOULDER_AXIS0_LEFT, JOINT_SHOULDER_AXIS1_LEFT, JOINT_SHOULDER_AXIS2_LEFT,
                JOINT_ELBOW_ROT0_RIGHT, JOINT_ELBOW_ROT1_RIGHT, JOINT_ELBOW_ROT0_LEFT, JOINT_ELBOW_ROT1_LEFT,
                JOINT_WRIST_0_RIGHT, JOINT_WRIST_1_RIGHT, JOINT_WRIST_0_LEFT, JOINT_WRIST_1_LEFT, JOINT_BIKE_FRONT]

_numTrajectoryPoints = 0

_trajectorySteering = []
_trajectoryShoulder0Right = []
_trajectoryShoulder1Right = []
_trajectoryShoulder2Right = []
_trajectoryShoulder0Left = []
_trajectoryShoulder1Left = []
_trajectoryShoulder2Left = []
_trajectoryElbow0Right = []
_trajectoryElbow1Right = []
_trajectoryElbow0Left = []
_trajectoryElbow1Left = []
_trajectoryWrist0Right = []
_trajectoryWrist1Right = []
_trajectoryWrist0Left = []
_trajectoryWrist1Left = []

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

_regressedShoulder0Right = None
_regressedShoulder1Right = None
_regressedShoulder2Right = None
_regressedShoulder0Left = None
_regressedShoulder1Left = None
_regressedShoulder2Left = None
_regressedElbow0Right = None
_regressedElbow1Right = None
_regressedElbow0Left = None
_regressedElbow1Left = None
_regressedWrist0Right = None
_regressedWrist1Right = None
_regressedWrist0Left = None
_regressedWrist1Left = None

_jointsStatusData = {
    JOINT_SHOULDER_AXIS0_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS1_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS2_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS0_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS1_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_SHOULDER_AXIS2_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_ELBOW_ROT0_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_ELBOW_ROT1_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_ELBOW_ROT0_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_ELBOW_ROT1_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_WRIST_0_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_WRIST_1_RIGHT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_WRIST_0_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    JOINT_WRIST_1_LEFT: {
        "Pos": 0.0,
        "Vel": 0.0
    }
}

ros_right_shoulder_axis0_pub = rospy.Publisher('/right_shoulder_axis0/right_shoulder_axis0/target', Float32,
                                               queue_size=2)
ros_right_shoulder_axis1_pub = rospy.Publisher('/right_shoulder_axis1/right_shoulder_axis1/target', Float32,
                                               queue_size=2)
ros_right_shoulder_axis2_pub = rospy.Publisher('/right_shoulder_axis2/right_shoulder_axis2/target', Float32,
                                               queue_size=2)
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

ros_bike_front_pub = rospy.Publisher('/joint_front/joint_front/target', Float32, queue_size=2)

ros_log_error_pub = rospy.Publisher('chatter', String, queue_size=10)

requested_steering_angle = 0
angle_change_successful = True


## Documentation for a function.
#
#  This function collects the current status of the joint-angles and saves
#  them in the global dictionary "_jointStatusData".
def joint_state_callback(joint_data):
    global _jointsStatusData
    # Assert order of joints
    for stringIter in range(len(joint_data.names)):
        if joint_data.names[stringIter] in _jointsStatusData:
            _jointsStatusData[joint_data.names[stringIter]]["Pos"] = joint_data.q[stringIter]
            _jointsStatusData[joint_data.names[stringIter]]["Vel"] = joint_data.qd[stringIter]


## Documentation for a function.
#
#  Returns current position of joint-angle @jointName.     .
def get_joint_position(joint_name):
    global _jointsStatusData
    return _jointsStatusData[joint_name ][ "Pos" ]


## Documentation for a function.
#
#  Initializes the interpolation-functions for every joint-angle using regression.
#  The input value of the function is a steering angle and the output value of the function
#  is the correspondent joint angle.
#
#  The functions can be used by calling "<function_name>(<steering_angle>)"
#  ==> returns <joint_angle>
def regress_joint_positions_from_file(filename):
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

    loaded_data = None
    with open(filename, "r") as read_file:
        loaded_data = json.load(read_file)

    _regressedShoulder0Right = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS0_RIGHT])
    _regressedShoulder1Right = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS1_RIGHT])
    _regressedShoulder2Right = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS2_RIGHT])
    _regressedElbow0Right = poly.Polynomial(loaded_data[JOINT_ELBOW_ROT0_RIGHT])
    _regressedElbow1Right = poly.Polynomial(loaded_data[JOINT_ELBOW_ROT1_RIGHT])
    _regressedWrist0Right = poly.Polynomial(loaded_data[JOINT_WRIST_0_RIGHT])
    _regressedWrist1Right = poly.Polynomial(loaded_data[JOINT_WRIST_1_RIGHT])

    _regressedShoulder0Left = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS0_LEFT])
    _regressedShoulder1Left = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS1_LEFT])
    _regressedShoulder2Left = poly.Polynomial(loaded_data[JOINT_SHOULDER_AXIS2_LEFT])
    _regressedElbow0Left = poly.Polynomial(loaded_data[JOINT_ELBOW_ROT0_LEFT])
    _regressedElbow1Left = poly.Polynomial(loaded_data[JOINT_ELBOW_ROT1_LEFT])
    _regressedWrist0Left = poly.Polynomial(loaded_data[JOINT_WRIST_0_LEFT])
    _regressedWrist1Left = poly.Polynomial(loaded_data[JOINT_WRIST_1_LEFT])

    return 1


## Documentation for a function.
#
#  Collects and saves all joint- and steering-angles from the pre-captured
#  trajectory from the @read_file (global variable).
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

    if loaded_data["num_points"] is None:
        return 0
    else:
        _numTrajectoryPoints = loaded_data["num_points"]

    for pointIterator in range(_numTrajectoryPoints):
        if "point_" + str(pointIterator) in loaded_data:
            _trajectorySteering.append(loaded_data["point_" + str(pointIterator)]["Right"]["Steering_angle"])

            _trajectoryShoulder0Right.append(
                loaded_data["point_" + str(pointIterator)]["Right"][JOINT_SHOULDER_AXIS0_RIGHT])
            _trajectoryShoulder1Right.append(
                loaded_data["point_" + str(pointIterator)]["Right"][JOINT_SHOULDER_AXIS1_RIGHT])
            _trajectoryShoulder2Right.append(
                loaded_data["point_" + str(pointIterator)]["Right"][JOINT_SHOULDER_AXIS2_RIGHT])
            _trajectoryElbow0Right.append(
                loaded_data["point_" + str(pointIterator)]["Right"][JOINT_ELBOW_ROT0_RIGHT])
            _trajectoryElbow1Right.append(
                loaded_data["point_" + str(pointIterator)]["Right"][JOINT_ELBOW_ROT1_RIGHT])
            _trajectoryWrist0Right.append(
                loaded_data["point_" + str(pointIterator)]["Right"][JOINT_WRIST_0_RIGHT])
            _trajectoryWrist1Right.append(
                loaded_data["point_" + str(pointIterator)]["Right"][JOINT_WRIST_1_RIGHT])

            _trajectoryShoulder0Left.append(
                loaded_data["point_" + str(pointIterator)]["Left"][JOINT_SHOULDER_AXIS0_LEFT])
            _trajectoryShoulder1Left.append(
                loaded_data["point_" + str(pointIterator)]["Left"][JOINT_SHOULDER_AXIS1_LEFT])
            _trajectoryShoulder2Left.append(
                loaded_data["point_" + str(pointIterator)]["Left"][JOINT_SHOULDER_AXIS2_LEFT])
            _trajectoryElbow0Left.append(
                loaded_data["point_" + str(pointIterator)]["Left"][JOINT_ELBOW_ROT0_LEFT])
            _trajectoryElbow1Left.append(
                loaded_data["point_" + str(pointIterator)]["Left"][JOINT_ELBOW_ROT1_LEFT])
            _trajectoryWrist0Left.append(loaded_data["point_" + str(pointIterator)]["Left"][JOINT_WRIST_0_LEFT])
            _trajectoryWrist1Left.append(loaded_data["point_" + str(pointIterator)]["Left"][JOINT_WRIST_1_LEFT])

        else:
            print("WARNING: No point_%s in trajectory" % pointIterator)
            _numTrajectoryPoints -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(_numTrajectoryPoints)
        print("max_angle = ", max(_trajectorySteering))
        print("min_angle = ", min(_trajectorySteering))


## Documentation for a function.
#
#  Initializes the interpolation-functions for every joint-angle using
#  cubic spline interpolation.
#  The input value of the function is a steering angle and the output value of the function
#  the correspondent joint angle.
#
#  The functions can be used by calling "<function_name>(<steering_angle>)"
#  ==> returns <joint_angle>
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


## Documentation for a function
#
#  Returns the absolute difference of two angles within the interval [0;2pi]
def get_angle_difference(angle_1, angle_2):
    return np.pi - np.abs(np.abs(angle_1 - angle_2) - np.pi)


## Documentation for a function
#
#  Sets Kp of joint-controller to  @proportional_value
#  Sets Kd of joint-controller to  @derivative_value
def set_joint_controller_parameters(proportional_value, derivative_value):
    for thisJointName in _joints_list:
        rospy.wait_for_service(thisJointName + '/' + thisJointName + '/params')
        try:
            joint_srv = rospy.ServiceProxy(thisJointName + '/' + thisJointName + '/params', SetControllerParameters)
            joint_srv(proportional_value, derivative_value)
        except rospy.ServiceException as e:
            print("Service call joint_foot_left failed:", e)


## Documentation for a function
#
#  Updates the global variable @requested_steering_angle when another node publishes a new
#  requested steering_angle to the topic "cmd_steering_angle_rickshaw".
def update_steering_angle(steering_angle_F32):
    global requested_steering_angle
    requested_steering_angle = steering_angle_F32.data

    if PRINT_DEBUG:
        log_msg = "updating requested_steering_angle: " + str(requested_steering_angle)
        print(log_msg)


## Documentation for a function.
#
#  Checks if the parameter @steering_angle is within the range of
#  reachable steering-angles of Roboy.
def check_steering_angle_range(steering_angle):
    min_angle = min(_trajectorySteering)
    max_angle = max(_trajectorySteering)
    if min_angle <= steering_angle <= max_angle:
        return True
    else:
        log_msg = "requested steering_angle (" + str(steering_angle) + ") out of range [" \
                  + str(min_angle) + ";" + str(max_angle) + "]"
        ros_log_error_pub.publish(log_msg)
        return False


## Documentation for a function.
#
#  Evaluates the correspondent joint-angle of @joint_name to given @steering_angle
#  using the interpolation-function of @joint_name
#
#  Publishes the computed value to the correspondent ros-topic of @joint_name
#  to apply position control.
#
#  Waits until the joint_angle has reached requested joint_angle within
#  error tolerance.
def publish_joint_angle(joint_name, steering_angle):
    if check_steering_angle_range(steering_angle):

        pub = None
        f_interpolated = None
        f_regressed = None

        if joint_name == JOINT_SHOULDER_AXIS0_LEFT:
            pub = ros_left_shoulder_axis0_pub
            f_interpolated = _interpolatedShoulder0Left
            f_regressed = _regressedShoulder0Left
        elif joint_name == JOINT_SHOULDER_AXIS1_LEFT:
            pub = ros_left_shoulder_axis1_pub
            f_interpolated = _interpolatedShoulder1Left
            f_regressed = _regressedShoulder1Left
        elif joint_name == JOINT_SHOULDER_AXIS2_LEFT:
            pub = ros_left_shoulder_axis2_pub
            f_interpolated = _interpolatedShoulder2Left
            f_regressed = _regressedShoulder2Left
        elif joint_name == JOINT_SHOULDER_AXIS0_RIGHT:
            pub = ros_right_shoulder_axis0_pub
            f_interpolated = _interpolatedShoulder0Right
            f_regressed = _regressedShoulder0Right
        elif joint_name == JOINT_SHOULDER_AXIS1_RIGHT:
            pub = ros_right_shoulder_axis1_pub
            f_interpolated = _interpolatedShoulder1Right
            f_regressed = _regressedShoulder1Right
        elif joint_name == JOINT_SHOULDER_AXIS2_RIGHT:
            pub = ros_right_shoulder_axis2_pub
            f_interpolated = _interpolatedShoulder2Right
            f_regressed = _regressedShoulder2Right
        elif joint_name == JOINT_ELBOW_ROT0_LEFT:
            pub = ros_elbow_left_rot0_pub
            f_interpolated = _interpolatedElbow0Left
            f_regressed = _regressedElbow0Left
        elif joint_name == JOINT_ELBOW_ROT1_LEFT:
            pub = ros_elbow_left_rot1_pub
            f_interpolated = _interpolatedElbow1Left
            f_regressed = _regressedElbow1Left
        elif joint_name == JOINT_ELBOW_ROT0_RIGHT:
            pub = ros_elbow_right_rot0_pub
            f_interpolated = _interpolatedElbow0Right
            f_regressed = _regressedElbow0Right
        elif joint_name == JOINT_ELBOW_ROT1_RIGHT:
            pub = ros_elbow_right_rot1_pub
            f_interpolated = _interpolatedElbow1Right
            f_regressed = _regressedElbow1Right
        elif joint_name == JOINT_WRIST_0_LEFT:
            pub = ros_left_wrist_0_pub
            f_interpolated = _interpolatedWrist0Left
            f_regressed = _regressedWrist0Left
        elif joint_name == JOINT_WRIST_1_LEFT:
            pub = ros_left_wrist_1_pub
            f_interpolated = _interpolatedWrist1Left
            f_regressed = _regressedWrist1Left
        elif joint_name == JOINT_WRIST_0_RIGHT:
            pub = ros_right_wrist_0_pub
            f_interpolated = _interpolatedWrist0Right
            f_regressed = _regressedWrist0Right
        elif joint_name == JOINT_WRIST_1_RIGHT:
            pub = ros_right_wrist_1_pub
            f_interpolated = _interpolatedWrist1Right
            f_regressed = _regressedWrist1Right
        elif joint_name == JOINT_BIKE_FRONT:
            pub = ros_bike_front_pub
        else:
            ros_log_error_pub.publish("Didn't catch joint_name in publish_joint_angle()")

        target_joint_angle = None
        if joint_name == JOINT_BIKE_FRONT:
            target_joint_angle = steering_angle
        else:
            # target_joint_angle = f_interpolated(steering_angle)
            target_joint_angle = f_regressed(steering_angle)
        pub.publish(target_joint_angle)

        if PRINT_DEBUG:
            log_msg = "publishing " + str(target_joint_angle) + " to joint: " + joint_name
            print(log_msg)

        # ADDED THIS FOR FEEDBACK CONTROL
        transition_end_time = time.time() + STEP_TRANSITION_TIME
        while (time.time() < transition_end_time) and abs(
                get_joint_position(joint_name) - target_joint_angle) > JOINT_TARGET_ERROR_TOLERANCE:
            time.sleep(0.001)  # Wait

        # time.sleep(STEP_TRANSITION_TIME)

    else:
        global angle_change_successful
        angle_change_successful = False
        pass


## Documentation for a function.
#
#   Controls the whole steering-process.
#   Evaluates the target_steering_angles between requested_steering_angle
#   and current_steering_angle and creates a Thread for every joint-angle
#   with which is responsible to apply correspondent joint-angle given
#   current target_steering_angle.
#
#   Simplified Pseudo-code:
#
#   while requested_steering_angle = current_steering_angle:
#
#       sleep()
#
#   if angle_difference(requested_steering_angle, current_steering_angle) > max_angle_change
#
#       target_steering_angle = current_steering_angle + max_angle_change
#
#   else:
#
#       target_steering_angle = requested_steering_angle
#
#   for joint in joint_list:
#
#       Thread.publish_joint_angle(joint_name, target_joint_angle)
#
#   for Thread in created_threads:
#
#           Thread.join
#
#   current_steering_angle = target_steering_angle
def steering_control():
    rospy.Subscriber("/target_angle", Float64, update_steering_angle)

    current_steering_angle = 0

    global angle_change_successful

    while not rospy.is_shutdown():

        if requested_steering_angle == current_steering_angle:
            if PRINT_DEBUG:
                log_msg = "\nrequested_steering_angle = current_steering_angle = " + str(requested_steering_angle)
                print(log_msg)
                while requested_steering_angle == current_steering_angle:
                    time.sleep(UPDATE_FREQUENCY)

        if PRINT_DEBUG:
            log_msg = "\nrequested_steering_angle = " + str(requested_steering_angle)
            log_msg += "\ncurrent_steering_angle = " + str(current_steering_angle)
            print(log_msg)

        if get_angle_difference(current_steering_angle, requested_steering_angle) > MAX_ANGLE_CHANGE:

            target_steering_angle = 0

            if current_steering_angle < requested_steering_angle:
                target_steering_angle = current_steering_angle + MAX_ANGLE_CHANGE
            else:
                target_steering_angle = current_steering_angle - MAX_ANGLE_CHANGE

            if PRINT_DEBUG:
                log_msg = "target_steering_angle = " + str(target_steering_angle)
                print(log_msg)

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
                print("Steering angle out of range: ", target_steering_angle)
                angle_change_successful = True

        else:
            publisher_threads = []
            i = 0
            for joint in _joints_list:
                publisher_threads.append(Thread(target=publish_joint_angle, args=(joint, requested_steering_angle)))
                publisher_threads[i].start()
                i += 1

            for thread in publisher_threads:
                thread.join()

            if angle_change_successful:
                current_steering_angle = requested_steering_angle
            else:
                print("Steering angle out of range: ", requested_steering_angle)
                angle_change_successful = True


## Documentation for a function
#
#  Initializes the Control-Node for Steering and starts Steering-Algorithm.
def main():
    rospy.init_node('steering_simulation', anonymous=True)
    rospy.Subscriber("joint_state", JointState, joint_state_callback)
    import_joint_trajectory_record()
    interpolate_joint_angles()
    regress_joint_positions_from_file("trajectory_steering/saved_coefficients.json")
    set_joint_controller_parameters(1000, 0)
    steering_control()


if __name__ == '__main__':
    main()


