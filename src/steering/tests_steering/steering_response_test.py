## @package steering
#  Documentation for this module.
#
#  Test-script for testing the transition-time between two different-steering-angles.
#  Accuracy-measurement is not important as we are using position-control for the joint-angles
#  and assume they are working.
#
#  @STEPS_DISTRIBUTION_TEST determines the amount of steps for measurement between min and max angle.
#  The program saves all transition-times between two angles going from min to max and from max to min anngle
#  and saves them in two separate lists.

from __future__ import print_function
from threading import Thread
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy.polynomial.polynomial as poly
import json
import time
import numpy as np
from scipy import interpolate
import rospy
from roboy_simulation_msgs.msg import JointState
from std_msgs.msg import Float32, Float64

PRINT_DEBUG = False

STEPS_DISTRIBUTION_TEST = [2, 4, 6, 8]
SHOW_AVERAGE = True  # Display only average transition times of step-distributions

UPDATE_FREQUENCY = 0.01
ERROR_TOLERANCE = np.pi/36
INITIAL_STARTING_TIME = 10
 
MAX_TRANSITION_TIME = 5

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

joint_status_data = {
    JOINT_SHOULDER_AXIS0_LEFT: 0,
    JOINT_SHOULDER_AXIS1_LEFT: 0,
    JOINT_SHOULDER_AXIS2_LEFT: 0,
    JOINT_SHOULDER_AXIS0_RIGHT: 0,
    JOINT_SHOULDER_AXIS1_RIGHT: 0,
    JOINT_SHOULDER_AXIS2_RIGHT: 0,
    JOINT_ELBOW_ROT0_LEFT: 0,
    JOINT_ELBOW_ROT1_LEFT: 0,
    JOINT_ELBOW_ROT0_RIGHT: 0,
    JOINT_ELBOW_ROT1_RIGHT: 0,
    JOINT_WRIST_0_LEFT: 0,
    JOINT_WRIST_1_LEFT: 0,
    JOINT_WRIST_0_RIGHT: 0,
    JOINT_WRIST_1_RIGHT: 0
}

_joints_list = [JOINT_SHOULDER_AXIS0_RIGHT, JOINT_SHOULDER_AXIS1_RIGHT, JOINT_SHOULDER_AXIS2_RIGHT,
                JOINT_SHOULDER_AXIS0_LEFT, JOINT_SHOULDER_AXIS1_LEFT, JOINT_SHOULDER_AXIS2_LEFT,
                JOINT_ELBOW_ROT0_RIGHT, JOINT_ELBOW_ROT1_RIGHT, JOINT_ELBOW_ROT0_LEFT, JOINT_ELBOW_ROT1_LEFT,
                JOINT_WRIST_0_RIGHT, JOINT_WRIST_1_RIGHT, JOINT_WRIST_0_LEFT, JOINT_WRIST_1_LEFT]

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

RECORDED_TRAJECTORY_FILENAME = "trajectory_steering/steering_trajectory.json"


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

    if loaded_data[ "num_points" ] is None:
        return 0
    else:
        _numTrajectoryPoints = loaded_data[ "num_points" ]

    for pointIterator in range(_numTrajectoryPoints):
        if "point_" + str(pointIterator) in loaded_data:
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
            print("WARNING: No point_%s in trajectory" % pointIterator)
            _numTrajectoryPoints -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(_numTrajectoryPoints)


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


## Documentation for a function.
#
#  This function collects the current status of the joint-angles and saves
#  them in the global dictionary "_jointStatusData".
def joint_state_callback(joint_data):
    global joint_status_data
    # Assert order of joints
    for stringIter in range(len(joint_data.names)):
        if joint_data.names[ stringIter ] == JOINT_SHOULDER_AXIS0_LEFT:
            joint_status_data[ JOINT_SHOULDER_AXIS0_LEFT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_SHOULDER_AXIS1_LEFT:
            joint_status_data[ JOINT_SHOULDER_AXIS1_LEFT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_SHOULDER_AXIS2_LEFT:
            joint_status_data[ JOINT_SHOULDER_AXIS2_LEFT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_SHOULDER_AXIS0_RIGHT:
            joint_status_data[ JOINT_SHOULDER_AXIS0_RIGHT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_SHOULDER_AXIS1_RIGHT:
            joint_status_data[ JOINT_SHOULDER_AXIS1_RIGHT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_SHOULDER_AXIS2_RIGHT:
            joint_status_data[ JOINT_SHOULDER_AXIS2_RIGHT ] = joint_data.q[ stringIter ]

        elif joint_data.names[ stringIter ] == JOINT_ELBOW_ROT0_LEFT:
            joint_status_data[ JOINT_ELBOW_ROT0_LEFT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_ELBOW_ROT1_LEFT:
            joint_status_data[ JOINT_ELBOW_ROT1_LEFT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_ELBOW_ROT0_RIGHT:
            joint_status_data[ JOINT_ELBOW_ROT0_RIGHT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_ELBOW_ROT1_RIGHT:
            joint_status_data[ JOINT_ELBOW_ROT1_RIGHT ] = joint_data.q[ stringIter ]

        elif joint_data.names[ stringIter ] == JOINT_WRIST_0_LEFT:
            joint_status_data[ JOINT_WRIST_0_LEFT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_WRIST_1_LEFT:
            joint_status_data[ JOINT_WRIST_1_LEFT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_WRIST_0_RIGHT:
            joint_status_data[ JOINT_WRIST_0_RIGHT ] = joint_data.q[ stringIter ]
        elif joint_data.names[ stringIter ] == JOINT_WRIST_1_RIGHT:
            joint_status_data[ JOINT_WRIST_1_RIGHT ] = joint_data.q[ stringIter ]


## Documentation for a function.
#
#  Checks if joint @joint_name has reached the joint-angle within error-tolerance
#  corresponding to the steering angle @steering_angle and returns if so or if @MAX_TRANSITION_TIME has been reached.
def check_joint_angle(joint_name, steering_angle):
    f_interpolated = None

    if joint_name == JOINT_SHOULDER_AXIS0_LEFT:
        f_interpolated = _regressedShoulder0Left
    elif joint_name == JOINT_SHOULDER_AXIS1_LEFT:
        f_interpolated = _regressedShoulder1Left
    elif joint_name == JOINT_SHOULDER_AXIS2_LEFT:
        f_interpolated = _regressedShoulder2Left
    elif joint_name == JOINT_SHOULDER_AXIS0_RIGHT:
        f_interpolated = _regressedShoulder0Right
    elif joint_name == JOINT_SHOULDER_AXIS1_RIGHT:
        f_interpolated = _regressedShoulder1Right
    elif joint_name == JOINT_SHOULDER_AXIS2_RIGHT:
        f_interpolated = _regressedShoulder2Right
    elif joint_name == JOINT_ELBOW_ROT0_LEFT:
        f_interpolated = _regressedElbow0Left
    elif joint_name == JOINT_ELBOW_ROT1_LEFT:
        f_interpolated = _regressedElbow1Left
    elif joint_name == JOINT_ELBOW_ROT0_RIGHT:
        f_interpolated = _regressedElbow0Right
    elif joint_name == JOINT_ELBOW_ROT1_RIGHT:
        f_interpolated = _regressedElbow1Right
    elif joint_name == JOINT_WRIST_0_LEFT:
        f_interpolated = _regressedWrist0Left
    elif joint_name == JOINT_WRIST_1_LEFT:
        f_interpolated = _regressedWrist1Left
    elif joint_name == JOINT_WRIST_0_RIGHT:
        f_interpolated = _regressedWrist0Right
    elif joint_name == JOINT_WRIST_1_RIGHT:
        f_interpolated = _regressedWrist1Right

    target_joint_angle = f_interpolated(steering_angle)
    current_joint_angle = joint_status_data[joint_name]

    start_time = time.time()

    while abs(target_joint_angle - current_joint_angle) > ERROR_TOLERANCE and time.time() < start_time + MAX_TRANSITION_TIME:
        current_joint_angle = joint_status_data[joint_name]
        time.sleep(UPDATE_FREQUENCY)


## Documentation for a function.
#
#  Checks if a steering-angle has been reached with checking if all joint-angles have reached the correspondent
#  joint-angles
def steering_angle_reached(steering_angle):
    threads = [ ]
    i = 0
    for joint in _joints_list:
        threads.append(Thread(target=check_joint_angle, args=(joint, steering_angle)))
        threads[ i ].start()
        i += 1

    for thread in threads:
        thread.join()


## Documentation for a function.
#
#  Program for testing the transition-time between two different steering-angles.
#  Saves measurements in lists and displays them using pyplot, or just the average values for all steps in
#  @STEPS_DISTRIBUTION_TEST
def steering_test(pub):
    transition_times_increasing = [[]] * len(STEPS_DISTRIBUTION_TEST)
    transition_times_decreasing = [[]] * len(STEPS_DISTRIBUTION_TEST)

    increasing_angles = [[]] * len(STEPS_DISTRIBUTION_TEST)
    decreasing_angles = [[]] * len(STEPS_DISTRIBUTION_TEST)

    min_angle = min(_trajectorySteering)+0.01
    max_angle = max(_trajectorySteering)-0.01
    max_angle_difference = max_angle - min_angle
    pub.publish(min_angle)
    time.sleep(INITIAL_STARTING_TIME)

    for j in range(len(STEPS_DISTRIBUTION_TEST)):

        print("\nStarting with amount of steps from min to max: ", STEPS_DISTRIBUTION_TEST[j])
        for i in range(STEPS_DISTRIBUTION_TEST[j]):
            start_time = time.time()
            target_angle = min_angle+(max_angle_difference/STEPS_DISTRIBUTION_TEST[j]*i)
            print("Step ", i, ": target_angle = ", target_angle)
            increasing_angles[j].append(target_angle)
            pub.publish(target_angle)
            steering_angle_reached(target_angle)
            end_time = time.time()
            transition_time = end_time-start_time
            print("transition_time (s) = ", transition_time)
            transition_times_increasing[j].append(transition_time)

        print("\nStarting with amount of steps from max to min: ", 2 ** j)
        for i in range(STEPS_DISTRIBUTION_TEST[j]):
            start_time = time.time()
            target_angle = max_angle-(max_angle_difference/STEPS_DISTRIBUTION_TEST[j]*i)
            print("Step ", i, ": target_angle = ", target_angle)
            decreasing_angles[j].append(target_angle)
            pub.publish(target_angle)
            steering_angle_reached(target_angle)
            end_time = time.time()
            transition_time = end_time-start_time
            print("transition_time (s) = ", transition_time)
            transition_times_decreasing[j].append(transition_time)

    if not SHOW_AVERAGE:
        for i in range(len(STEPS_DISTRIBUTION_TEST)):
            plt.plot(increasing_angles[i], transition_times_increasing[i],
                     label="transition times with increasing step-length :"+str(max_angle_difference/STEPS_DISTRIBUTION_TEST[i]))
            plt.plot(decreasing_angles[ i ], transition_times_decreasing[ i ],
                     label="transition times with decreasing step-length :" + str(max_angle_difference / STEPS_DISTRIBUTION_TEST[i]))

        plt.xlabel("target_angle")
        plt.ylabel("transition time in s")

    else:
        step_lengths = []
        for i in range(len(STEPS_DISTRIBUTION_TEST)):
            step_lengths.append(max_angle_difference/STEPS_DISTRIBUTION_TEST[i])
        avg_times_increasing = []
        avg_times_decreasing = []
        for transition_times in transition_times_increasing:
            avg_times_increasing.append(sum(transition_times)/len(transition_times))
        for transition_times in transition_times_decreasing:
            avg_times_decreasing.append(sum(transition_times)/len(transition_times))

        plt.plot(step_lengths, avg_times_increasing, label="avg transition time with increasing step-length")
        plt.plot(step_lengths, avg_times_decreasing, label="avg transition time with decreasing step-length")

        plt.xlabel("step_length")
        plt.ylabel("avg transition time in s")

    plt.legend()
    plt.show()


## Documentation for a function
#
#  Initializes the Test-Node for the steering-test
def main():
    pub = rospy.Publisher('/target_angle', Float64, queue_size=10)
    rospy.Subscriber("joint_state", JointState, joint_state_callback)
    rospy.init_node('steering_test', anonymous=True)
    import_joint_trajectory_record()
    regress_joint_positions_from_file("trajectory_steering/saved_coefficients.json")
    steering_test(pub)


if __name__ == '__main__':
    main()



