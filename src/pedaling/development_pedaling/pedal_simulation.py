## @package pedaling
#  Documentation for this module.
#
#  Control of Roboys' hip, knees and feet for pedaling.
#  In order to reach the requested bike-velocity, we compute correspondent angular-velocity for the
#  pedals. The circulation is ideally divided into @NUMBER_CIRCULATION_POINTS trajectory points represented by angles
#  within the circulation circle (pedal-angle). With angle-difference between two points and angular-velocity we use
#  the correspondent transition time to control the joints in order to create a pedaling motion for Roboy.
#
#  For every joint, we use cubic spline interpolation of pre-captured set points with pedal_angle as input to receive
#  continuous function to be able to get the corresponding joint-angle for every pedal-angle.
#  This enables to use joint-angle-difference between two trajectory-points and transition time to compute and apply
#  velocity-control to the joint-angles to create pedaling-motion
#
#  In order to decrease and hopefully erase velocity-error, we multiply the computed (ideal) velocity with an
#  error-factor which is computed and dependent on the velocity-error of previous transitions and measured
#  by pedal-angle-error


from __future__ import print_function
import json
import math
import time
from threading import Thread

from geometry_msgs.msg import Twist
from scipy import interpolate
import numpy as np
import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from roboy_simulation_msgs.msg import JointState
from roboy_control_msgs.srv import SetControllerParameters
from std_msgs.msg import Float32

PRINT_DEBUG = True
SIMULATION_FACTOR = 100.0  # factor to slow down the motion for better simulation
NUMBER_CIRCULATION_POINTS = 30  # number of points for controlling
RECORDED_TRAJECTORY_FILENAME = "trajectory_pedaling/captured_trajectory_old.json"
JOINT_VELOCITY_FACTOR_SIMULATION = 0.01  # publish 1 => velocity = 0.01 rad/s  for Kp = 0.1 and simulation-step-length = 0.01

PEDAL_POSITION_ERROR_TOLERANCE = 0.02  # [meters]
JOINT_TRAJECTORY_ERROR_TOLERANCE = 0.02
CONTROLLER_FREQUENCY = 100  # [Hz]

MIN_JOINT_VEL = -500.0
MAX_JOINT_VEL = 500.0

RADIUS_BACK_TIRE = 0.294398  # in m
RADIUS_GEAR_CLUSTER = 0.06  # in m
RADIUS_FRONT_CHAIN_RING = 0.075

BIKE_VELOCITY = 0.0
PEDAL_SINGLE_ROTATION_DURATION = 0.0  # [seconds]
TRAJECTORY_POINT_DURATION = 0.0

x_pedal_record = [ ]
y_pedal_record = [ ]

ROS_JOINT_HIP_RIGHT = "joint_hip_right"
ROS_JOINT_KNEE_RIGHT = "joint_knee_right"
ROS_JOINT_ANKLE_RIGHT = "joint_foot_right"
ROS_JOINT_HIP_LEFT = "joint_hip_left"
ROS_JOINT_KNEE_LEFT = "joint_knee_left"
ROS_JOINT_ANKLE_LEFT = "joint_foot_left"

RIGHT_HIP_JOINT = "right_hip"
RIGHT_KNEE_JOINT = "right_knee"
RIGHT_ANKLE_JOINT = "right_ankle"
LEFT_HIP_JOINT = "left_hip"
LEFT_KNEE_JOINT = "left_knee"
LEFT_ANKLE_JOINT = "left_ankle"

f_interpolated_hip_right = None
f_interpolated_hip_left = None
f_interpolated_knee_right = None
f_interpolated_knee_left = None
f_interpolated_ankle_right = None
f_interpolated_ankle_left = None
f_interpolated_pedal_angle = None

velocity_error_factor_hip = 1.0
velocity_error_factor_knee = 1.0
velocity_error_factor_ankle = 1.0
velocity_error_counter = 0

ros_right_hip_publisher = rospy.Publisher('/joint_hip_right/joint_hip_right/target', Float32, queue_size=2)
ros_right_knee_publisher = rospy.Publisher('/joint_knee_right/joint_knee_right/target', Float32, queue_size=2)
ros_right_ankle_publisher = rospy.Publisher('/joint_foot_right/joint_foot_right/target', Float32, queue_size=2)

ros_left_hip_publisher = rospy.Publisher('/joint_hip_left/joint_hip_left/target', Float32, queue_size=2)
ros_left_knee_publisher = rospy.Publisher('/joint_knee_left/joint_knee_left/target', Float32, queue_size=2)
ros_left_ankle_publisher = rospy.Publisher('/joint_foot_left/joint_foot_left/target', Float32, queue_size=2)

PEDAL_CENTER_OFFSET_X = 0.20421
PEDAL_CENTER_OFFSET_Y = -0.00062
PEDAL_CENTER_OFFSET_Z = 0.2101

_jointsList = [ RIGHT_HIP_JOINT, RIGHT_KNEE_JOINT, RIGHT_ANKLE_JOINT, LEFT_HIP_JOINT, LEFT_KNEE_JOINT,
                LEFT_ANKLE_JOINT ]

_parametersRightHip = {
    "param_p": 1500.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersRightKnee = {
    "param_p": 2000.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersRightAnkle = {
    "param_p": 1000.0,
    "param_i": 0.0,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersLeftHip = {
    "param_p": 1500.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersLeftKnee = {
    "param_p": 2000.0,
    "param_i": 0.05,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_parametersLeftAnkle = {
    "param_p": 1000.0,
    "param_i": 0.0,
    "param_d": 0.0,
    "prev_pos": 0.0,
    "prev_vel": 0.0,
    "prev_time": 0.0,
    "prev_error": 0.0,
    "pos_error_integral": 0.0,
    "trajectory_startpoint": 0.0,
    "trajectory_endpoint": 0.0,
    "ideal_velocity": 0.0,
    "bool_update_iv": True
}

_jointsControlData = {
    RIGHT_HIP_JOINT: _parametersRightHip,
    RIGHT_KNEE_JOINT: _parametersRightKnee,
    RIGHT_ANKLE_JOINT: _parametersRightAnkle,
    LEFT_HIP_JOINT: _parametersLeftHip,
    LEFT_KNEE_JOINT: _parametersLeftKnee,
    LEFT_ANKLE_JOINT: _parametersLeftAnkle
}

joint_status_data = {
    RIGHT_HIP_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    LEFT_HIP_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    RIGHT_KNEE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    LEFT_KNEE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    RIGHT_ANKLE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    },
    LEFT_ANKLE_JOINT: {
        "Pos": 0.0,
        "Vel": 0.0
    }
}

number_imported_trajectory_points = -1
trajectoryStartingPoint = 0

pedalTrajectoryRight = [ ]
pedalAngleTrajectoryRight = [ ]
pedalTrajectoryLeft = [ ]
hipTrajectoryRight = [ ]
kneeTrajectoryRight = [ ]
ankleTrajectoryRight = [ ]
hipTrajectoryLeft = [ ]
kneeTrajectoryLeft = [ ]
ankleTrajectoryLeft = [ ]


## Documentation for a function.
#
#  This function collects the current status of the joint-angles and saves
#  them in the global dictionary "joint_status_data".
def joint_state_callback(joint_data):
    global joint_status_data
    # Assert order of joints
    for stringIter in range(len(joint_data.names)):
        if joint_data.names[ stringIter ] == ROS_JOINT_HIP_RIGHT:
            joint_status_data[ RIGHT_HIP_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ RIGHT_HIP_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_HIP_LEFT:
            joint_status_data[ LEFT_HIP_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ LEFT_HIP_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_KNEE_RIGHT:
            joint_status_data[ RIGHT_KNEE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ RIGHT_KNEE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_KNEE_LEFT:
            joint_status_data[ LEFT_KNEE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ LEFT_KNEE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_ANKLE_RIGHT:
            joint_status_data[ RIGHT_ANKLE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ RIGHT_ANKLE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]
        elif joint_data.names[ stringIter ] == ROS_JOINT_ANKLE_LEFT:
            joint_status_data[ LEFT_ANKLE_JOINT ][ "Pos" ] = joint_data.q[ stringIter ]
            joint_status_data[ LEFT_ANKLE_JOINT ][ "Vel" ] = joint_data.qd[ stringIter ]


## Documentation for a function.
#
#  Collects and saves all joint- and pedal-angles from the pre-captured
#  trajectory from the @read_file (global variable).
def import_joint_trajectory_record():
    global number_imported_trajectory_points
    global pedalTrajectoryLeft
    global pedalTrajectoryRight
    global hipTrajectoryRight
    global kneeTrajectoryRight
    global ankleTrajectoryRight
    global hipTrajectoryLeft
    global kneeTrajectoryLeft
    global ankleTrajectoryLeft
    global PRINT_DEBUG

    with open(RECORDED_TRAJECTORY_FILENAME, "r") as read_file:
        loaded_data = json.load(read_file)

    if loaded_data[ "num_points" ] is None:
        return 0
    else:
        number_imported_trajectory_points = loaded_data[ "num_points" ]

    # Deleting previous trajectory before loading new
    del pedalTrajectoryLeft[ : ]
    del pedalTrajectoryRight[ : ]
    del pedalAngleTrajectoryRight[ : ]
    del hipTrajectoryRight[ : ]
    del kneeTrajectoryRight[ : ]
    del ankleTrajectoryRight[ : ]
    del hipTrajectoryLeft[ : ]
    del kneeTrajectoryLeft[ : ]
    del ankleTrajectoryLeft[ : ]
    for pointIterator in range(number_imported_trajectory_points):
        if "point_" + str(pointIterator) in loaded_data:
            pedalTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Pedal" ])
            pedalTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Pedal" ])
            pedalAngleTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Pedal_angle" ])
            hipTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Hip" ])
            kneeTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Knee" ])
            ankleTrajectoryRight.append(loaded_data[ "point_" + str(pointIterator) ][ "Right" ][ "Ankle" ])
            hipTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Hip" ])
            kneeTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Knee" ])
            ankleTrajectoryLeft.append(loaded_data[ "point_" + str(pointIterator) ][ "Left" ][ "Ankle" ])
        else:
            print("WARNING: No point_%s in trajectory" % pointIterator)
            number_imported_trajectory_points -= 1

    if PRINT_DEBUG:
        print("--------- Num trajectory points:")
        print(number_imported_trajectory_points)


## Documentation for a function.
#
#  Return current joint-angle of @jointName.
def get_joint_position(jointName):
    global joint_status_data
    return joint_status_data[ jointName ][ "Pos" ]


## Documentation for a function.
#
#  Return current joint-velocity of @jointName.
def get_joint_velocity(jointName):
    global joint_status_data
    return joint_status_data[ jointName ][ "Vel" ]


## Documentation for a function.
#
#  Return the postion of the @frame of the correspondent @endeffector.
def get_position(endeffector, frame):
    fkJointNamesList = [ ROS_JOINT_HIP_RIGHT, ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_KNEE_LEFT,
                         ROS_JOINT_ANKLE_RIGHT, ROS_JOINT_ANKLE_LEFT ]
    fkJointPositions = [ joint_status_data[ RIGHT_HIP_JOINT ][ "Pos" ], joint_status_data[ LEFT_HIP_JOINT ][ "Pos" ],
                         joint_status_data[ RIGHT_KNEE_JOINT ][ "Pos" ], joint_status_data[ LEFT_KNEE_JOINT ][ "Pos" ],
                         joint_status_data[ RIGHT_ANKLE_JOINT ][ "Pos" ],
                         joint_status_data[ LEFT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv(endeffector, frame, fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException as e:
        print("Service call failed:", e)
    return [ 0.0, 0.0 ]  # [x, z]


## Documentation for a function
#
#  Sets Kp of joint-controller to  @proportional_value
#  Sets Kd of joint-controller to  @derivative_value
def set_joint_controller_parameters(proportionalVal, derivativeVal):
    for thisJointName in _jointsList:
        rospy.wait_for_service(thisJointName + '/' + thisJointName + '/params')
        try:
            param_srv = rospy.ServiceProxy(thisJointName + '/' + thisJointName + '/params', SetControllerParameters)
            param_srv(proportionalVal, derivativeVal)
        except rospy.ServiceException as e:
            print("Service call for " + thisJointName + "failed:", e)

    print("Controller parameters updated")


## Documentation for a function
#
#  Return the position of the left foot of Roboy.
def get_position_left_foot():
    fkJointNamesList = [ ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT ]
    fkJointPositions = [ joint_status_data[ LEFT_HIP_JOINT ][ "Pos" ], joint_status_data[ LEFT_KNEE_JOINT ][ "Pos" ],
                         joint_status_data[ LEFT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_left_tip", "foot_left_tip", fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException as e:
        print("Service call failed:", e)

    print("ERROR fk foot_left failed")
    return [ 0.0, 0.0 ]  # [x, z]


## Documentation for a function
#
#  Return the position of the right foot of Roboy.
def get_position_right_foot():
    fkJointNamesList = [ ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT ]
    fkJointPositions = [ joint_status_data[ RIGHT_HIP_JOINT ][ "Pos" ], joint_status_data[ RIGHT_KNEE_JOINT ][ "Pos" ],
                         joint_status_data[ RIGHT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_right_tip", "foot_right_tip", fkJointNamesList, fkJointPositions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException as e:
        print("Service call failed:", e)

    print("ERROR fk foot_right failed")
    return [ 0.0, 0.0 ]  # [x, z]


## Documentation for a function
#
#  Return the distance between two points where points are a list of two coordinates.
def get_distance(point1, point2):
    x_diff = point2[ 0 ] - point1[ 0 ]
    y_diff = point2[ 1 ] - point1[ 1 ]

    return math.sqrt((x_diff * x_diff) + (y_diff * y_diff))


## Documentation for a function
#
#  Checks if @inputVal is inside possible range of joint-angle velocities.
#  If not, returns max-velocity if @inputVal too high or min-velocity if @inputVal to low instead of @inputVal.
def check_output_limits(inputVal):
    returnVal = inputVal

    if inputVal > MAX_JOINT_VEL:
        returnVal = MAX_JOINT_VEL
    elif inputVal < MIN_JOINT_VEL:
        returnVal = MIN_JOINT_VEL

    return returnVal


## Documentation for a function
#
#  Returns ideal joint-velocity for @joint_name according to @end_time and joint-angle-difference
#  between @current_joint_angle and @next_joint_angle:
#
#  ideal_velocity = joint_angle_difference / (end_time - current_time)
def compute_velocity(joint_name, next_joint_angle, current_joint_angle, end_time):
    joint_angle_difference = next_joint_angle - current_joint_angle

    current_time = time.time()
    publish_time = end_time - current_time

    ideal_velocity = joint_angle_difference / publish_time

    if PRINT_DEBUG:
        log_string = "\n"
        log_string += ("\n" + joint_name + ":")
        log_string += ("\ncurrent_joint_angle = " + str(current_joint_angle))
        log_string += ("\nnext_joint_angle = " + str(next_joint_angle))
        log_string += ("\nd = " + str(joint_angle_difference))
        log_string += ("\npublish_time = " + str(publish_time))
        log_string += ("\nideal_velocity = " + str(ideal_velocity))
        print(log_string)

    return ideal_velocity


## Documentation for a function
#
#  Evaluate and return joint-angle of @joint_name correspondent to @trajectory_angle using the interpolated function:
#
#  The functions can be used by calling "<function_name>(<pedal_angle>)"
#  ==> returns <joint_angle>
def get_joint_angle(joint_name, pedal_angle):
    if joint_name == RIGHT_HIP_JOINT:
        return f_interpolated_hip_right(pedal_angle)
    elif joint_name == RIGHT_KNEE_JOINT:
        return f_interpolated_knee_right(pedal_angle)
    elif joint_name == RIGHT_ANKLE_JOINT:
        return f_interpolated_ankle_right(pedal_angle)
    elif joint_name == LEFT_HIP_JOINT:
        return f_interpolated_hip_left(pedal_angle)
    elif joint_name == LEFT_KNEE_JOINT:
        return f_interpolated_knee_left(pedal_angle)
    elif joint_name == LEFT_ANKLE_JOINT:
        return f_interpolated_ankle_left(pedal_angle)


## Documentation for a function
#
#  Initializing interpolated functions for joint-angles using pre-captured set points with pedal-angle as input and
#  joint-angle as output:
#
#  The functions can be used by calling "<function_name>(<pedal_angle>)"
#  ==> returns <joint_angle>
def interpolate_functions():
    global f_interpolated_hip_right
    global f_interpolated_hip_left
    global f_interpolated_knee_right
    global f_interpolated_knee_left
    global f_interpolated_ankle_right
    global f_interpolated_ankle_left
    global f_interpolated_pedal_angle

    f_interpolated_hip_right = interpolate.interp1d(pedalAngleTrajectoryRight, hipTrajectoryRight, kind="cubic")
    f_interpolated_knee_right = interpolate.interp1d(pedalAngleTrajectoryRight, kneeTrajectoryRight, kind="cubic")
    f_interpolated_ankle_right = interpolate.interp1d(pedalAngleTrajectoryRight, ankleTrajectoryRight, kind="cubic")
    f_interpolated_hip_left = interpolate.interp1d(pedalAngleTrajectoryRight, hipTrajectoryLeft, kind="cubic")
    f_interpolated_knee_left = interpolate.interp1d(pedalAngleTrajectoryRight, kneeTrajectoryLeft, kind="cubic")
    f_interpolated_ankle_left = interpolate.interp1d(pedalAngleTrajectoryRight, ankleTrajectoryLeft, kind="cubic")


## Documentation for a function
#
#  Evaluating the current pedal-angle according to the current position of the left foot @current_point.
#  Using trigonometric functions for the evaluation of the angle.
def evaluate_current_pedal_angle(current_point):
    current_x = current_point[ 0 ] - PEDAL_CENTER_OFFSET_X
    current_y = current_point[ 1 ] - PEDAL_CENTER_OFFSET_Y

    if current_x > 0 and current_y > 0:
        return np.arctan(current_y / current_x)
    elif current_x < 0 < current_y:
        return np.arctan(current_y / current_x) + np.pi
    elif current_x < 0 and current_y < 0:
        return np.arctan(current_y / current_x) + np.pi
    elif current_x > 0 > current_y:
        return np.arctan(current_y / current_x) + 2 * np.pi

    elif current_x == 0 and current_y > 0:
        return np.pi / 2
    elif current_x == 0 and current_y < 0:
        return np.pi * 3 / 2
    elif current_x > 0 and current_y == 0:
        return 0
    elif current_x < 0 and current_y == 0:
        return np.pi


## Documentation for a function
#
#  For joint @joint_name:
#
#  - Evaluate ideal velocity in rad/s according to joint-angle-difference and end-time of transition
#  - Multiply value with @JOINT_VELOCITY_FACTOR_SIMULATION (global variable) to receive velocity in rad/s
#  - Multiply value with @SIMULATION_FACTOR (global variable) to slow down simulation time.
#  - Multiply value with @error_factor (global variable) of correspondent joint to erase joint-error
#  - Publish velocity to joint-controller and sleep until end-time of transition
def publish_velocity(joint_name, next_joint_angle, current_joint_angle, end_time):
    ideal_velocity = compute_velocity(joint_name, next_joint_angle, current_joint_angle, end_time)
    publisher = None
    error_factor = 1

    if joint_name == RIGHT_HIP_JOINT:
        publisher = ros_right_hip_publisher
        error_factor = velocity_error_factor_hip
        return
    elif joint_name == RIGHT_KNEE_JOINT:
        publisher = ros_right_knee_publisher
        error_factor = velocity_error_factor_knee
        return
    elif joint_name == RIGHT_ANKLE_JOINT:
        publisher = ros_right_ankle_publisher
        error_factor = velocity_error_factor_ankle
        return
    elif joint_name == LEFT_HIP_JOINT:
        publisher = ros_left_hip_publisher
        error_factor = velocity_error_factor_hip
    elif joint_name == LEFT_KNEE_JOINT:
        publisher = ros_left_knee_publisher
        error_factor = velocity_error_factor_knee
    elif joint_name == LEFT_ANKLE_JOINT:
        publisher = ros_left_ankle_publisher
        error_factor = velocity_error_factor_ankle

    published_velocity = ideal_velocity * error_factor / JOINT_VELOCITY_FACTOR_SIMULATION / SIMULATION_FACTOR

    if PRINT_DEBUG:
	log_msg = "error_factor = " + str(error_factor) + "\njoint_velocity_factor_simulation = " + str(JOINT_VELOCITY_FACTOR_SIMULATION) + "\nsimulation_factor = " + str(SIMULATION_FACTOR)
        log_msg += "\npublishing velocity "+str(published_velocity*JOINT_VELOCITY_FACTOR_SIMULATION)+" rad/s to "+joint_name
        print(log_msg)

    duration = end_time - time.time()

    publisher.publish(published_velocity)
    time.sleep(duration * SIMULATION_FACTOR)


## Documentation for a function
#
#  Updates the global variables @BIKE_VELOCITY, @PEDAL_SINGLE_ROTATION_DURATION and TRAJECTORY_POINT_DURATION
#  when a bike-velocity gets published to the topic "cmd_vel".
def update_velocity(velocity_Twist):
    global PEDAL_SINGLE_ROTATION_DURATION
    global TRAJECTORY_POINT_DURATION
    global BIKE_VELOCITY

    velocity = velocity_Twist.linear.x

    BIKE_VELOCITY = velocity

    if velocity == 0:
        ros_right_hip_publisher.publish(0)
        ros_right_knee_publisher.publish(0)
        ros_right_ankle_publisher.publish(0)
        ros_left_hip_publisher.publish(0)
        ros_left_knee_publisher.publish(0)
        ros_left_ankle_publisher.publish(0)

    else:
        PEDAL_SINGLE_ROTATION_DURATION = 2 * np.pi * (RADIUS_FRONT_CHAIN_RING / RADIUS_GEAR_CLUSTER /
                                                      (velocity / RADIUS_BACK_TIRE))
        TRAJECTORY_POINT_DURATION = PEDAL_SINGLE_ROTATION_DURATION / NUMBER_CIRCULATION_POINTS


## Documentation for a function
#
#  Returns the absolute difference of two angles within the interval [0;2pi]
def get_angle_difference(angle_1, angle_2):
    return np.pi - np.abs(np.abs(angle_1 - angle_2) - np.pi)


## Documentation for a function.
#
#  Controls the whole pedaling-process.
#
#  Evaluates next pedal-angle according to current pedal-angle and @NUMBER_CIRCULATION_POINTS (global variable).
#  Uses @BIKE_VELOCITY to compute joint-velocities for every joint-angle for every transition between two.
#  trajectory points.
#
#  Use Threads to simultaneously publish and control joint-angles.
#
#  Computes joint-angle-error and adjusts error-factors for every joint to optimize ideal
#  joint-velocity for further transitions.
#
#  Simplified Pseudo-Code:
#
#   while bike_velocity = 0:
#
#       for joint in joints:
#
#           publish(0)
#
#       sleep()
#
#   current_pedal_angle = get_current_pedal_angle(left_foot_position)
#
#   next_pedal_angle = current_pedal_angle + (2pi / number_circulation_points)
#
#   for joint in joints:
#
#       next_joint_angle = interpolation_function(next_pedal_angle)
#
#       Thread.publish_velocity(joint_name, current_joint_angle, next_joint_angle, end_time)
#
#   for Thread in created_threads:
#
#       Thread.join
#
#   for joint in joints:
#
#       update_error_factor()
def control_pedaling():
    global NUMBER_CIRCULATION_POINTS
    global _jointsControlData
    global _jointsList

    global x_pedal_record
    global y_pedal_record
    global pedalTrajectoryRight

    global velocity_error_factor_hip
    global velocity_error_factor_knee
    global velocity_error_factor_ankle
    global velocity_error_factor_hip
    global velocity_error_factor_knee
    global velocity_error_factor_ankle

    INIT = "INIT"
    PEDAL = "PEDAL"

    _runFSM = True

    _currState = INIT
    _currTrajectoryPoint = get_position_left_foot()
    current_pedal_angle = evaluate_current_pedal_angle(_currTrajectoryPoint)
    next_pedal_angle = (current_pedal_angle + (2 * np.pi / NUMBER_CIRCULATION_POINTS)) % (np.pi * 2)

    _startTime = 0.0
    _endTime = 0.0
    _currTime = 0.0
    _prevTime = 0.0

    trajectory_points = 0

    while _runFSM:

        if _currState == INIT:

            import_joint_trajectory_record()
            interpolate_functions()
            _currState = PEDAL

            while PEDAL_SINGLE_ROTATION_DURATION == 0:
                pass
                # wait for velocity != 0

        if _currState == PEDAL:

            while PEDAL_SINGLE_ROTATION_DURATION == 0:
                pass
                # wait for velocity != 0

            # Initialize state
            _startTime = time.time()
            _endTime = _startTime + TRAJECTORY_POINT_DURATION

            x_pedal_record.append(_currTrajectoryPoint[ 0 ])
            y_pedal_record.append(_currTrajectoryPoint[ 1 ])

            # Regulate update frequency
            _currTime = time.time()
            while float(float(_currTime) - float(_prevTime)) < (1 / CONTROLLER_FREQUENCY):
                time.sleep(1)
                x_pedal_record.append(_currTrajectoryPoint[ 0 ])
                y_pedal_record.append(_currTrajectoryPoint[ 1 ])
                _currTime = time.time()
            _prevTime = _currTime

            _currTrajectoryPoint = get_position_left_foot()
            current_pedal_angle = evaluate_current_pedal_angle(_currTrajectoryPoint)

            _startTime = time.time()
            _endTime = _startTime + TRAJECTORY_POINT_DURATION
            next_pedal_angle = (current_pedal_angle + (2 * np.pi / NUMBER_CIRCULATION_POINTS)) % (2 * np.pi)

            trajectory_points += 1

            if PRINT_DEBUG:
                print("\n\ntrajectory_point = ", trajectory_points, ":")
                print("bike_velocity = ", BIKE_VELOCITY)
                print("rotation_duration = ", PEDAL_SINGLE_ROTATION_DURATION)
                print("trajectory_point_duration = ", TRAJECTORY_POINT_DURATION)
                print("current_pedal_angle = ", current_pedal_angle)
                print("next_pedal_angle = ", next_pedal_angle)
                print("d = ", get_angle_difference(current_pedal_angle, next_pedal_angle))

            # Iterate through joints and update setpoints
            publisher_threads = [ ]
            i = 0
            for thisJointName in _jointsList:
                current_joint_angle = joint_status_data[ thisJointName ][ "Pos" ]
                next_joint_angle = get_joint_angle(thisJointName, next_pedal_angle)

                _currTime = time.time()

                publisher_threads.append(Thread(target=publish_velocity, args=(thisJointName, next_joint_angle,
                                                                               current_joint_angle, _endTime)))
                publisher_threads[ i ].start()
                i += 1

            for thread in publisher_threads:
                thread.join()

            for joint in _jointsList:

                actual_joint_angle = get_joint_angle(joint, evaluate_current_pedal_angle(get_position_left_foot()))
                error = get_angle_difference(actual_joint_angle, next_joint_angle)

                new_factor = get_angle_difference(current_joint_angle, next_joint_angle) \
                             / get_angle_difference(current_joint_angle, actual_joint_angle)

                if np.abs(new_factor - 1) >= 0.5:
                    if new_factor < 1:
                        new_factor = 0.5
                    else:
                        new_factor = 1.5

                if thisJointName == RIGHT_HIP_JOINT:
                    velocity_error_factor_hip = ((velocity_error_factor_hip * velocity_error_counter)
                                                   + (new_factor * velocity_error_factor_hip)) / (
                                                              velocity_error_counter + 1)
                elif thisJointName == RIGHT_KNEE_JOINT:
                    velocity_error_factor_knee = ((velocity_error_factor_knee * velocity_error_counter)
                                                   + (new_factor * velocity_error_factor_knee)) / (
                                                              velocity_error_counter + 1)
                elif thisJointName == RIGHT_ANKLE_JOINT:
                    velocity_error_factor_ankle = ((velocity_error_factor_ankle * velocity_error_counter)
                                                   + (new_factor * velocity_error_factor_ankle)) / (
                                                              velocity_error_counter + 1)
                elif thisJointName == LEFT_HIP_JOINT:
                    velocity_error_factor_hip = ((velocity_error_factor_hip * velocity_error_counter)
                                                   + (new_factor * velocity_error_factor_hip)) / (
                                                              velocity_error_counter + 1)
                elif thisJointName == LEFT_KNEE_JOINT:
                    velocity_error_factor_knee = ((velocity_error_factor_knee * velocity_error_counter)
                                                   + (new_factor * velocity_error_factor_knee)) / (
                                                              velocity_error_counter + 1)
                elif thisJointName == LEFT_ANKLE_JOINT:
                    velocity_error_factor_ankle = ((velocity_error_factor_ankle * velocity_error_counter)
                                                   + (new_factor * velocity_error_factor_ankle)) / (
                                                              velocity_error_counter + 1)

            global velocity_error_counter
            velocity_error_counter += 1


## Documentation for a function.
#
#  Initializes the Control-Node for Pedaling and starts Pedaling-Algorithm.
def main():
    rospy.init_node('pedal_simulation', anonymous=True)
    rospy.Subscriber("joint_state", JointState, joint_state_callback)
    rospy.Subscriber("/cmd_vel", Twist, update_velocity)
    control_pedaling()

    return 1


if __name__ == '__main__':
    main()
