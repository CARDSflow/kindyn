## @package pedaling
#  Documentation for this module.
#
#  Test-script for testing the accuracy of a requested velocity and the transition-time (acceleration) between two
#  different velocities, which is only important for testing in reality, because there is no transition-time between
#  different velocities in simulation.
#
#  @TIME_STEP_SIMULATION and @TIME_STEP_REALITY determine the time-steps in seconds for the measurement of velocity
#  @VELOCITY_STEPS_REALITY and @VELOCITY_STEP_SIMULATION  determine the size of a step between two different velocities
#  in m/s

from __future__ import print_function
import time
import numpy as np
import matplotlib.pyplot as plt
import rospy
from roboy_middleware_msgs.srv import InverseKinematics, ForwardKinematics
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

TIME_STEP_SIMULATION = 0.5
TIME_STEP_REALITY = 0.5
VELOCITY_STEP_SIMULATION = 0.1
VELOCITY_STEPS_REALITY = [0.1, 0.2, 0.5]
ERROR_TOLERANCE_REALITY = 2*np.pi / 720
MAX_VELOCITY = 5


PEDAL_CENTER_OFFSET_X = 0.20421
PEDAL_CENTER_OFFSET_Y = -0.00062
PEDAL_CENTER_OFFSET_Z = 0.2101

RADIUS_BACK_TIRE = 0.294398  # in m
RADIUS_GEAR_CLUSTER = 0.06  # in m
RADIUS_FRONT_CHAIN_RING = 0.075

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


## Documentation for a function
#
#  Return a Twist-msg with the linear velocity #velocity
def get_twist(velocity):
    twist = Twist()
    twist.linear.x = velocity
    return twist


## Documentation for a function
#
#  Return the position of the left foot of Roboy.
def get_position_left_foot():
    fk_joint_names_list = [ ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT ]
    fk_joint_positions = [ joint_status_data[ LEFT_HIP_JOINT ][ "Pos" ], joint_status_data[ LEFT_KNEE_JOINT ][ "Pos" ],
                         joint_status_data[ LEFT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_left_tip", "foot_left_tip", fk_joint_names_list, fk_joint_positions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    print("ERROR fk foot_left failed")
    return [ 0.0, 0.0 ]  # [x, z]


## Documentation for a function
#
#  Return the position of the right foot of Roboy.
def get_position_right_foot():
    fk_joint_names_list = [ ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT ]
    fk_joint_positions = [ joint_status_data[ RIGHT_HIP_JOINT ][ "Pos" ],
                           joint_status_data[ RIGHT_KNEE_JOINT ][ "Pos" ],
                           joint_status_data[ RIGHT_ANKLE_JOINT ][ "Pos" ] ]

    rospy.wait_for_service('fk')
    try:
        fk_srv = rospy.ServiceProxy('fk', ForwardKinematics)
        fk_result = fk_srv("foot_right_tip", "foot_right_tip", fk_joint_names_list, fk_joint_positions)
        return [ fk_result.pose.position.x, fk_result.pose.position.z ]

    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

    print("ERROR fk foot_right failed")
    return [ 0.0, 0.0 ]  # [x, z]


## Documentation for a function
#
#  Evaluating the current pedal-angle according to the current position of the left foot @current_point.
#  Using trigonometric functions for the evaluation of the angle.
def evaluate_current_pedal_angle(current_point):
    current_x = current_point[ 0 ] - PEDAL_CENTER_OFFSET_X
    current_y = current_point[ 1 ] - PEDAL_CENTER_OFFSET_Y

    if current_x > 0 and current_y > 0:
        return np.arctan(current_y / current_x)
    elif current_x < 0 and current_y > 0:
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
#  Returns the absolute difference of two angles within the interval [0;2pi]
def get_angle_difference(angle_1, angle_2):
    return np.pi - np.abs(np.abs(angle_1 - angle_2) - np.pi)


## Documentation for a function
#
#  Evaluate the angle-error for velocity @velocity after @TIME_STEP_SIMULATION seconds
def evaluate_error(velocity, leg):

    current_angle = 0
    next_angle = 0
    if leg == "right":
        current_angle = evaluate_current_pedal_angle(get_position_right_foot())
        time.sleep(TIME_STEP_SIMULATION)
        next_angle = evaluate_current_pedal_angle(get_position_right_foot())
    elif leg == "left":
        current_angle = evaluate_current_pedal_angle(get_position_left_foot())
        time.sleep(TIME_STEP_SIMULATION)
        next_angle = evaluate_current_pedal_angle(get_position_left_foot())

    circulation_time = 2 * np.pi * (RADIUS_FRONT_CHAIN_RING / RADIUS_GEAR_CLUSTER /
                                                          (velocity / RADIUS_BACK_TIRE))
    target_angle = (current_angle + (np.pi * 2 / circulation_time * TIME_STEP_SIMULATION)) % (2 * np.pi)

    return get_angle_difference(target_angle, next_angle)


## Documentation for a function
#
#  Test program for evaluating the accuracy of velocity.
#  Saves ten error-results for every target velocity and determines average error and max error for every velocity
#  and displays them using pyplot.
def simulation_test(pub):

    error_results_right = [[]] * (MAX_VELOCITY / VELOCITY_STEP_SIMULATION)
    error_results_left = [[]] * (MAX_VELOCITY / VELOCITY_STEP_SIMULATION)

    for i in range(1, MAX_VELOCITY, VELOCITY_STEP_SIMULATION):
        pub.publish(get_twist(i))
        for k in range(10):
            error_results_right[i].append(evaluate_error(i, "right"))
            error_results_left[i].append(evaluate_error(i, "left"))

    avg_error_left = []
    avg_error_right = []
    max_error_left = []
    max_error_right = []
    for velocity in error_results_right:
        avg_error_right.append(sum(velocity) / len(velocity))
        max_error_right.append(max(velocity))
    for velocity in error_results_left:
        avg_error_left.append(sum(velocity) / len(velocity))
        max_error_left.append(max(velocity))

    plt.plot(range(1, MAX_VELOCITY, VELOCITY_STEP_SIMULATION), avg_error_left, label="average error left")
    plt.plot(range(1, MAX_VELOCITY, VELOCITY_STEP_SIMULATION), avg_error_right, label="average error right")
    plt.plot(range(1, MAX_VELOCITY, VELOCITY_STEP_SIMULATION), max_error_left, label="max error left")
    plt.plot(range(1, MAX_VELOCITY, VELOCITY_STEP_SIMULATION), max_error_right, label="max error right")
    plt.ylabel("error value")
    plt.xlabel("velocity")
    plt.show()


## Documentation for a function
#
#  Returns if @velocity has been reached within the error-tolerance
def velocity_reached(velocity):
    current_angle = evaluate_current_pedal_angle(get_position_right_foot())
    time.sleep(TIME_STEP_REALITY)
    next_angle = evaluate_current_pedal_angle(get_position_right_foot())

    circulation_time = 2 * np.pi * (RADIUS_FRONT_CHAIN_RING / RADIUS_GEAR_CLUSTER /
                                    (velocity / RADIUS_BACK_TIRE))
    target_angle = (current_angle + (np.pi * 2 / circulation_time * TIME_STEP_REALITY)) % (2 * np.pi)

    angle_error = get_angle_difference(next_angle, target_angle)

    return angle_error < ERROR_TOLERANCE_REALITY


## Documentation for a function
#
#  Test program for evaluating the acceleration time (time needed to change between two velocities.
#
#  Uses @VELOCITY_STEPS_REALITY to determine the difference between two different velocities and measures the time
#  needed to change between the two velocities. Displays all acceleration times using pyplot.
def reality_test_acceleration(pub):
    acceleration_times = [[]] * len(VELOCITY_STEPS_REALITY)

    for j in range(len(VELOCITY_STEPS_REALITY)):
        for i in range(1, MAX_VELOCITY, VELOCITY_STEPS_REALITY[i]):
            start_time = rospy.get_rostime()
            pub.publish(get_twist(i))
            while not velocity_reached(i):
                pass
            end_time = rospy.get_rostime()
            acceleration_time = rospy.Duration(end_time-start_time).to_sec()
            acceleration_times[j].append(acceleration_time)

    for i in range(len(VELOCITY_STEPS_REALITY)):
        plt.plot(range(1, MAX_VELOCITY, VELOCITY_STEPS_REALITY[i]), acceleration_times[i],
                 label="velocity steps = "+str(VELOCITY_STEPS_REALITY))

    plt.xlabel("velocity")
    plt.ylabel("acceleration time in s")
    plt.show()


## Documentation for a function
#
#  Initializes the Test-Node for the velocity-tests
def main():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('velocity_publisher', anonymous=True)
    simulation_test(pub)
    reality_test_acceleration(pub)
    return 1


if __name__ == '__main__':
    main()

