#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math
import time

rospy.init_node('humanoid_robot_arms_controller', anonymous=False)
joint_target_pub = rospy.Publisher(
    '/roboy/pinky/orchestrated/joint_targets', JointState, queue_size=1)

rate = rospy.Rate(60)  # 10 Hz

joint_state = JointState()

def zero_joints():
    joint_state.name = ['shoulder_left_axis0', 'shoulder_right_axis0', 'shoulder_left_axis1', 'shoulder_right_axis1', 'elbow_left_axis0', 'elbow_right_axis0']
    joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_state.header.stamp = rospy.Time.now()
    joint_target_pub.publish(joint_state)


def move_arms_in_out():

    joint_state.name = ['shoulder_left_axis0', 'shoulder_right_axis0', 'shoulder_left_axis1', 'shoulder_right_axis1', 'elbow_left_axis0', 'elbow_right_axis0']
    
    shoulder_left_min = -1.2
    shoulder_left_max = 0.0
    shoulder_right_min = -1.2
    shoulder_right_max = 0.0
    elbow_left_min = 0.0
    elbow_left_max = 0.9
    elbow_right_min = 0.0
    elbow_right_max = 0.9

    shoulder_axis0_position = -0.3

    for pos in range(300,-300, -1):
        t = pos / 600.0
        joint_state.header.stamp = rospy.Time.now()

        
        
        shoulder_axis1_position = shoulder_left_min + (shoulder_left_max - shoulder_left_min) * (math.sin(math.pi * t) + 1) / 2
        elbow_position = elbow_left_min + (elbow_left_max - elbow_left_min) * (1 - (shoulder_axis1_position - shoulder_left_min) / (shoulder_left_max - shoulder_left_min))

        joint_state.position = [shoulder_axis0_position, shoulder_axis0_position, shoulder_axis1_position, shoulder_axis1_position, elbow_position, elbow_position]

        joint_target_pub.publish(joint_state)
        rate.sleep()
        if rospy.is_shutdown():
            break

    for pos in range(-300,300, 1):
        t = pos / 600.0
        joint_state.header.stamp = rospy.Time.now()

        
        shoulder_axis1_position = shoulder_left_min + (shoulder_left_max - shoulder_left_min) * (math.sin(math.pi * t) + 1) / 2
        elbow_position = elbow_left_min + (elbow_left_max - elbow_left_min) * (1 - (shoulder_axis1_position - shoulder_left_min) / (shoulder_left_max - shoulder_left_min))

        joint_state.position = [shoulder_axis0_position, shoulder_axis0_position, shoulder_axis1_position, shoulder_axis1_position, elbow_position, elbow_position]

        joint_target_pub.publish(joint_state)
        rate.sleep()
        if rospy.is_shutdown():
            break

def move_arms_up_down():

    joint_state.name = ['shoulder_left_axis0',
                    'shoulder_right_axis0', 'elbow_left_axis0', 'elbow_right_axis0']

    shoulder_left_min = -1.0
    shoulder_left_max = -0.4
    shoulder_right_min = -1.0
    shoulder_right_max = -0.4
    elbow_left_min = 0.2
    elbow_left_max = 0.5
    elbow_right_min = 0.2
    elbow_right_max = 0.7

    for pos in range(-300, 300, 1):
        t = pos / 300.0
        joint_state.header.stamp = rospy.Time.now()

        shoulder_left_position = shoulder_left_min + \
            (shoulder_left_max - shoulder_left_min) * \
            (math.sin(math.pi * t) + 1) / 2
        shoulder_right_position = shoulder_right_max - \
            (shoulder_right_max - shoulder_right_min) * \
            (math.sin(math.pi * t) + 1) / 2

        elbow_left_position = elbow_left_min + (elbow_left_max - elbow_left_min) * (1 - (
            shoulder_left_position - shoulder_left_min) / (shoulder_left_max - shoulder_left_min))
        elbow_right_position = elbow_right_min + (elbow_right_max - elbow_right_min) * (1 - (
            shoulder_right_position - shoulder_right_min) / (shoulder_right_max - shoulder_right_min))

        joint_state.position = [
            shoulder_left_position, shoulder_right_position, elbow_left_position, elbow_right_position]

        joint_target_pub.publish(joint_state)
        rate.sleep()
        if rospy.is_shutdown():
            break


def main():
    # zero_joints()
    
    rospy.set_param("orchestrated_arms_active", True)
    for i in range(4):
        # move_arms_up_down()
        move_arms_in_out()
    rospy.set_param("orchestrated_arms_active", False)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.set_param("orchestrated_arms_active", False)
        zero_joints()
        pass
