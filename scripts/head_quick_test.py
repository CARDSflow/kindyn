#!/usr/bin/env python

import rospy
import argparse
from argparse import RawTextHelpFormatter
import numpy as np
from sensor_msgs.msg import JointState
from enum import Enum


class MoveArm(str, Enum):
    LEFT = "left"
    RIGHT = "right"

    def __str__(self):
        return self.value


rospy.init_node("shoulder_test")
topic_root = '/roboy/pinky/'
publisher = rospy.Publisher(topic_root + "/control/joint_targets", JointState, queue_size=1)
rate = rospy.Rate(100)
joints_name = ['shoulder_left_axis0', 'shoulder_left_axis1', 'shoulder_left_axis2',
               'elbow_left_axis0', 'elbow_left_axis1',
               'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2',
               'head_axis0', 'head_axis1', 'head_axis2',
               'shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2',
               'elbow_right_axis0', 'elbow_right_axis1',
               'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2']
is_simulated = rospy.get_param("/simulated")


def hard_reset():
    msg = JointState()

    msg.name = joints_name
    msg.position = [0] * len(msg.name)
    msg.velocity = [0] * len(msg.name)
    msg.effort = [0] * len(msg.name)
    publisher.publish(msg)

    rospy.loginfo("Hard reset")
    rospy.sleep(2.0)


def move_head(axis=0, shoulder_range=None, samples=200):

    msg = JointState()

    # msg.name = ["shoulder_" + left_or_right + "_axis" + str(axis)]
    # targets = [np.linspace(shoulder_range[0], shoulder_range[1], samples)]

    msg.name = ["head_axis"+str(axis)]
    targets = [np.linspace(shoulder_range[0], shoulder_range[1], samples)]

    targets = np.array(targets)

    for i in range(samples):
        msg.position = list(targets[:, i])
        msg.velocity = [0] * len(msg.name)
        msg.effort = [0] * len(msg.name)
        publisher.publish(msg)
        rate.sleep()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Test wrist, use python wrist_test.py -h for more information',
                                     formatter_class=RawTextHelpFormatter)
    parser.add_argument('--manual-test', action='store_true',
                        help='Run manual test')
    parser.add_argument('--automatic-test', dest='manual-test', action='store_false',
                        help='Run automatic test')
    parser.add_argument('--axis', type=int, choices=[0, 1, 2],
                        help='axis', default=0)
    parser.add_argument('--ranges', type=float,
                        help='Move ranges: e.g -0.5, 0.5', nargs='+')
    parser.add_argument('--samples', type=int,
                        help='Number of samples', default=300)
    parser.add_argument('--scene', type=int, choices=[1, 2, 3, 4],
                        help='1: Only move the wrist \n'
                             '2: Move shoulder and elbow then the wrist \n'
                             '3: Move shoulder, elbow and wrist altogether and do a handshake \n'
                             '4: Move all sequentially and randomly')
    args = parser.parse_args()

    while publisher.get_num_connections() == 0:
        rospy.loginfo("Waiting for a connection")
        rospy.sleep(1)

    rospy.loginfo("Move forward")
    if args.manual_test:
        for ax in range(3):
            move_head(ax, args.ranges, args.samples)

    rospy.loginfo("Move backward")
    rospy.sleep(5.0)
    move_head(args.axis, list(reversed(args.ranges)), args.samples)
    rospy.sleep(1.0)

    # hard_reset()
    # rospy.sleep(1.0)
