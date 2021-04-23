#!/usr/bin/env python

import rospy
import argparse
from argparse import RawTextHelpFormatter
import numpy as np
from sensor_msgs.msg import JointState
from enum import Enum


class MoveStyle(Enum):
    SEQUENTIALLY = 1
    RANDOMLY = 2
    RANDOMLY_IN_RANGE = 3


class MoveArm(str, Enum):
    LEFT = "left"
    RIGHT = "right"

    def __str__(self):
        return self.value


rospy.init_node("wrist_test")
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


def hand_shake(left_or_right, ranges, samples=50):
    msg = JointState()
    msg.name = ["elbow_" + left_or_right + "_axis0"]
    msg.velocity = [0] * len(msg.name)
    msg.effort = [0] * len(msg.name)

    for i in range(10):
        if not i % 2:
            msg.name = ["elbow_" + left_or_right + "_axis0"]
            el_axis1_targets = np.linspace(ranges[0], ranges[-1], samples)

            for t2 in el_axis1_targets:
                msg.name = ["elbow_" + left_or_right + "_axis0"]
                msg.position = [t2]
                msg.velocity = [0]
                msg.effort = [0]
                publisher.publish(msg)
                rate.sleep()
        else:

            el_axis1_targets = np.linspace(ranges[-1], ranges[0], samples)

            for t2 in el_axis1_targets:
                msg.name = ["elbow_" + left_or_right + "_axis0"]
                msg.position = [t2]
                msg.velocity = [0]
                msg.effort = [0]
                publisher.publish(msg)
                rate.sleep()


def move_shoulder_elbow_wrist(left_or_right, move_elbow=True, move_wrist=True,
                              shoulder_range=None, elbow_range=None, wrist_range=None, samples=200):

    msg = JointState()

    msg.name = ["shoulder_" + left_or_right + "_axis0"]
    targets = [np.linspace(shoulder_range[0], shoulder_range[1], samples)]
    if move_elbow:
        msg.name.append("elbow_" + left_or_right + "_axis0")
        targets.append(np.linspace(elbow_range[0], elbow_range[1], samples))
    if move_wrist:
        msg.name.append("wrist_" + left_or_right + "_axis0")
        targets.append(np.linspace(wrist_range[0], wrist_range[1], samples))

    targets = np.array(targets)

    for i in range(samples):
        msg.position = list(targets[:, i])
        msg.velocity = [0] * len(msg.name)
        msg.effort = [0] * len(msg.name)
        publisher.publish(msg)
        rate.sleep()


def move_wrist(left_or_right, axis, ranges, samples=200):
    msg = JointState()
    msg.name = ["wrist_" + left_or_right + "_axis" + str(axis), "wrist_" + left_or_right + "_axis0"]
    msg.velocity = [0] * len(msg.name)
    msg.effort = [0] * len(msg.name)

    for i in range(6):

        if not i % 2:
            targets = np.linspace(ranges[-1], ranges[0], samples)

            for t in targets:
                msg.position = [t, t]
                publisher.publish(msg)
                rospy.sleep(0.01)

        else:
            targets = np.linspace(ranges[0], ranges[-1], samples)

            for t in targets:
                msg.position = [t, t]
                publisher.publish(msg)
                rospy.sleep(0.01)


def move_all(move_style=MoveStyle.SEQUENTIALLY,
             shoulder_range=None, shoulder_samples=200,
             elbow_range=None, elbow_samples=200,
             wrist_range=None, wrist_samples=200,
             head_range=None, head_samples=200):
    msg = JointState()
    msg.name = ["shoulder_left_axis0", "elbow_left_axis0", "wrist_left_axis0", "wrist_left_axis1",
                "shoulder_right_axis0", "elbow_right_axis0", "wrist_right_axis0", "wrist_right_axis1",
                "head_axis0"]

    if move_style == MoveStyle.SEQUENTIALLY:
        targets = np.linspace(shoulder_range[0], shoulder_range[1], shoulder_samples)
        targets_el = np.linspace(elbow_range[0], elbow_range[1], elbow_samples)
        targets_wr = np.linspace(wrist_range[0], wrist_range[1], wrist_samples)
        targets_head = np.linspace(head_range[0], head_range[1], head_samples)
    elif move_style == MoveStyle.RANDOMLY:
        targets = np.random.uniform(shoulder_range[0], shoulder_range[1], shoulder_samples)
        targets_el = np.random.uniform(elbow_range[0], elbow_range[1], elbow_samples)
        targets_wr = np.random.uniform(wrist_range[0], wrist_range[1], wrist_samples)
        targets_head = np.random.uniform(head_range[0], head_range[1], head_samples)
    elif move_style == MoveStyle.RANDOMLY_IN_RANGE:
        shoulder_range = [np.random.uniform(shoulder_range[0], shoulder_range[1] / 4),
                          np.random.uniform(shoulder_range[0] / 2, shoulder_range[1])]
        elbow_range = [np.random.uniform(elbow_range[0], elbow_range[1] / 4),
                       np.random.uniform(elbow_range[0] / 2, elbow_range[1])]
        wrist_range = [np.random.uniform(wrist_range[0], wrist_range[1] / 4),
                       np.random.uniform(wrist_range[0] / 2, wrist_range[1])]
        head_range = [np.random.uniform(head_range[0], head_range[1] / 4),
                      np.random.uniform(head_range[0] / 2, head_range[1])]
        targets = np.linspace(shoulder_range[0], shoulder_range[1], shoulder_samples)
        targets_el = np.linspace(elbow_range[0], elbow_range[1], elbow_samples)
        targets_wr = np.linspace(wrist_range[0], wrist_range[1], wrist_samples)
        targets_head = np.linspace(head_range[0], head_range[1], head_samples)

    if move_style == MoveStyle.RANDOMLY and not is_simulated:
        rospy.logerr("Random movement on real robot is prohibited")
        return

    for t, te, tw, th in zip(targets, targets_el, targets_wr, targets_head):
        msg.position = [t, te, tw, tw, t, te, tw, tw, th]
        msg.velocity = [0] * len(msg.name)
        msg.effort = [0] * len(msg.name)
        publisher.publish(msg)
        rate.sleep()


def test_scene_1(left_or_right):
    rospy.loginfo("########## SCENE 1 ##########")
    rospy.loginfo("Description: Only move the wrist")

    hard_reset()
    rospy.loginfo("Move " + left_or_right + " wrist in axis 0")
    move_wrist(left_or_right, axis=0, ranges=[-0.5, 0.5], samples=300)

    hard_reset()
    rospy.loginfo("Move " + left_or_right + " wrist in axis 1")
    move_wrist(left_or_right, axis=1, ranges=[-0.5, 0.5], samples=300)

    hard_reset()
    rospy.loginfo("Move " + left_or_right + " wrist in axis 2")
    move_wrist(left_or_right, axis=2, ranges=[-0.5, 0.5], samples=300)


def test_scene_2(left_or_right):
    rospy.loginfo("########## SCENE 2 ##########")
    rospy.loginfo("Description: Move shoulder and elbow then the wrist")

    hard_reset()
    rospy.loginfo("Move " + left_or_right + " shoulder and elbow")
    move_shoulder_elbow_wrist(left_or_right,
                              shoulder_range=[0.0, -1.1], elbow_range=[0.0, 0.6], move_wrist=False)
    rospy.loginfo("Move " + left_or_right + " wrist")
    move_wrist(left_or_right, axis=2, ranges=[-1.5, -0.05])


def test_scene_3(left_or_right):
    rospy.loginfo("########## SCENE 3 ##########")
    rospy.loginfo("Description: Move shoulder, elbow and wrist altogether and do a handshake")

    hard_reset()
    rospy.loginfo("Move " + left_or_right + " shoulder, elbow and wrist")
    move_shoulder_elbow_wrist(left_or_right,
                              shoulder_range=[0.0, -1.1],
                              elbow_range=[0.0, 0.6],
                              wrist_range=[-1.5, -0.05])
    rospy.loginfo("Do " + left_or_right + " handshake")
    hand_shake(left_or_right, ranges=[0.6, 1.0])


def test_scene_4():
    rospy.loginfo("########## SCENE 4 ##########")
    rospy.loginfo("Move all sequentially and randomly")

    shoulder_range = [0.0, -1.1]
    elbow_range = [0.0, 0.6]
    wrist_range = [-1.5, -0.05]
    head_range = [-0.56, 0.56]

    hard_reset()
    rospy.loginfo("Move all sequentially")
    move_all(move_style=MoveStyle.SEQUENTIALLY,
             shoulder_range=shoulder_range,
             elbow_range=elbow_range,
             wrist_range=wrist_range,
             head_range=head_range)

    rospy.sleep(5.0)

    hard_reset()
    rospy.loginfo("Move all with random range")
    move_all(move_style=MoveStyle.RANDOMLY_IN_RANGE,
             shoulder_range=shoulder_range,
             elbow_range=elbow_range,
             wrist_range=wrist_range,
             head_range=head_range)

    rospy.sleep(5.0)

    if is_simulated:
        hard_reset()
        rospy.loginfo("Move all randomly")
        move_all(move_style=MoveStyle.RANDOMLY,
                 shoulder_range=shoulder_range,
                 elbow_range=elbow_range,
                 wrist_range=wrist_range,
                 head_range=head_range)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Test wrist, use python wrist_test.py -h for more information',
                                     formatter_class=RawTextHelpFormatter)
    parser.add_argument('--manual-test', action='store_true',
                        help='Run manual test')
    parser.add_argument('--automatic-test', dest='manual-test', action='store_false',
                        help='Run automatic test')
    parser.add_argument('--arm', type=MoveArm, choices=list(MoveArm),
                        help='Move arm: left or right', required=True)
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

    if args.manual_test:
        move_wrist(args.arm, args.axis, args.ranges, args.samples)
    else:
        # Test based on predefined scenarios
        if args.scene == 1:
            test_scene_1(args.arm)
        elif args.scene == 2:
            test_scene_2(args.arm)
        elif args.scene == 3:
            test_scene_3(args.arm)
        elif args.scene == 4:
            test_scene_4()

    rospy.sleep(3.0)
    hard_reset()
    rospy.sleep(1.0)
