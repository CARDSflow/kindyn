#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np

rospy.init_node("wrist_test")
topic_root = '/roboy/pinky/'
publisher = rospy.Publisher("/roboy/pinky/control/joint_targets", JointState, queue_size=1)
rate = rospy.Rate(100)
joints_name = ['shoulder_left_axis0', 'shoulder_left_axis1', 'shoulder_left_axis2',
               'elbow_left_axis0', 'elbow_left_axis1',
               'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2',
               'head_axis0', 'head_axis1', 'head_axis2',
               'shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2',
               'elbow_right_axis0', 'elbow_right_axis1',
               'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2']


def hard_reset():
    msg = JointState()

    msg.name = joints_name
    msg.position = [0] * len(msg.name)
    msg.velocity = [0] * len(msg.name)
    msg.effort = [0] * len(msg.name)
    publisher.publish(msg)

    rospy.loginfo("Hard reset")
    rospy.sleep(2.0)


def hand_shake(left_or_right="right", ranges=None):
    if ranges is None:
        ranges = [0.6, 1.0]

    msg = JointState()
    msg.name = ["elbow_" + left_or_right + "_axis0"]
    msg.velocity = [0] * len(msg.name)
    msg.effort = [0] * len(msg.name)

    samples = 50
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


def move_shoulder_elbow_wrist(left_or_right="right", move_elbow=True, move_wrist=True,
                              shoulder_range=None, elbow_range=None, wrist_range=None):
    shoulder_range = [0.0, -1.1] if not shoulder_range else shoulder_range
    elbow_range = [0.0, 0.6] if not elbow_range else elbow_range
    wrist_range = [-1.5, -0.05] if not wrist_range else wrist_range

    msg = JointState()
    num_samples = 200

    msg.name = ["shoulder_" + left_or_right + "_axis0"]
    targets = [np.linspace(shoulder_range[0], shoulder_range[1], num_samples)]
    if move_elbow:
        msg.name.append("elbow_" + left_or_right + "_axis0")
        targets.append(np.linspace(elbow_range[0], elbow_range[1], num_samples))
    if move_wrist:
        msg.name.append("wrist_" + left_or_right + "_axis0")
        targets.append(np.linspace(wrist_range[0], wrist_range[1], num_samples))

    targets = np.array(targets)

    for i in range(num_samples):
        msg.position = list(targets[:, i])
        msg.velocity = [0] * len(msg.name)
        msg.effort = [0] * len(msg.name)
        publisher.publish(msg)
        rate.sleep()


def move_wrist(left_or_right="right", axis=0, ranges=None):
    ranges = [-1.5, -0.05] if not ranges else ranges

    msg = JointState()
    samples = 200
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


def move_all(move_type="sequentially", shoulder_range=None, elbow_range=None, wrist_range=None, head_range=None):
    shoulder_range = [0.0, -1.1] if not shoulder_range else shoulder_range
    elbow_range = [0.0, 0.6] if not elbow_range else elbow_range
    wrist_range = [-1.5, -0.05] if not wrist_range else wrist_range
    head_range = [-0.56, 0.56] if not head_range else head_range

    msg = JointState()
    msg.name = ["shoulder_left_axis0", "elbow_left_axis0", "wrist_left_axis0", "wrist_left_axis1",
                "shoulder_right_axis0", "elbow_right_axis0", "wrist_right_axis0", "wrist_right_axis1",
                "head_axis0"]

    if move_type == "sequentially":
        targets = np.linspace(shoulder_range[0], shoulder_range[1], 200)
        targets_el = np.linspace(elbow_range[0], elbow_range[1], 200)
        targets_wr = np.linspace(wrist_range[0], wrist_range[1], 200)
        targets_head = np.linspace(head_range[0], head_range[1], 200)
    elif move_type == "randomly":
        targets = np.random.uniform(shoulder_range[0], shoulder_range[1], 200)
        targets_el = np.random.uniform(elbow_range[0], elbow_range[1], 200)
        targets_wr = np.random.uniform(wrist_range[0], wrist_range[1], 200)
        targets_head = np.random.uniform(head_range[0], head_range[1], 200)
    elif move_type == "randomly_in_range":
        shoulder_range = [np.random.uniform(shoulder_range[0], shoulder_range[1] / 4),
                          np.random.uniform(shoulder_range[0] / 2, shoulder_range[1])]
        elbow_range = [np.random.uniform(elbow_range[0], elbow_range[1] / 4),
                       np.random.uniform(elbow_range[0] / 2, elbow_range[1])]
        wrist_range = [np.random.uniform(wrist_range[0], wrist_range[1] / 4),
                       np.random.uniform(wrist_range[0] / 2, wrist_range[1])]
        head_range = [np.random.uniform(head_range[0], head_range[1] / 4),
                      np.random.uniform(head_range[0] / 2, head_range[1])]
        targets = np.linspace(shoulder_range[0], shoulder_range[1], 200)
        targets_el = np.linspace(elbow_range[0], elbow_range[1], 200)
        targets_wr = np.linspace(wrist_range[0], wrist_range[1], 200)
        targets_head = np.linspace(head_range[0], head_range[1], 200)

    for t, te, tw, th in zip(targets, targets_el, targets_wr, targets_head):
        msg.position = [t, te, tw, tw, t, te, tw, tw, th]
        msg.velocity = [0] * len(msg.name)
        msg.effort = [0] * len(msg.name)
        publisher.publish(msg)
        rate.sleep()


def test_scene_1():
    rospy.loginfo("########## SCENE 1 ##########")
    rospy.loginfo("Description: Only move the wrist")
    hard_reset()
    rospy.loginfo("Move left wrist")
    move_wrist("left", 2)
    rospy.loginfo("Move right wrist")
    move_wrist("right", 2)


def test_scene_2():
    rospy.loginfo("########## SCENE 2 ##########")
    rospy.loginfo("Description: Move shoulder and elbow then the wrist")
    hard_reset()

    rospy.loginfo("Move left shoulder and elbow")
    move_shoulder_elbow_wrist("left", move_wrist=False)
    rospy.loginfo("Move left wrist")
    move_wrist("left", 2)

    rospy.sleep(5.0)

    rospy.loginfo("Move right shoulder and elbow")
    move_shoulder_elbow_wrist("right", move_wrist=False)
    rospy.loginfo("Move right wrist")
    move_wrist("right", 2)


def test_scene_3():
    rospy.loginfo("########## SCENE 3 ##########")
    rospy.loginfo("Description: Move shoulder, elbow and wrist altogether and do a handshake")
    hard_reset()
    rospy.loginfo("Move left shoulder, elbow and wrist")
    move_shoulder_elbow_wrist("left")
    rospy.loginfo("Do left handshake")
    hand_shake("left")

    rospy.sleep(5.0)

    rospy.loginfo("Move right shoulder, elbow and wrist")
    move_shoulder_elbow_wrist("right")
    rospy.loginfo("Do right handshake")
    hand_shake("right")


def test_scene_4():
    rospy.loginfo("########## SCENE 4 ##########")
    rospy.loginfo("Move all sequentially and randomly")
    hard_reset()
    rospy.loginfo("Move all sequentially")
    move_all(move_type="sequentially")

    rospy.sleep(5.0)

    hard_reset()
    rospy.loginfo("Move all with random range")
    move_all(move_type="randomly_in_range")

    rospy.sleep(5.0)

    hard_reset()
    rospy.loginfo("Move all randomly")
    move_all(move_type="randomly")


if __name__ == '__main__':

    while publisher.get_num_connections() == 0:
        rospy.loginfo("Waiting for a connection")
    rospy.sleep(1)

    # Create scenarios and test
    test_scene_1()
    rospy.sleep(5.0)
    test_scene_2()
    rospy.sleep(5.0)
    test_scene_3()
    rospy.sleep(5.0)
    test_scene_4()
