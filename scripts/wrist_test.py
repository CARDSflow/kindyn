#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np

rospy.init_node("wrist_test")
topic_root = '/roboy/pinky/'
publisher = rospy.Publisher("/roboy/pinky/control/joint_targets", JointState, queue_size=1)
rate = rospy.Rate(100)
joints_name = ['shoulder_left_axis0','shoulder_left_axis1','shoulder_left_axis2',
               'elbow_left_axis0', 'elbow_left_axis1',
               'wrist_left_axis0', 'wrist_left_axis1', 'wrist_left_axis2',
               'head_axis0', 'head_axis1', 'head_axis2',
               'shoulder_right_axis0', 'shoulder_right_axis1', 'shoulder_right_axis2',
               'elbow_right_axis0', 'elbow_right_axis1',
               'wrist_right_axis0', 'wrist_right_axis1', 'wrist_right_axis2']

test_lr = "right"
notest_lr = "left" if test_lr == "right" else "right"


def hard_reset():
    msg = JointState()

    msg.name = joints_name
    msg.position = [0]*len(msg.name)
    msg.velocity = [0]*len(msg.name)
    msg.effort = [0]*len(msg.name)
    publisher.publish(msg)

    rospy.loginfo("Hard reset")
    rospy.sleep(2.0)


def soft_reset():
    pass


def hand_shake(lr="right"):
    msg = JointState()
    msg.name = ["elbow_"+lr+"_axis0"]
    msg.velocity = [0]*len(msg.name)
    msg.effort = [0]*len(msg.name)
    ranges = [0.6, 1.0]
    samples = 50
    for i in range(10):
        if not i % 2:
            rospy.loginfo("up")
            msg.name = [ "elbow_"+lr+"_axis0"]
            # msg.position = [1.1]
            # msg.velocity = [0]*len(msg.name)
            # msg.effort = [0]*len(msg.name)

            el_axis1_targets = np.linspace(ranges[0], ranges[-1], samples)

            for t2 in el_axis1_targets:
                msg.name = [ "elbow_"+lr+"_axis0"]
                msg.position = [t2]
                # print(t2)
                msg.velocity = [0]
                msg.effort = [0]
                publisher.publish(msg)
                rate.sleep()
                # rospy.sleep(0.15)

            # rospy.loginfo(msg)
            # rospy.sleep(1.0)
            # input()
        else:
            rospy.loginfo("down")
            sh_axis1_targets = np.linspace(-1.0,-1.0, 5)
            el_axis1_targets = np.linspace(ranges[-1], ranges[0], samples)

            for t2 in el_axis1_targets:
                msg.name = [ "elbow_"+lr+"_axis0"]
                msg.position = [t2]
                # print(t2)
                msg.velocity = [0]
                msg.effort = [0]
                publisher.publish(msg)
                rate.sleep()
                # rospy.sleep(0.1)
            # rospy.sleep(2.0)
            # msg.position = [-0.3]


def move_shoulder_elbow(lr="right", elbow=True):
    msg = JointState()

    msg.name = ["shoulder_"+lr+"_axis0"]
    if elbow:
        msg.name.append("elbow_"+lr+"_axis0")

    targets = np.linspace(0.0, -1.1, 200)
    targets_el = np.linspace(0, 0.6, 200)

    if elbow:
        for t, te in zip(targets,targets_el):
            msg.position = [t, te]
            msg.velocity = [0]*len(msg.name)
            msg.effort = [0]*len(msg.name)
            publisher.publish(msg)
            rate.sleep()
    else:
        for t in targets:
            msg.position = [t]
        msg.velocity = [0]*len(msg.name)
        msg.effort = [0]*len(msg.name)
        publisher.publish(msg)
        rate.sleep()


def move_wrist(lr="right", axis=0):
    msg = JointState()

    ranges = [-1.5, -0.05]
    samples = 200
    msg.name = ["wrist_"+lr+"_axis"+str(axis), "wrist_"+lr+"_axis0"]
    msg.velocity = [0]*len(msg.name)
    msg.effort = [0]*len(msg.name)

    out_str = lr + " " + str(axis) + " "

    for i in range(6):

        if not i % 2:
            rospy.loginfo(out_str+"up")
            targets = np.linspace(ranges[-1], ranges[0], samples)

            for t2 in targets:
                msg.position = [t2, t2]
                publisher.publish(msg)
                rospy.sleep(0.01)

        else:
            rospy.loginfo(out_str+"down")
            targets = np.linspace(ranges[0], ranges[-1], samples)

            for t2 in targets:
                msg.position = [t2, t2]
                publisher.publish(msg)
                rospy.sleep(0.01)


def move_all(lr="right", axis=0):
    msg = JointState()

    msg_name = ["shoulder_left_axis0", "elbow_left_axis0",  "wrist_left_axis0", "wrist_left_axis1",
                "shoulder_right_axis0", "elbow_right_axis0",  "wrist_right_axis0", "wrist_right_axis1",
                "head_axis0"
                ]
    # ["shoulder_"+lr+"_axis0", "elbow_"+lr+"_axis0",  "wrist_"+lr+"_axis0", "wrist_"+lr+"_axis1", "head_axis0"]
    msg.name = msg_name
    targets = np.linspace(0.0,-1.1, 200)
    targets_el = np.linspace(0, 0.6, 200)
    targets_wr = np.linspace(-1.5, -0.05, 200)
    targets_head = np.linspace(-0.56, 0.56, 200)
    for t, te, tw, th in zip(targets, targets_el, targets_wr, targets_head):
        msg.position = [t, te, tw, tw, t, te, tw, tw, th]
        msg.velocity = [0]*len(msg.name)
        msg.effort = [0]*len(msg.name)
        publisher.publish(msg)
        rate.sleep()


def test_scene_1():
    print("########## SCENE 1 ##########")
    hard_reset()
    # move_wrist(test_lr, 0)
    # move_wrist(test_lr, 1)
    move_wrist(test_lr, 2)


def test_scene_2():
    print("########## SCENE 2 ##########")
    hard_reset()
    move_all(test_lr)
    # move_wrist(test_lr, 0)
    move_wrist(test_lr, 1)
    move_wrist(test_lr, 2)


def test_scene_3():
    print("########## SCENE 3 ##########")
    hard_reset()
    move_shoulder_elbow(test_lr)
    move_shoulder_elbow(notest_lr, True)
    hand_shake(test_lr)


def test_scene_4():
    print("########## SCENE 4 ##########")
    hard_reset()
    move_shoulder_elbow(notest_lr, True)
    rospy.sleep(5.0)
    hard_reset()
    move_all(test_lr)
    hand_shake(notest_lr)
    # move_wrist(test_lr, 0)
    move_wrist(test_lr, 1)
    move_wrist(test_lr, 2)


if __name__ == '__main__':

    while publisher.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect")
    rospy.sleep(1)

    # Create scenarios and test
    # test_scene_1()
    # rospy.sleep(5.0)
    # test_scene_2()
    # rospy.sleep(5.0)
    # test_scene_3()
    # rospy.sleep(5.0)
    test_scene_4()



