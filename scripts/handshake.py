import rospy
from sensor_msgs.msg import JointState
from roboy_middleware_msgs.msg import MotorCommand
import numpy as np
from std_srvs.srv import Trigger
from std_msgs.msg import Empty

rospy.init_node("handshake")
topic_root = 'roboy/pinky/'
publisher = rospy.Publisher(topic_root + "joint_targets", JointState, queue_size=1)
msg = JointState()
hand_msg = MotorCommand()
hand_msg.global_id = [42,42,44,45]
motorcmd_pub = rospy.Publisher(topic_root + "middleware/MotorCommand", MotorCommand, queue_size=1)
rate = rospy.Rate(100)

def close_hand():
    hand_msg.setpoint = [900]*len(hand_msg.global_id)
    motorcmd_pub.publish(hand_msg)

def open_hand():
    hand_msg.setpoint = [0]*len(hand_msg.global_id)
    motorcmd_pub.publish(hand_msg)


def reachout():
    # msg.name = ["shoulder_right_axis1"]#,"elbow_right_axis1"]
    # targets = np.linspace(0.0,-1.1, 10)
    # for t in targets:
    #     msg.position = [t]
    #     msg.velocity = [0]
    #     msg.effort = [0]
    #     publisher.publish(msg)
    #     rospy.sleep(0.1)

    # msg.position = [-1.1]#,-0.3]
    # msg.velocity = [0,0]#,0]
    # msg.effort = [0,0]#,0]
    # publisher.publish(msg)
    # rospy.sleep(1)
    # msg.name = ["shoulder_right_axis0"]
    # msg.position = [-0.8]
    # msg.velocity = [0]
    # msg.effort = [0]
    # publisher.publish(msg)

    # rospy.sleep(1)
    msg.name = ["shoulder_right_axis0", "elbow_right_axis0"]#,"shoulder_right_axis1"]
    targets = np.linspace(0.0,-1.1, 200)
    targets_el = np.linspace(0, 0.6, 200)
    for t,te in zip(targets,targets_el):
        msg.position = [t,te]#,t/3.0]
        msg.velocity = [0]*len(msg.name)
        msg.effort = [0]*len(msg.name)
        publisher.publish(msg)
        rate.sleep()
    # msg.position = [-1.0]
    # msg.velocity = [0]
    # msg.effort = [0]
    # publisher.publish(msg)
    # rospy.sleep(0.5)

def goback():
    msg.name = ["shoulder_right_axis0"]#, "shoulder_right_axis0"]#, "elbow_right_axis1"]
    axis1_targets = np.linspace(-1.1,0, 150)

    for t in axis1_targets:
        msg.position = [t]#,t/3.0]
        msg.velocity = [0]*len(msg.name)
        msg.effort = [0]*len(msg.name)
        publisher.publish(msg)
        rate.sleep()
        # rospy.sleep(0.1)

    msg.name = ["shoulder_right_axis1", "shoulder_right_axis0"]#, "elbow_right_axis1"]

    msg.position = [0]*len(msg.name)
    msg.velocity = [0]*len(msg.name)
    msg.effort = [0]*len(msg.name)
    # publisher.publish(msg)

def raw_motorcmd():
    msg = MotorCommand()
    msg.global_id = [19]
    ranges = [-0.22,-0.16]
    samples = 100
    for i in range(4):
        if not i%2:
            targets = np.linspace(ranges[-1],ranges[0], samples)
            for t2 in targets:
                msg.setpoint = [t2]
                motorcmd_pub.publish(msg)
                rospy.sleep(0.07)
        else:
            targets = np.linspace(ranges[0],ranges[-1], samples)
            for t2 in targets:
                msg.setpoint = [t2]
                motorcmd_pub.publish(msg)
                rospy.sleep(0.07)

def shake():
    msg.name = ["elbow_right_axis0"]
    msg.velocity = [0]*len(msg.name)
    msg.effort = [0]*len(msg.name)
    ranges = [0.6, 1.0]
    samples = 50
    for i in range(10):
        if not i%2:
            print("up")
            msg.name = [ "elbow_right_axis0"]
            # msg.position = [1.1]
            # msg.velocity = [0]*len(msg.name)
            # msg.effort = [0]*len(msg.name)

            el_axis1_targets = np.linspace(ranges[0], ranges[-1], samples)

            for t2 in el_axis1_targets:
                msg.name = [ "elbow_right_axis0"]
                msg.position = [t2]
                # print(t2)
                msg.velocity = [0]
                msg.effort = [0]
                publisher.publish(msg)
                rate.sleep()
                # rospy.sleep(0.15)
            print(msg)
            # rospy.sleep(1.0)
            # input()
        else:
            print("down")
            sh_axis1_targets = np.linspace(-1.0,-1.0, 5)
            el_axis1_targets = np.linspace(ranges[-1], ranges[0], samples)

            for t2 in el_axis1_targets:
                msg.name = [ "elbow_right_axis0"]
                msg.position = [t2]
                # print(t2)
                msg.velocity = [0]
                msg.effort = [0]
                publisher.publish(msg)
                rate.sleep()
                # rospy.sleep(0.1)
            # rospy.sleep(2.0)
            # msg.position = [-0.3]

def shoulder():
    ranges = [-1.5,-0.05]
    samples = 200
    msg.name = ["shoulder_right_axis1"]
    msg.velocity = [0]*len(msg.name)
    msg.effort = [0]*len(msg.name)
    for i in range(6):
        if not i%2:
            print("up")
            targets = np.linspace(ranges[-1],ranges[0], samples)

            for t2 in targets:
                msg.position = [t2]
                publisher.publish(msg)
                rospy.sleep(0.01)

        else:
            print("down")
            targets = np.linspace(ranges[0],ranges[-1], samples)

            for t2 in targets:
                msg.position = [t2]
                publisher.publish(msg)
                rospy.sleep(0.01)
            # rospy.sleep(2.0)
            # msg.position = [-0.3]

def head(axis, ranges):
    # ranges = [-0.56,0.56]
    samples = 100
    msg.name = [axis]
    msg.velocity = [0]*len(msg.name)
    msg.effort = [0]*len(msg.name)


    targets = np.linspace(0.0,ranges[-1], samples)

    for t2 in targets:
        msg.position = [t2]
        publisher.publish(msg)
        rospy.sleep(0.01)


    for i in range(6):
        if not i%2:
            print("up")
            targets = np.linspace(ranges[-1],ranges[0], samples)

            for t2 in targets:
                msg.position = [t2]
                publisher.publish(msg)
                rospy.sleep(0.01)

        else:
            print("down")
            targets = np.linspace(ranges[0],ranges[-1], samples)

            for t2 in targets:
                msg.position = [t2]
                publisher.publish(msg)
                rospy.sleep(0.01)
            # rospy.sleep(2.0)
            # msg.position = [-0.3]

    targets = np.linspace(ranges[-1],0.0, samples)

    for t2 in targets:
        msg.position = [t2]
        publisher.publish(msg)
        rospy.sleep(0.01)

def do_handshake():
    rospy.set_param("trajectory_active", True)

    rospy.loginfo("reach")
    reachout()
    rospy.sleep(1.0)
    close_hand()
    rospy.sleep(1.0)

    rospy.loginfo("shake")
    shake()
    # rospy.sleep(1)
    open_hand()
    rospy.sleep(2)
    rospy.loginfo("back")
    goback()

    rospy.set_param("trajectory_active", False)

def cb1(req):
    do_handshake()
    return True, "finished handshake"

def cb2(req):
    do_handshake()

rospy.sleep(1)
close_hand()
rospy.sleep(2.0)
open_hand()

# handshake_srv = rospy.Service(topic_root + "control/handshake", Trigger, cb1)
# handshake_sub = rospy.Subscriber(topic_root + "control/handshake", Empty, cb2)
#
# while not rospy.is_shutdown():
#     rospy.spin()
# raw_motorcmd()
# shoulder()
# shake()
# head("head_axis1",[-0.56,0.56])
# head("head_axis0",[-0.4,0.4])
# head("head_axis2",[-0.5,0.5])
#joint_cmd = [{"shoulder_right_axis1": -0.4}, {"elbow_right_axis0": 0.3, "elbow_right_axis0": 0.0}]
