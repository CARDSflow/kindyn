#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Tendon

rospy.init_node('joint_state_kalman_filter', anonymous=True)

last_l = None
last_l_ext = None

tendon_error_pub = rospy.Publisher('/tendon_error', Tendon)
tendon_ext_error_pub = rospy.Publisher('/tendon_ext_error', Tendon)

tendon_msg = Tendon()
tendon_ext_msg = Tendon()


def tendon_state_cb(data):
    global last_l
    if last_l is None:
        last_l = []
        for l in data.l:
            last_l.append(l)
        last_l = np.array(last_l)
    else:
        delta_l = np.array(data.l) - last_l
        tendon_msg.l = delta_l
        tendon_error_pub.publish(tendon_msg)
        last_l = np.array(data.l)


def tendon_state_ext_cb(data):
    global last_l_ext
    if last_l_ext is None:
        last_l_ext = []
        for l in data.l:
            last_l_ext.append(l)
        last_l_ext = np.array(last_l_ext)
    else:
        delta_l_ext = np.array(data.l) - last_l_ext
        tendon_ext_msg.l = delta_l_ext
        tendon_ext_error_pub.publish(tendon_ext_msg)
        last_l_ext = np.array(data.l)


rospy.Subscriber('/roboy/pinky/control/tendon_state', Tendon, tendon_state_cb)
rospy.Subscriber('/roboy/pinky/control/tendon_state_ext', Tendon, tendon_state_ext_cb)
rospy.spin()
