import rospy
import numpy as np
import std_msgs.msg
from sensor_msgs.msg import JointState

rospy.init_node('extjs_remap', anonymous=True)

extjs_pub = rospy.Publisher('/external', JointState)

msg = JointState()

def extjs_cb(data):

    msg = data
    msg.header = std_msgs.msg.Header()
    msg.header.stamp = rospy.Time.now()

    extjs_pub.publish(msg)


rospy.Subscriber('/roboy/pinky/external_joint_states', JointState, extjs_cb)
rospy.spin()