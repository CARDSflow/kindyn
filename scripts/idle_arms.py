#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
from random import random
import time

rospy.init_node("idle_moves")
topic_root = "/roboy/brain/"
publisher = rospy.Publisher(topic_root + 'joint_targets', JointState, queue_size=1)

axis1_range_upper = -0.5
shoulder_range_upper = -0.5
elbow_range_upper =  1.0
sleep_range_upper = 5.0

msg = JointState()
msg.name = ["shoulder_right_axis0", "shoulder_right_axis1", "shoulder_left_axis0", "shoulder_left_axis0", "elbow_right_axis0", "elbow_left_axis0"]

while not rospy.is_shutdown():
    msg.position = [random()*shoulder_range_upper, random()*shoulder_range_upper, random()*shoulder_range_upper,random()*shoulder_range_upper, random()*elbow_range_upper,random()*elbow_range_upper]
    msg.velocity = [0]*len(msg.name)
    msg.effort = [0]*len(msg.name)
    publisher.publish(msg)
    time.sleep(random()*sleep_range_upper)