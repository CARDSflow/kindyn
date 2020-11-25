#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
from random import random, choice
import time
import numpy as np
from roboy_control_msgs.msg import Emotion

rospy.init_node("idle_moves")
topic_root = "/roboy/pinky/"
publisher = rospy.Publisher(topic_root + 'joint_targets', JointState, queue_size=1)
emotion_pub = rospy.Publisher(topic_root + "cognition/face/emotion", Emotion, queue_size=1)
axis1_range_upper = 0.3
shoulder_range_upper = -0.4
elbow_range_upper =  0.5
head_range_upper = 0.5
sleep_range_upper = 3.0

emotions = ["shy", "kiss", "hypno", "hearts", "lucky", "lookleft", "lookright", "surprise", "smileblink", "rolling"]

msg = JointState()
msg.name = ["head_axis1","shoulder_right_axis0", "shoulder_right_axis1", "shoulder_left_axis0", "shoulder_left_axis1", "elbow_right_axis0", "elbow_left_axis0"]

#msg.name = ["shoulder_right_axis0", "shoulder_right_axis1", "shoulder_left_axis0", "shoulder_left_axis1", "elbow_right_axis0", "elbow_left_axis0"]
old_targets = None

while not rospy.is_shutdown():
    msg.position = []

    targets = [random()*head_range_upper-0.25,random()*shoulder_range_upper, random()*shoulder_range_upper, random()*axis1_range_upper, random()*shoulder_range_upper, random()*elbow_range_upper,random()*elbow_range_upper]
    #targets = [random()*shoulder_range_upper, random()*shoulder_range_upper, random()*axis1_range_upper, random()*shoulder_range_upper, random()*elbow_range_upper,random()*elbow_range_upper]
    if old_targets is None:
        old_targets = [0]*len(targets)
    interp_targets = []
    for i in range(len(targets)):
        interp_targets.append(np.linspace(old_targets[i],targets[i],100))
    for i in range(len(interp_targets[0])):
        msg.position = []
        for j in range(len(targets)):#joints
            msg.position.append(interp_targets[j][i])
        msg.velocity = [0]*len(msg.name)
        msg.effort = [0]*len(msg.name)
        publisher.publish(msg)
        rospy.loginfo(msg.position)
        rospy.sleep(0.01)
        
    #msg.velocity = [0]*len(msg.name)
    #msg.effort = [0]*len(msg.name)
    #publisher.publish(msg)
    if random() > 0.5:
        rospy.loginfo("emotion")
        emotion_msg = Emotion(emotion=choice(emotions))
        emotion_pub.publish(emotion_msg)
    rospy.sleep(random()*sleep_range_upper)
    old_targets = targets
