import rospy

import time
import math
from sensor_msgs.msg import JointState

rospy.init_node("trajectory")

joint_targets = rospy.Publisher('/joint_targets', JointState, queue_size=1)

axis0 = [0,-math.pi/2]
axis1 = [0,0]
axis2 = [0,0.5]

time = 5
sampleTime = 0.01
samples = time/sampleTime
axis0_increment = (axis0[1]-axis0[0])/samples
axis1_increment = (axis1[1]-axis1[0])/samples
axis2_increment = (axis2[1]-axis2[0])/samples

startTime = rospy.Time.now()

msg = JointState()
msg.name = ["shoulder_right_axis0","shoulder_right_axis1","shoulder_right_axis2"]
msg.position = [axis0[0],axis1[0],axis2[0]]
msg.velocity = [0,0,0]
msg.effort = [0,0,0]

rate = rospy.Rate(1/sampleTime)

while not rospy.is_shutdown() and (rospy.Time.now()-startTime).to_sec()<time:
    joint_targets.publish(msg)
    rate.sleep()
    msg.position[0] = msg.position[0] + axis0_increment
    msg.position[1] = msg.position[1] + axis1_increment
    msg.position[2] = msg.position[2] + axis2_increment
