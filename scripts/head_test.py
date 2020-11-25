import rospy
from sensor_msgs.msg import JointState
import time
import math

rospy.init_node("head_test")
pub = rospy.Publisher("/joint_targets", JointState, queue_size=1)
msg = JointState()
msg.name = ["head_axis1"]
msg.velocity = [0]
msg.effort = [0]
posRange = [-1,1]
maxv = 1.5
msg.position = [posRange[0]]
rate = rospy.Rate(5)
direction = 1
while not rospy.is_shutdown():
	if abs(msg.position[0]) > maxv:
		direction = -direction
	msg.position[0] += direction*0.01
	pub.publish(msg)
	time.sleep(0.01)
