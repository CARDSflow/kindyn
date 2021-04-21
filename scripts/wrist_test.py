import rospy
from sensor_msgs.msg import JointState

rospy.init_node("wrist_test")
pub = rospy.Publisher("roboy/brain/control/joint_targets", JointState, queue_size=1)
msg = JointState()
msg.name = ["wrist_right_axis1"]
msg.position = [0.75]
while not rospy.is_shutdown():
	for i in range(5):
		msg.position = [-x for x in msg.position]
		pub.publish(msg)
		rospy.sleep(3)

	msg.position = [0]
	pub.publish(msg)

	# msg.name = ["wrist_left_axis0"]
	# for i in range(5):
	# 	msg.position = [-x for x in msg.position]
	# 	pub.publish(msg)
	# 	rospy.sleep(3)
	# msg.position = [0]
	# pub.publish(msg)

	# msg.name = ["wrist_left_axis2"]
	# for i in range(5):
	# 	msg.position = [-x for x in msg.position]
	# 	pub.publish(msg)
	# 	rospy.sleep(3)
	# msg.position = [0]
	# pub.publish(msg)
