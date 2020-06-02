import rospy
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped



class JoystickRoboy:

	def __init__(self):
		self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
		self.joint_pub = rospy.Publisher('/joint_targets', JointState, queue_size=1)
		self.eyes_pub = rospy.Publisher('/roboy/eyes', PoseStamped, queue_size=1)
		self.joint_sub = rospy.Subscriber('/cardsflow_joint_states', JointState, self.joint_cb)
		self.axes_names = ["head_axis1", "head_axis0"]
		self.axes = []
		self.scale = [0.05, -0.05]
		self.joint_names = []
		self.joint_positions = []

	def joy_cb(self, msg):
		self.axes = msg.axes
	
	def update(self):
		joint_msg = JointState()
		
		if len(self.axes) != 0:
			for i in range(len(self.axes_names)):
				j = self.axes_names[i]
				pos = self.get_joint_position(j) + self.scale[i]*self.axes[i]
				if pos is not None:
					joint_msg.name.append(j)
					joint_msg.position.append(pos)
					joint_msg.velocity.append(0)
					joint_msg.effort.append(0)
			self.joint_pub.publish(joint_msg)


	def joint_cb(self, msg):
		
		self.joint_names = msg.name
		self.joint_positions = msg.position
		
	def get_joint_position(self, name):
		if len(self.joint_names) != 0 and len(self.joint_positions) != 0:
			try:
				# import pdb; pdb.set_trace()
				id = self.joint_names.index(name)
				return self.joint_positions[id]
			except: 
				rospy.logwarn("could not find position of %s"%name)
				return None

if __name__ == '__main__':
	rospy.init_node('joy_ctl')
	jr = JoystickRoboy()
	rate = rospy.Rate(200)
	while not rospy.is_shutdown():
		jr.update()
		rate.sleep()
