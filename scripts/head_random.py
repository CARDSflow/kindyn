import rospy
import random
import time
from sensor_msgs.msg import JointState
target = rospy.Publisher('joint_targets', JointState, queue_size=10)

rospy.init_node('head_random')
msg = JointState()
msg.name = ["neck_axis0","neck_axis1","neck_axis2"]
msg.velocity = [0,0,0]
msg.effort = [0,0,0]
while(not rospy.is_shutdown()):
    n0 = 0%random.uniform(-0.05,0.05)
    n1 = random.uniform(-0.2,0)
    n2 = random.uniform(-0.3,0.3)
    print("%f %f %f"%(n0,n1,n2))
    msg.position = [n0,n2,n2]
    target.publish(msg)
    time.sleep(3)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

