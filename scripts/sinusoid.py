import rospy
from roboy_middleware_msgs.msg import MotorCommand
import matplotlib.pyplot as plt
import numpy as np

rospy.init_node("elbow_test")
t = np.arange(1.5, 25.5, 0.0001)
r = [0,-0.1]

pub = rospy.Publisher('/roboy/pinky/middleware/MotorCommand', MotorCommand, queue_size=1)
setpoints = (0.03*np.sin(np.pi*t)) -0.3# + 1)*np.mean(r)
# plt.plot(setpoints)
# plt.show()
print(setpoints)
cmd = MotorCommand()
cmd.global_id = [19]

rate = rospy.Rate(600)

for s in setpoints:
    cmd.setpoint = [s]
    pub.publish(cmd)
    rate.sleep()

