import rospy

import time
import math
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.srv import MotorConfigService

rospy.init_node("trajectory")

motor_target = rospy.Publisher('/roboy/middleware/MotorCommand', MotorCommand, queue_size=1)

motor_setpoints = [0,-2000,0,2000,0]

time = [5,10,15,20,25,30]
sampleTime = 0.01
motor_setpoint_increment = []

for i in range(len(motor_setpoints)-1):
    samples = (time[i+1]-time[i])/sampleTime
    motor_setpoint_increment.append((motor_setpoints[i+1]-motor_setpoints[i])/samples)

startTime = rospy.Time.now()

msg = MotorCommand()
msg.motor = [0]
msg.setpoint = [motor_setpoints[0]]

rate = rospy.Rate(1/sampleTime)
checkpoint = 0
while not rospy.is_shutdown() and checkpoint<len(motor_setpoints)-1:
    if (rospy.Time.now()-startTime).to_sec()<time[checkpoint]:
        motor_target.publish(msg)
        rate.sleep()
        msg.setpoint[0] = msg.setpoint[0] + motor_setpoint_increment[checkpoint]
    else:
        checkpoint = checkpoint + 1
rospy.loginfo("trajectory done")