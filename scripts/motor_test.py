import rospy

import time
import math
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.srv import MotorConfigService

rospy.init_node("trajectory")

motor_target = rospy.Publisher('/roboy/middleware/MotorCommand', MotorCommand, queue_size=1)

motor_setpoints = [3000,3000+4096*3,3000,3000]
MAX_CYCLE = 1
time = [0,5,6,10]
sampleTime = 0.01
motor_setpoint_increment = []

for i in range(len(motor_setpoints)-1):
    samples = (time[i+1]-time[i])/sampleTime
    motor_setpoint_increment.append((motor_setpoints[i+1]-motor_setpoints[i])/samples)
    print("samples %d, setpoint increment %d"%(samples, motor_setpoint_increment[i]))

msg = MotorCommand()
msg.motor = [1]

cycle = 0
rate = rospy.Rate(1/sampleTime)
while not rospy.is_shutdown() and cycle<MAX_CYCLE:
    checkpoint = 1
    startTime = rospy.Time.now()
    msg.setpoint = [motor_setpoints[0]]
    while not rospy.is_shutdown() and checkpoint<len(motor_setpoints)-1:
        if (rospy.Time.now()-startTime).to_sec()<time[checkpoint]:
            motor_target.publish(msg)
            rate.sleep()
            msg.setpoint[0] = msg.setpoint[0] + motor_setpoint_increment[checkpoint-1]
            rospy.loginfo_throttle(1,"time %f"%(rospy.Time.now()-startTime).to_sec())
        else:
            rospy.loginfo("checkpoint %d done, time now %f, checkpoint time %f"%(checkpoint,(rospy.Time.now()-startTime).to_sec(),time[checkpoint]))
            checkpoint = checkpoint + 1
    rospy.loginfo("cycle %d done"%(cycle))
    cycle = cycle + 1
