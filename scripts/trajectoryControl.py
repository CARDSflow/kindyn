import rospy

import time
import math
from sensor_msgs.msg import JointState

rospy.init_node("trajectory")

joint_targets = rospy.Publisher('/joint_targets', JointState, queue_size=1)

def degree2rad(degree):
    return (math.pi/180*degree)

# # trajectory 1
# axis0 = [0,-degree2rad(60)  ,-degree2rad(60),   -degree2rad(60),    -degree2rad(60),   -degree2rad(60),0]
# axis1 = [0,0,0,0,0,0,0]
# axis2 = [0,0                ,-degree2rad(90),   degree2rad(90),    -degree2rad(90),   degree2rad(90),0]

# # trajectory 2
# axis0 = [0,0,0,0,0,0,0]
# axis1 = [0,-degree2rad(60) ,-degree2rad(60)   ,-degree2rad(60), -degree2rad(60)   ,-degree2rad(60)   ,0]
# axis2 = [0,0                ,-degree2rad(90)    ,degree2rad(90),-degree2rad(90)    ,degree2rad(90)     ,0]

# # # trajectory 3
# axis0 = [0,-degree2rad(20)  ,-degree2rad(20),   -degree2rad(20),    -degree2rad(20),   -degree2rad(20),0]
# axis1 = [0,-degree2rad(20)  ,-degree2rad(20),   -degree2rad(20),    -degree2rad(20),   -degree2rad(20),0]
# axis2 = [0,0                ,-degree2rad(90),   degree2rad(90),    -degree2rad(90),   degree2rad(90),0]

# # trajectory 1
# axis0 = [0,0,-degree2rad(50),0,0,0]
# axis1 = [0,-degree2rad(50),-degree2rad(50),-degree2rad(50),0]
# axis2 = [0,0,0,0,0,0]

# # trajectory 1
# axis0 = [0,-degree2rad(70),0,0]
# axis1 = [0,0,0,0]
# axis2 = [0,0,0,0,0,0]

# trajectory 1
axis0 = [0,0,0,0]
axis1 = [0,-degree2rad(70),0,0]
axis2 = [0,0,0,0]

time = [10,20,30,40]
sampleTime = 0.01
axis0_increment = []
axis1_increment = []
axis2_increment = []

for i in range(len(time)-1):
    samples = (time[i+1]-time[i])/sampleTime
    axis0_increment.append((axis0[i+1]-axis0[i])/samples)
    axis1_increment.append((axis1[i+1]-axis1[i])/samples)
    axis2_increment.append((axis2[i+1]-axis2[i])/samples)

startTime = rospy.Time.now()

msg = JointState()
msg.name = ["shoulder_right_axis0","shoulder_right_axis1","shoulder_right_axis2"]
msg.position = [axis0[0],axis1[0],axis2[0]]
msg.velocity = [0,0,0]
msg.effort = [0,0,0]

rate = rospy.Rate(1/sampleTime)
checkpoint = 0
while not rospy.is_shutdown() and checkpoint<len(time)-1:
    if (rospy.Time.now()-startTime).to_sec()<time[checkpoint]:
        joint_targets.publish(msg)
        rate.sleep()
        msg.position[0] = msg.position[0] + axis0_increment[checkpoint]
        msg.position[1] = msg.position[1] + axis1_increment[checkpoint]
        msg.position[2] = msg.position[2] + axis2_increment[checkpoint]
    else:
        checkpoint = checkpoint + 1
rospy.loginfo("trajectory done")