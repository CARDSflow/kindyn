import rospy
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.srv import MotorConfigService, MotorConfigServiceRequest
import matplotlib.pyplot as plt
import numpy as np

rospy.init_node("elbow_test")
t = np.arange(1.5, 25.5, 0.001)
r = [0,-0.1]

srv = rospy.ServiceProxy("/roboy/oxford/middleware/MotorConfig", MotorConfigService)
req = MotorConfigServiceRequest()
ids =  [0,1,2,3,4,5,6,7]
req.config.global_id = ids
req.config.update_frequency = [500]*len(ids)

req.config.control_mode = [0]*len(ids)
req.config.PWMLimit = [30]*len(ids)
req.config.IntegralLimit = [24]*len(ids)
req.config.Kp = [1]*len(ids)
req.config.Ki = [0]*len(ids)
req.config.Kd = [0]*len(ids)
req.config.deadband = [0]*len(ids)
req.config.setpoint = [0]*len(ids)
srv(req)

pub = rospy.Publisher('/roboy/oxford/middleware/MotorCommand', MotorCommand, queue_size=1)
factor = 216000
# factor = 6000
# factor = 0.01
setpoints = (2*factor*np.sin(np.pi*t)) #+ 216000# + 1)*np.mean(r)
# plt.plot(setpoints)
# plt.show()
print(setpoints)
cmd = MotorCommand()
ids = [0] #[0,2,3,4,5,6,7]
cmd.global_id = ids #[0,1,3,4,5,6,7]

rate = rospy.Rate(100)

for id in ids:

    i =0
    # print(id)
    # cmd.global_id = [id]
    while not rospy.is_shutdown() and 10000>i:
    # for s in setpoints:
        # if i==0: i += 1;continue
        cmd.setpoint = [setpoints[i]]*len(ids)
        print(i)
        # print(setpoints[i])
        i += 1

        pub.publish(cmd)
        rate.sleep()
