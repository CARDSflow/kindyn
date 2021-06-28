import rospy
from roboy_middleware_msgs.msg import MotorState

rospy.init_node("muscle_length_error")
pub = rospy.Publisher("/roboy/pinky/middleware/MotorError", MotorState, queue_size=1)
error = {}
def cb(msg):
    msg1 = MotorState()
    msg1.header.stamp = rospy.Time.now()
    msg1.global_id = msg.global_id
    msg1.encoder0_pos = [100*(x-y) for (x,y) in zip(msg.setpoint, msg.encoder0_pos)]
    pub.publish(msg1)
    # for i in range(len(msg.global_id)):
    #     msg1.global_id
    #     error[msg.global_id[i]] = msg.setpoint[i] - msg.encoder0_pos[i]



sub = rospy.Subscriber("/roboy/pinky/middleware/MotorState", MotorState, cb)
rospy.spin()