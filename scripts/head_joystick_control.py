import rospy


from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String
sphere_head_axis0 = rospy.Publisher('sphere_head_axis0/sphere_head_axis0/target', Float32, queue_size=1)
sphere_head_axis1 = rospy.Publisher('sphere_head_axis1/sphere_head_axis1/target', Float32, queue_size=1)
sphere_head_axis2 = rospy.Publisher('sphere_head_axis2/sphere_head_axis2/target', Float32, queue_size=1)
emotion = rospy.Publisher('roboy/cognition/face/show_emotion', String, queue_size=1)

def callback(data):
    global sphere_head_axis0
    global sphere_head_axis1
    global sphere_head_axis2
    global emotion
    axis1 = data.axes[0]/6.0
    axis0 = data.axes[1]/20.0
    axis2 = data.axes[2]/2.0
    sphere_head_axis0.publish(axis0)
    sphere_head_axis1.publish(axis1)
    sphere_head_axis2.publish(axis2)
    rospy.loginfo("%f %f %f"%(axis0, axis1, axis2))
    if(data.buttons[6]):
        emotion.publish('shy')
    elif (data.buttons[7]):
        emotion.publish('kiss')
    elif (data.buttons[8]):
        emotion.publish('hearts')
    elif (data.buttons[9]):
        emotion.publish('smileblink')
    elif (data.buttons[10]):
        emotion.publish('teeth')

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('head_joystick_control', anonymous=True)

    rospy.Subscriber("joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()