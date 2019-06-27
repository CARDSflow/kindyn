import rospy

import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point
from roboy_control_msgs.msg import PerformMovementsActionGoal
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
sphere_head_axis0 = rospy.Publisher('sphere_head_axis0/sphere_head_axis0/target', Float32, queue_size=10)
sphere_head_axis1 = rospy.Publisher('sphere_head_axis1/sphere_head_axis1/target', Float32, queue_size=10)
sphere_head_axis2 = rospy.Publisher('sphere_head_axis2/sphere_head_axis2/target', Float32, queue_size=10)
emotion = rospy.Publisher('roboy/cognition/face/show_emotion', String, queue_size=1)
movement_left = rospy.Publisher('/shoulder_left_movements_server/goal', PerformMovementsActionGoal, queue_size=1)
movement_right = rospy.Publisher('/shoulder_right_movements_server/goal', PerformMovementsActionGoal, queue_size=1)
interactive_marker = rospy.Publisher('/interactive_markers/feedback', InteractiveMarkerFeedback, queue_size=1)
visualisation = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
joystick_control_active = False
face_tracking_active = False
head_tracking_speed = 2

def joystick_callback(data):
    global sphere_head_axis0
    global sphere_head_axis1
    global sphere_head_axis2
    global joystick_control_active
    global emotion
    global movement_left
    global movement_right
    global head_tracking_speed
    axis0 = data.axes[1]/6.0
    axis1 = data.axes[0]/3.0
    axis2 = data.axes[2]*1.5
    if abs(axis0)>0.00001 or abs(axis1)>0.00001 or abs(axis2)>0.00001:
        joystick_control_active = True
        sphere_head_axis0.publish(axis0)
        sphere_head_axis1.publish(axis1)
        sphere_head_axis2.publish(axis2)
        rospy.loginfo("%f %f %f"%(axis0, axis1, axis2))
    else:
        joystick_control_active = False
    if(data.buttons[6]):
        emotion.publish('shy')
        print('shy')
    elif (data.buttons[7]):
        emotion.publish('kiss')
        print('kiss')
    elif (data.buttons[8]):
        emotion.publish('hearts')
        print('hearts')
    elif (data.buttons[9]):
        emotion.publish('smileblink')
        print('smileblink')
    elif (data.buttons[10]):
        emotion.publish('teeth')
        print('teeth')
    elif (data.buttons[0]):
        msg = PerformMovementsActionGoal()
        msg.goal.actions = ['shoulder_left_hug']
        movement_left.publish(msg)
        msg.goal.actions = ['shoulder_right_hug']
        movement_right.publish(msg)
        print("HUG HUG HUG")
    elif (data.axes[4]==1):
        emotion.publish('lookleft')
    elif (data.axes[4]==-1):
        emotion.publish('lookright')

    head_tracking_speed = 4-(data.axes[3]+1)*2
    x = 0.5+data.axes[0]
    y = -0.5-data.axes[1]
    z = 0+0.5*(1+data.axes[3])
    pos = InteractiveMarkerFeedback()
    pos.header.frame_id = 'world'
    pos.marker_name = "arm_left_palm"
    pos.event_type = 5
    pos.pose.position.x = x
    pos.pose.position.y = y
    pos.pose.position.z = z
    rospy.loginfo_throttle(1,"%f %f %f"%(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z))
    vis = Marker()
    vis.type = 10
    vis.id = 69696969
    vis.header.frame_id = 'world'
    vis.pose = pos.pose
    vis.mesh_resource = "package://robots/common/meshes/visuals/target.stl"
    vis.scale.x = 0.01
    vis.scale.y = 0.01
    vis.scale.z = 0.01
    vis.color.r = 0
    vis.color.g = 1
    vis.color.b = 1
    vis.color.a = 1
    visualisation.publish(vis)
    interactive_marker.publish(pos)

def face_coordinates_callback(data):
    global joystick_control_active
    global face_tracking_active
    global sphere_head_axis0
    global sphere_head_axis1
    global sphere_head_axis2
    global head_tracking_speed
    global interactive_marker
    if not joystick_control_active:
        axis0 = data.y/800.0
        if axis0<0.1:
            axis0 = 0.1
        axis2 = -data.x/1000.0
        axis1 = axis2/10.0
        if axis0>-0.15:
            print("%f %f"%(data.x, data.y))
            print("%f %f %f"%(axis0, axis1, axis2))
            sphere_head_axis0.publish(axis0)
            sphere_head_axis1.publish(axis1)
            sphere_head_axis2.publish(axis2)
            face_tracking_active = True
        else:
            sphere_head_axis0.publish(0.1)
            sphere_head_axis2.publish(0)
            face_tracking_active = False
        time.sleep(head_tracking_speed)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('head_joystick_control')

    rospy.Subscriber("joy", Joy, joystick_callback)
    rospy.Subscriber("/roboy/cognition/vision/face_coordinates", Point, face_coordinates_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
