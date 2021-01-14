#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped
from roboy_control_msgs.msg import Emotion
import pyroboy


# rospy.init_node('joy_ctl')
# pyroboy.init()


class JoystickRoboy:

    def __init__(self):
        self.topic_root = "/roboy/pinky"
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.joint_pub = rospy.Publisher(
            self.topic_root + '/joint_targets', JointState, queue_size=1)
        self.eyes_pub = rospy.Publisher(
            self.topic_root + '/eyes', PoseStamped, queue_size=1)
        self.joint_sub = rospy.Subscriber(
            self.topic_root + '/cardsflow_joint_states', JointState, self.joint_cb)
        self.face_emotions = ["shy", "hearts", "hypno_color", "kiss",
                              "angry", "talking", "pissed", "img:money", None, None, None, None]
        self.show_emotions = []
        self.axes_names = ["head_axis1",
                           "head_axis0", "eyes_axis1", "eyes_axis0"]
        self.axes = []
        self.prev_axes = []
        self.scale = [-0.01, 0.01]
        self.joint_names = []
        self.joint_positions = []

        self.emotion_pub = rospy.Publisher("/roboy/pinky/cognition/face/emotion", Emotion, queue_size=1)


    def joy_cb(self, msg):
        self.prev_axes = self.axes
        self.axes = msg.axes
        self.show_emotions = [x for (x, y) in zip(
            self.face_emotions, msg.buttons) if y == 1]

    def head_ctl(self):
        joint_msg = JointState()

        if len(self.axes) != 0 and len(self.joint_positions) != 0:
            for i in range(len(self.axes_names[:2])):
                j = self.axes_names[i]
                pos = self.get_joint_position(j) - self.scale[i]*self.axes[i]
                if pos is not None:
                    joint_msg.name.append(j)
                    joint_msg.position.append(pos)
                    joint_msg.velocity.append(0)
                    joint_msg.effort.append(0)
            self.joint_pub.publish(joint_msg)

    def eyes_ctl(self):
        msg = PoseStamped()
        if len(self.axes) != 0:
            if (self.axes != self.prev_axes):
                msg.pose.position.x = self.axes[self.axes_names.index(
                    "eyes_axis1")]
                msg.pose.position.y = self.axes[self.axes_names.index(
                    "eyes_axis0")]
                self.eyes_pub.publish(msg)

    def emotion_ctl(self):
        if len(self.show_emotions) != 0:
            for e in self.show_emotions:
                if e is not None:
                    msg = Emotion()
                    msg.emotion = e
                    self.emotion_pub.publish(msg)
                    # pyroboy.show_emotion(e)
            self.show_emotions = []

    def update(self):
        self.head_ctl()
        self.eyes_ctl()
        self.emotion_ctl()

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
                rospy.logwarn("could not find position of %s" % name)
                return None


if __name__ == '__main__':

    jr = JoystickRoboy()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        jr.update()
        rate.sleep()
