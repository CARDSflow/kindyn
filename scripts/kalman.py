#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from roboy_simulation_msgs.msg import Tendon


class Kalman(object):
    """docstring for Kalman"""
    def __init__(self, n_states, n_sensors):
        super(Kalman, self).__init__()
        self.n_states = n_states
        self.n_sensors = n_sensors

        self.x = np.matrix(np.zeros(shape=(n_states,1)))
        self.P = np.matrix(np.identity(n_states))
        self.F = np.matrix(np.identity(n_states))
        self.u = np.matrix(np.zeros(shape=(n_states,1)))
        self.H = np.matrix(np.zeros(shape=(n_sensors, n_states)))
        self.R = np.matrix(np.identity(n_sensors))
        self.I = np.matrix(np.identity(n_states))

        self.first = True

    def update(self, Z, R):
        '''Z: new sensor values as numpy matrix'''

        w = Z - self.H * self.x
        S = self.H * self.P * self.H.getT() + R
        K = self.P * self.H.getT() * S.getI()
        self.x = self.x + K * w
        self.P = (self.I - K * self.H) * self.P

    def predict(self):
        self.x = self.F * self.x + self.u
        self.P = self.F * self.P * self.F.getT()


class Runner(object):
    """docstring for Subscriber"""
    def __init__(self):
        super(Runner, self).__init__()
        rospy.init_node('joint_state_kalman_filter', anonymous=True)

        self.kalman = Kalman(n_states=38, n_sensors=38)
        self.kalman.H = np.matrix(np.identity(self.kalman.n_states))
        self.kalman.P *= 0.01
        # self.kalman.R *= 0.01
        self.R1 = self.kalman.R * 0.001
        self.R2 = self.kalman.R * 0.01

        self.last_l = None
        self.last_l_ext = None
        self.delta_l = None
        self.delta_l_ext = None
        self.inital_l = self.kalman.x

        self.publisher = rospy.Publisher('kalman/tendon_kf', Tendon)

        rospy.Subscriber('/roboy/pinky/control/tendon_state', Tendon, self.callback_cardsflow)
        rospy.Subscriber('/roboy/pinky/control/tendon_state_ext', Tendon, self.callback_external)
        # rospy.spin()

    def callback_cardsflow(self, data):
        # print "received data: ", data
        Z = np.array(data.l)[:, None]

        if self.last_l is None:
            self.last_l = Z
            return

        self.delta_l = Z - self.last_l
        self.last_l = Z

    def callback_external(self, data):
        # print "received data: ", data
        Z = np.array(data.l)[:, None]

        if self.last_l_ext is None:
            self.last_l_ext = Z
            return

        self.delta_l_ext = Z - self.last_l_ext
        self.last_l_ext = Z

    def run(self):

        if self.last_l is None or self.last_l_ext is None:
            return

        if self.kalman.first:
            self.inital_l = self.last_l_ext
            # self.kalman.x = self.inital_l
            self.kalman.first = False

        if self.delta_l is None or self.delta_l_ext is None:
            return

        # delta_l = self.last_l - self.inital_l
        # delta_l_ext = self.last_l_ext - self.inital_l

        self.kalman.predict()
        self.kalman.update(self.delta_l, self.R1)
        self.kalman.update(self.delta_l_ext, self.R2)

        self.inital_l += np.array(self.kalman.x)

        tendon_msg = Tendon()
        tendon_msg.l = self.inital_l
        self.publisher.publish(tendon_msg)


if __name__ == '__main__':
    runner = Runner()
    rate = rospy.Rate(20)

    # while runner.publisher.get_num_connections() == 0:
    #     rospy.loginfo("Waiting for a connection")
    #     rospy.sleep(1)

    while not rospy.is_shutdown():
        runner.run()
        rate.sleep()


