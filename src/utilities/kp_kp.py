import rospy
import sys
from roboy_control_msgs.srv import SetControllerParameters
from std_msgs.msg import Float32

JOINT_SHOULDER_AXIS0_RIGHT = "right_shoulder_axis0"
JOINT_SHOULDER_AXIS1_RIGHT = "right_shoulder_axis1"
JOINT_SHOULDER_AXIS2_RIGHT = "right_shoulder_axis2"
JOINT_SHOULDER_AXIS0_LEFT = "left_shoulder_axis0"
JOINT_SHOULDER_AXIS1_LEFT = "left_shoulder_axis1"
JOINT_SHOULDER_AXIS2_LEFT = "left_shoulder_axis2"
JOINT_ELBOW_RIGHT = "elbow_right"
JOINT_ELBOW_LEFT = "elbow_left"
JOINT_WRIST_RIGHT_SPHERE_AXIS0 = "wrist_right_sphere_axis0"
JOINT_WRIST_RIGHT_SPHERE_AXIS1 = "wrist_right_sphere_axis1"
JOINT_WRIST_RIGHT_SPHERE_AXIS2 = "wrist_right_sphere_axis2"
JOINT_WRIST_LEFT_SPHERE_AXIS0 = "wrist_left_sphere_axis0"
JOINT_WRIST_LEFT_SPHERE_AXIS1 = "wrist_left_sphere_axis1"
JOINT_WRIST_LEFT_SPHERE_AXIS2 = "wrist_left_sphere_axis2"

ROS_JOINT_HIP_RIGHT = "joint_hip_right"
ROS_JOINT_KNEE_RIGHT = "joint_knee_right"
ROS_JOINT_ANKLE_RIGHT = "joint_foot_right"
ROS_JOINT_HIP_LEFT = "joint_hip_left"
ROS_JOINT_KNEE_LEFT = "joint_knee_left"
ROS_JOINT_ANKLE_LEFT = "joint_foot_left"


JOINT_LIST = [JOINT_SHOULDER_AXIS0_RIGHT, JOINT_SHOULDER_AXIS1_RIGHT, JOINT_SHOULDER_AXIS2_RIGHT,
              JOINT_SHOULDER_AXIS0_LEFT, JOINT_SHOULDER_AXIS1_LEFT, JOINT_SHOULDER_AXIS2_LEFT, JOINT_ELBOW_RIGHT, JOINT_ELBOW_LEFT, JOINT_WRIST_RIGHT_SPHERE_AXIS0,
              JOINT_WRIST_LEFT_SPHERE_AXIS0, ROS_JOINT_HIP_RIGHT, ROS_JOINT_KNEE_RIGHT, ROS_JOINT_ANKLE_RIGHT, ROS_JOINT_HIP_LEFT, ROS_JOINT_KNEE_LEFT, ROS_JOINT_ANKLE_LEFT] #JOINT_WRIST_RIGHT_SPHERE_AXIS1, JOINT_WRIST_RIGHT_SPHERE_AXIS2,  JOINT_WRIST_LEFT_SPHERE_AXIS1, JOINT_WRIST_LEFT_SPHERE_AXIS2,


def setJointControllerParameters(proportionalVal, derivativeVal):
    for thisJointName in JOINT_LIST:
        rospy.wait_for_service(thisJointName + '/' + thisJointName + '/params')
        try:
            param_srv = rospy.ServiceProxy(thisJointName + '/' + thisJointName + '/params', SetControllerParameters)
            param_srv(proportionalVal, derivativeVal)
        except rospy.ServiceException, e:
            print
            "Service call for " + thisJointName + "failed: %s" % e

    print("Controller paramters updated")

def main():

    print("Usage: kp_kd.py [kp] [kd]. Sets PD values for all joint controllers in arms and legs")
    kp = 1
    kd = 1

    if len(sys.argv) > 2:
        kp = int(sys.argv[1])
        kd = int(sys.argv[2])
    elif len(sys.argv) > 1:
        kp = int(sys.argv[1])

    setJointControllerParameters(kp, kd)

    return 0

if __name__ == '__main__':
    main()
