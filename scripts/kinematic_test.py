#!/usr/bin/python3
import pybullet as p
import math
import time
import rospy
from sensor_msgs.msg import JointState, CompressedImage
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import InteractiveMarkerUpdate
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
import cv2

from pyquaternion import Quaternion
import pybullet_data

import rospkg
rospack = rospkg.RosPack()
robots_path = rospack.get_path('robots')

import argparse
parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--wait', dest='wait', action='store_const',
                    const=True, default=False,
                    help='wait 3s before autocalibration')

args = parser.parse_args()

if args.wait:
    rospy.logwarn("Waiting 3 seconds before launching IK node...")
    rospy.sleep(3)



def quaternion_multiply(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                     -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64)



rospy.init_node("bullet_joints")
topic_root = "/roboy/oxford"
joint_target_pub = rospy.Publisher(topic_root+"/control/joint_targets", JointState, queue_size=1)
caml_pub = rospy.Publisher(topic_root+'/sensors/caml/compressed', CompressedImage,tcp_nodelay=True,queue_size=1)
camr_pub = rospy.Publisher(topic_root+'/sensors/camr/compressed', CompressedImage,tcp_nodelay=True,queue_size=1)
bridge = CvBridge()

msg = JointState()

p.connect(p.GUI) # (p.GUI)
roboy = ob = p.loadURDF(robots_path+"/musc_shoulder/model.urdf", useFixedBase=1, basePosition=(0,0,1), baseOrientation=(0,0,0.7071,0.7071))

p.setAdditionalSearchPath(pybullet_data.getDataPath())
# castle = p.loadURDF("samurai.urdf", 0,2,0)

p.setGravity(0,0,-10)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0

useOrientation = 0

#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 1
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15


markerVisualId = {}
markerVisualId["hand_left"] = p.addUserDebugText("hand_left", [0,0,0])
markerVisualId["hand_right"] = p.addUserDebugText("hand_right", [0,0,0])
markerVisualId["head"] = p.addUserDebugText("head", [0,0,0])


global freeJoints, numJoints, init_orn,  head_initialized, right_initialized, right_orn_offset
numJoints = p.getNumJoints(ob)
rospy.logwarn("Using hardcoded orientation correction quterions for controllers!")
right_initialized = True
left_initialized = True
head_initialized = False
orn_offset = {}
orn_offset["hand_right"] = [-0.5839645865280022, -0.6905731440685547, 0.2882076733510812, 0.3146909727401144] #[0,0,0,1]
orn_offset["hand_left"] = orn_offset["hand_right"]

if not rospy.has_param('publish_cardsflow'):
    rospy.logwarn("Set param /publish_cardsflow true to forward joint poses")
    rospy.set_param('publish_cardsflow', False)

init_orn = (0,0,0,1)
freeJoints = []
joint_names = {}
efs = {}
for i in range(numJoints):
    info = p.getJointInfo(ob,i)
    if info[2] == p.JOINT_REVOLUTE:
        # if info[1] == b'head_axis0' or info[1] == b'head_axis1' or info[1] == b'head_axis2':
        #     continue
        freeJoints.append(i)
        joint_names[i] = (p.getJointInfo(0, i)[1].decode("utf-8"))
        rospy.loginfo("Added joint %s to freeJoints"%joint_names[i])
    if info[12] == b'hand_right':
        endEffectorId = i
        print("EF id: " + str(i))
        efs["hand_right"] = i
    if info[12] == b'hand_left':
        efs["hand_left"] = i


height = 240 #720# 1080 #720 #240#*2
width = 320 #1280 #1920 #1280 #320#*2
aspect = width/height

fov, nearplane, farplane = 100, 0.1, 100
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

init_up_vector = (0, 0, 1)

def camera(link_id):
    com_p, com_o, _, _, _, _ = p.getLinkState(roboy, link_id)
    com_t = list(com_p)
    com_p = list(com_p)

    com_t[1] = com_p[1] - 5

    rot_matrix = p.getMatrixFromQuaternion(com_o)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    init_camera_vector = com_t
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)

    view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
    w, h, img, depth, mask = p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=0, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags=p.ER_NO_SEGMENTATION_MASK)
    # p.addUserDebugLine(lineFromXYZ=com_p, lineToXYZ=camera_vector, lifeTime=0)
    return w,h,img

def to_cv2(w,h,img,right):
    rgba_pic = np.array(img, np.uint8).reshape((h, w, 4))
    # if right:
        # rgba_pic = np.roll(rgba_pic, 300)
    pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
        # pic = pic[0:height,300:width]
    # else:
        # rgba_pic = np.roll(rgba_pic, -300)
        # pic = cv2.cvtColor(rgba_pic, cv2.COLOR_RGBA2BGR)
        # pic = pic[0:height,0:width-300]

    return pic

def isRight(id):
    info = p.getJointInfo(ob,id)
    return "right" in str(info[12])

def idFromName(joint_name):
    for i in range(numJoints):
        if p.getJointInfo(0,i)[1].decode("utf-8") == joint_name:
            return i
    return None

def accurateCalculateInverseKinematics(ob, endEffectorId, targetPos, threshold, maxIter, targetOrn=None):
    closeEnough = False
    iter = 0
    dist2 = 1e30
    while (not closeEnough and iter < maxIter):
        if targetOrn is None:
            jointPoses = p.calculateInverseKinematics(ob, endEffectorId, targetPos)
        else:
            jointPoses = p.calculateInverseKinematics(ob, endEffectorId, targetPos, targetOrientation=targetOrn)
        #import pdb; pdb.set_trace()
        for idx,pos in zip(freeJoints,jointPoses): #range(len(freeJoints)):#p.getNumJoints(ob)):#
            # p.resetJointState(ob, freeJoints[i], jointPoses[i])
            if (isRight(idx) and isRight(endEffectorId)) or (not isRight(idx) and not isRight(endEffectorId)):
                p.setJointMotorControl2(bodyIndex=ob,
                                            jointIndex=idx,#freeJoints[i],
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=pos,
                                            maxVelocity=0.5)#jointPoses[i])
                                            # targetVelocity=0,
                                            # force=1,
                                            # positionGain=5,
                                            # velocityGain=0.1)
        ls = p.getLinkState(ob, endEffectorId)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter = iter + 1
    #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
    return jointPoses

def ik(msg):

    global freeJoints, markerVisualId, head_initialized, right_initialized, orn_offset, left_initialized
    pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    orn = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]


    if msg.header.frame_id == "head":
        if not head_initialized:
            pos = list(pos)
            pos[2] -= 0.4
            rospy.logwarn_throttle(1, "head: " +  str(pos))
            p.resetBasePositionAndOrientation(0, pos, (0,0,0.7071,0.7071))
            head_initialized = True

    else:#if msg.header.frame_id == "hand_left":
        if not right_initialized and msg.header.frame_id == "hand_right":
            orn = list(orn)
            ctlr_orn = Quaternion(orn[3], orn[0], orn[1], orn[2])
            ctlr_orn = ctlr_orn.inverse
            orn_offset["hand_right"] = [ctlr_orn[1], ctlr_orn[2], ctlr_orn[3], ctlr_orn[0]]
            rospy.logwarn("Right controller orientation offset: " + str(orn_offset["hand_right"]))
            # rospy.logwarn_throttle(1,right_orn_offset)
            right_initialized = True

        if not left_initialized and msg.header.frame_id == "hand_left":
            orn = list(orn)
            ctlr_orn = Quaternion(orn[3], orn[0], orn[1], orn[2])
            ctlr_orn = ctlr_orn.inverse
            orn_offset["hand_left"] = [ctlr_orn[1], ctlr_orn[2], ctlr_orn[3], ctlr_orn[0]]
            rospy.logwarn("Left controller orientation offset: " + str(orn_offset["hand_left"]))
            # rospy.logwarn_throttle(1,left_orn_offset)
            left_initialized = True

        else:
            endEffectorId = efs[msg.header.frame_id]
            threshold = 0.03
            maxIter = 80

            # if first:
            #     init_orn = orn
            # else:
            rospy.loginfo_throttle(1, orn)
            corrected_orn = quaternion_multiply(orn, orn_offset[msg.header.frame_id]) #(0,0,0.7071,0.7071))
            # corrected_orn = quaternion_multiply(corrected_orn, (0,0,-0.7071,0.7071))

            # jointPoses = p.calculateInverseKinematics(0, endEffectorId, pos, corrected_orn)#(0,0,0,1)) #orn)  #accurateCalculateInverseKinematics(0, endEffectorId, pos,
            #                                                 # threshold, maxIter, corrected_orn)

            jointPoses = accurateCalculateInverseKinematics(0, endEffectorId, pos,
                                                            threshold, maxIter)#, corrected_orn)
            # if (useSimulation):
            # for i in range(len(freeJoints)):
            #     p.setJointMotorControl2(bodyIndex=0,
            #                             jointIndex=freeJoints[i],
            #                             controlMode=p.POSITION_CONTROL,
            #                             targetPosition=jointPoses[i],
            #                             targetVelocity=0,
            #                             force=1,
            #                             positionGain=5,
            #                             velocityGain=0.1)

    markerVisualId[msg.header.frame_id] = p.addUserDebugText(msg.header.frame_id, pos, replaceItemUniqueId=markerVisualId[msg.header.frame_id])

def marker(msg):
    global freeJoints
    global markerVisualId
    threshold = 0.001
    maxIter = 100
    for i in range(len(msg.poses)):
        pose_msg = msg.poses[i]
        endEffectorId = efs[pose_msg.name]

        pos = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]
        orn = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]
        markerVisualId = [0,0.7,0,0.7] #p.addUserDebugText("X",pos, replaceItemUniqueId=markerVisualId)
        jointPoses = accurateCalculateInverseKinematics(0, endEffectorId, pos, orn,
                                                        threshold, maxIter)
        if (useSimulation):
            for i in range(len(freeJoints)):
                p.setJointMotorControl2(bodyIndex=0,
                                        jointIndex=freeJoints[i],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=jointPoses[i],
                                        targetVelocity=0,
                                        force=100,
                                        positionGain=1,
                                        velocityGain=0.1)

def joint_targets_cb(msg):
    for i in range(len(msg.name)):
        id = idFromName(msg.name[i])
        if id is not None:
            p.setJointMotorControl2(bodyIndex=ob,
                                        jointIndex=id,#freeJoints[i],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=msg.position[i],
                                        maxVelocity=0.3)
            # p.setJointMotorControl2(bodyIndex=0,
            #                         jointIndex=id,
            #                         controlMode=p.POSITION_CONTROL,
            #                         targetPosition=msg.position[i],
            #                         targetVelocity=0,
            #                         force=10,
            #                         positionGain=0.4,
            #                         velocityGain=0.1)
    # rospy.logwarn_throttle(1,debug + "\n\n")

def cmdVelCB(msg):
    """callback to receive vel commands from user"""
    p.resetBaseVelocity(roboy, [msg.linear.x,msg.linear.y,msg.linear.z],
                              [msg.angular.x,msg.angular.y,msg.angular.z])

ik_sub = rospy.Subscriber(topic_root + "/control/bullet_ik", PoseStamped, ik, queue_size=1)
joint_target_sub = rospy.Subscriber(topic_root+"/simulation/joint_targets", JointState, joint_targets_cb, queue_size=1)

vel_sub = rospy.Subscriber("/cmd_vel", Twist, cmdVelCB)
marker_sub = rospy.Subscriber("/interactive_markers/update", InteractiveMarkerUpdate, marker)
rate = rospy.Rate(200)
while not rospy.is_shutdown():
    p.stepSimulation()
    # left = camera(11)
    # right = camera(12)
    # left_pic = to_cv2(left[0],left[1],left[2],0)
    # right_pic = to_cv2(right[0],right[1],right[2],1)
    # caml_pub.publish(bridge.cv2_to_compressed_imgmsg(left_pic))
    # camr_pub.publish(bridge.cv2_to_compressed_imgmsg(right_pic))
    # vis = np.concatenate((left_pic, right_pic), axis=1)
    # cv2.imshow('window', vis)
    # cv2.waitKey(1)
    msg.header.stamp = rospy.Time.now()
    msg.position = []
    msg.velocity = []
    msg.effort = []
    msg.name = []
    for i in freeJoints:
    #     if "head" not in joint_names[i]:
        js = p.getJointState(0, i)
        msg.position.append(js[0])
        msg.velocity.append(0)
        msg.effort.append(0)
        msg.name.append(joint_names[i])
    if rospy.get_param('publish_cardsflow'):
    #     # print(msg)
        rospy.loginfo_throttle(1, msg.position)
        joint_target_pub.publish(msg)
    rate.sleep()
p.disconnect()
