import rospy
from sensor_msgs.msg import JointState
import numpy as np
from itertools import product

from progressbar import progressbar

rospy.init_node("axis_joint_test")
pub = rospy.Publisher("/roboy/oxford/simulation/joint_targets", JointState, queue_size=1)

# axis0_limits = [0,1.5]
# axis1_limits = [-0.32,0.0]
# axis2_limits = [-0.3,0.0]

axis0_limits = [0,1.0]
axis1_limits = [0,0.52]
axis2_limits = [0.0,0.3]

msg = JointState()
msg.name = ["axis0","axis1", "axis2"]

def explore_workspace():
    targets = []
    ops = [[0,"x_min","x_max"],[0,"y_min","y_max"],[0,"z_min","z_max"]]
    points = list(product(*ops))
    idx = 0

    for sid in range(len(points)):
        for eid in range(sid,len(points)):
            start = points[sid]
            end = points[eid]
            if start != end:
                if((start[0] == 0 or end[0] == 0) and start[0] != end[0]): continue
                if((start[1] == 0 or end[1] == 0) and start[1] != end[1]): continue
                if((start[2] == 0 or end[2] == 0) and start[2] != end[2]): continue
                idx = idx+1
                # start = list.map(lambda item: item.replace("z_min", axis2_limits[0], start))
                # start = list.map(lambda item: item.replace("z_max", axis2_limits[1], start))
                targets.append(start)
                targets.append(end)
    for j in range(len(targets)):
        targets[j]  = list(targets[j])
        for i in range(len(targets[j])):
            if targets[j][i]=='x_min': targets[j][i] = axis0_limits[0]
            if targets[j][i]=='x_max': targets[j][i] = axis0_limits[1]
            if targets[j][i]=='y_min': targets[j][i] = axis1_limits[0]
            if targets[j][i]=='y_max': targets[j][i] = axis1_limits[1]
            if targets[j][i]=='z_min': targets[j][i] = axis2_limits[0]
            if targets[j][i]=='z_max': targets[j][i] = axis2_limits[1]
    # for t in progressbar(targets):
    #     msg.position = t
    #     pub.publish(msg)
    #     rospy.sleep(1)
    return targets

global targets, idx, out_msg, last_timestamp
idx = 0
targets = explore_workspace()
out_msg = JointState()
out_msg.name = ["axis0","axis1", "axis2"]
out_msg.position = [0,0,0]
# next = targets[idx]
# out_msg.position = next
# pub.publish(out_msg)
wait = False

def js_sub(in_msg):
    global idx, out_msg, targets, next, wait
    # print(in_msg.position)
    # print(out_msg.position)
    diff = [abs(element1 - element2)<0.1 for (element1, element2) in zip(out_msg.position, in_msg.position)]
    # print(diff)
    # print(all(diff) < 0.1)
    # import pdb; pdb.set_trace()
    # print(diff)
    if (diff[0] and diff[1]) or (rospy.Time.now()-last_timestamp).secs > 2:
        wait = False


sub = rospy.Subscriber("/roboy/oxford/control/cardsflow_joint_states", JointState, js_sub)
# sub = rospy.Subscriber("/roboy/oxford/sensing/external_joint_states", JointState, js_sub)

# while idx < len(targets) or not rospy.is_shutdown():
#     next = targets[idx]
#     out_msg.position = next
#     pub.publish(out_msg)
#     if not wait or (rospy.Time.now()-last_timestamp).secs > 2:
#         print("new target")
#         idx += 1
#         next = targets[idx]
#         out_msg.position = next
#         wait = True
#         last_timestamp = rospy.Time.now()
#         pub.publish(out_msg)
#
#
#
# rospy.spin()

# #
while not rospy.is_shutdown():
    # print(i)


    pub.publish(out_msg)
    # if rospy.is_shutdown():
    #     break
    # print(wait)
    if not wait:
        wait = True
        last_timestamp = rospy.Time.now()
        print(out_msg.position)
        out_msg.position = [np.random.uniform(axis0_limits[0], axis0_limits[1], 1),
                        np.random.uniform(axis1_limits[0], axis1_limits[1], 1),
                        np.random.uniform(axis2_limits[0], axis2_limits[1], 1)]
        print(out_msg.position)
    #     # print(msg)
    #
        # rospy.sleep(1)
