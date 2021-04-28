import rospy
from roboy_middleware_msgs.msg import MotorCommand
import numpy as np

rospy.init_node("hand_finger_test")
topic_root = '/roboy/pinky/'
hand_msg = MotorCommand()
motorcmd_pub = rospy.Publisher(topic_root + "middleware/MotorCommand", MotorCommand,
                               queue_size=1)
rate = rospy.Rate(100)

def move_fingers(right_hand=True, targets=None, finger = "INDEX"):
    """
    :param target: array of finger target values between 0 (open hand) and 1 (closed hand)
    right_hand: True (right hand); False (left hand)
    left_ids = [38, 39, 40, 41]
    right_ids = [42, 43, 44, 45]
    """
    if targets is None:
        targets = [0, 0, 0, 0]

    # just take the ids with actual values (>0)
    right_ids = [42, 42, 44, 45]
    left_ids = [38, 39, 40, 41]
    if right_hand:
        ids_rl = right_ids
    else:
        ids_rl = left_ids

    if finger == "INDEX":
        id_picked = ids_rl[0]
    elif finger == "MIDDLE":
        id_picked = ids_rl[1]
    elif finger == "RING":
        id_picked = ids_rl[2]
    elif finger == "THUMB":
        id_picked = ids_rl[3]
    elif finger == "ALL":
        id_picked = ids_rl
    else:
        rospy.logwarn("Warning: No finger chosen to move")
        id_picked = []
        targets = []

    hand_msg.global_id = id_picked

    # make sure values are within the limits
    targets = clamp_values(targets, min_allowed_value=0.0, max_allowed_value=1.0)
    targets_scaled = [i * 800 for i in targets]
    targets_scaled = clamp_values(targets, min_allowed_value=0.0, max_allowed_value=800.0) # 2nd security check
    rospy.loginfo("targets_scaled: " + str(targets_scaled))
    hand_msg.setpoint = targets_scaled  # multiply all values in the array with 800
    motorcmd_pub.publish(hand_msg)

def clamp_values(targets, min_allowed_value, max_allowed_value):
    for t in range(len(targets)):
        if targets[t] < min_allowed_value:
            targets[t] = min_allowed_value
            rospy.logwarn("Warning: target value too low -> clamped it")
        elif targets[t] > max_allowed_value:
            targets[t] = max_allowed_value
            rospy.logwarn("Warning: target value too high -> clamped it")
    return targets

def move_single_finger(right_hand=True, intervals=11, ranges=[0, 1], finger="INDEX"):
    """
    pos. values for fingers:  "INDEX" / "MIDDLE" / "RING" / "THUMB"
    """
    for i in range(2):
        if not i % 2:
            rospy.loginfo(str(right_hand) + " " + finger + " finger close")
            target_value = np.linspace(ranges[0], ranges[1], intervals)  # start with 0 (open) -> 1 (close)
            for pos in target_value:
                targets = [pos]  # set the current finger goal position
                rospy.loginfo(targets)
                move_fingers(right_hand, targets, finger)
        else:
            rospy.loginfo(str(right_hand) + " " + finger + " pointer finger open")
            target_value = np.linspace(ranges[1], ranges[0], intervals)  # start with 1 -> 0
            for pos in target_value:
                targets = [pos]
                rospy.loginfo(targets)
                move_fingers(right_hand, targets, finger)


def move_fingers_choreography(right_hand=True, intervals=11, ranges=[0, 1]):
    # move all fingers except index finger, such that index finger is pointing somewhere
    for i in range(2):
        if not i % 2:
            rospy.loginfo(str(right_hand) + " fingers close")
            target_value = np.linspace(ranges[0], ranges[1], intervals)  # start with 0 -> 1
            for pos in target_value:
                targets = [0, pos, pos, pos]  # put the current pos value in a 4-element list
                rospy.loginfo(targets)
                move_fingers(right_hand, targets, "ALL")
        else:
            rospy.loginfo(str(right_hand) + " fingers open")
            target_value = np.linspace(ranges[1], ranges[0], intervals)  # start with 1 -> 0
            for pos in target_value:
                targets = [0, pos, pos, pos]  # put the current pos value in a 4-element list
                rospy.loginfo(targets)
                move_fingers(right_hand, targets, "ALL")

#move_single_finger(right_hand=True, intervals=11, ranges = [0, 1], finger = "INDEX")
move_fingers_choreography(right_hand=True, intervals=11, ranges=[0, 1])
