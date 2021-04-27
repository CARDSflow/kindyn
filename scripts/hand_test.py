import rospy
from roboy_middleware_msgs.msg import MotorCommand
import numpy as np

rospy.init_node("hand_finger_test")
topic_root = '/roboy/pinky/'
hand_msg = MotorCommand()
motorcmd_pub = rospy.Publisher(topic_root + "middleware/MotorCommand", MotorCommand,
                               queue_size=1)
rate = rospy.Rate(100)

def move_fingers(right_hand=True, targets=None):
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
    targets_relevant = []
    ids_picked = []

    for j in range(len(targets)):  # check if there are 0- values -> motor is not commanded
        if targets[j] > 0.0:
            ids_picked.append(ids_rl[j])
            targets_relevant.append(targets[j])
    hand_msg.global_id = ids_picked

    # make sure values are within the limits
    targets = clamp_values(targets_relevant, min_allowed_value=0.0, max_allowed_value=1.0)
    hand_msg.setpoint = [i * 800 for i in targets]
    motorcmd_pub.publish(hand_msg)

def clamp_values(targets, min_allowed_value, max_allowed_value):
    for t in range(len(targets)):
        if targets[t] < min_allowed_value:
            targets[t] = min_allowed_value
            print("Warning: target value too low -> clamped it")
        elif targets[t] > max_allowed_value:
            targets[t] = max_allowed_value
            print("Warning: target value too high -> clamped it")
    return targets


def move_pointer_finger(right_hand=True, intervals=11):
    ranges = [0, 1]

    for i in range(2):
        if not i % 2:
            print(right_hand, "pointer finger close")
            target_value = np.linspace(ranges[0], ranges[1], intervals)  # start with 0 (open) -> 1 (close)
            for pos in target_value:
                targets = [0, pos, pos,
                           pos]  # put the current pos value in a 4-element list TODO: what is the pointer finger
                print(targets)
                move_fingers(right_hand, targets)
        else:
            print(right_hand, "pointer finger open")
            target_value = np.linspace(ranges[1], ranges[0], intervals)  # start with 1 -> 0
            for pos in target_value:
                targets = [0, pos, pos, pos]
                print(targets)
                move_fingers(right_hand, targets)


def move_each_finger_individually(right_hand=True, intervals=11):
    ranges = [0, 1]

    for i in range(2):
        if not i % 2:
            for pos in target_value:
                pos2 = pos - 100
                pos3 = pos - 200
                pos4 = pos - 300
                targets = [pos, max(0, pos2), max(0, pos3), max(0,
                                                                pos4)]  # put the current pos value in a 4-element list TODO: what is the pointer finger
                move_fingers(right_hand, targets)
        else:
            print(right_hand, "finger by finger open")
            target_value = np.linspace(ranges[1], ranges[0], intervals)  # start with 1 -> 0
            for pos in target_value:
                pos2 = pos + 100
                pos3 = pos + 200
                pos4 = pos + 300
                targets = [pos, min(800, pos2), min(800, pos3), min(800, pos4)]
                move_fingers(right_hand, targets)


move_pointer_finger(right_hand=True, intervals=11)
# move_each_finger_individually(right_hand = True)
