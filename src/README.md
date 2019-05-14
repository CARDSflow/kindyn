This folder contains the control files for CARDSFlow robots.

Doxygen documentation is found in documentation/html/index.html.

Before the Python scripts related to the Rickshaw can be run, the Rickshaw robot must be launched.
This can at the time of writing be done with: \
`roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='joint_hip_left joint_hip_right joint_wheel_right joint_wheel_back joint_pedal spine_joint joint_wheel_left joint_front joint_pedal_right joint_pedal_left elbow_right_rot1 joint_foot_left joint_knee_right joint_knee_left joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left_rot1 elbow_left_rot0 left_wrist_0 left_wrist_1 right_shoulder_axis0 right_shoulder_axis2 right_shoulder_axis1 elbow_right_rot0 right_wrist_0 right_wrist_1 head_axis0 head_axis1 head_axis2'`

#roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='spine_joint sphere_head_axis0 sphere_head_axis1 sphere_head_axis2 joint_hip_left joint_knee_left joint_foot_left joint_hip_right joint_knee_right joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left wrist_left_sphere_axis0 wrist_left_sphere_axis1 wrist_left_sphere_axis2 right_shoulder_axis0 right_shoulder_axis1 right_shoulder_axis2 elbow_right wrist_right_sphere_axis0 wrist_right_sphere_axis1 wrist_right_sphere_axis2 little_right_joint0 ring_right_joint0 middle_right_joint0 index_right_joint0 thumb_right_joint0 little_right_joint1 ring_right_joint1 middle_right_joint1 index_right_joint1 thumb_right_joint1 little_right_joint2 ring_right_joint2 middle_right_joint2 index_right_joint2 thumb_right_joint2 little_left_joint0 ring_left_joint0 middle_left_joint0 index_left_joint0 thumb_left_joint0 little_left_joint1 ring_left_joint1 middle_left_joint1 index_left_joint1 thumb_left_joint1 little_left_joint2 ring_left_joint2 middle_left_joint2 index_left_joint2 thumb_left_joint2 joint_wheel_left joint_wheel_right joint_front joint_wheel_back joint_pedal joint_pedal_right joint_pedal_left'

#roslaunch kindyn robot.launch robot_name:=rikshaw start_controllers:='joint_hip_left joint_knee_left joint_foot_left joint_hip_right joint_knee_right joint_foot_right left_shoulder_axis0 left_shoulder_axis1 left_shoulder_axis2 elbow_left wrist_left_sphere_axis0 right_shoulder_axis0 right_shoulder_axis1 right_shoulder_axis2 elbow_right wrist_right_sphere_axis0 joint_front joint_pedal joint_pedal_right joint_pedal_left'


Below is a content description of each subfolder:

**controller** \
This folder contains myo-muscle control files.

**images** \
This folder contains graphs and images relevant to steering and pedaling control of Roboy.

**pedaling** \
This folder contains files relevant to Roboy pedaling the rickshaw.

**robots** \
This folder contains the CARDSFlow implementation of each robot.

**steering**  \
This folder contains files relevant to Roboy steering the rickshaw.

**utilities** \
This folder contains files used in the Rickshaw project.

**Todo:** \
Add usage information? \
Add description of files inside folders \
Add prerequisites \
Add installation guide
