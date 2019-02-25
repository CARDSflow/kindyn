# Idea
deepRoboy-feature branch of Kindyn creates interface for [Gym Environment](https://github.com/Roboy/gym-roboy) to train cable robots for specific tasks.

# Gym Services
Gym services currently only apply to [MsjPlatform robot](https://github.com/CARDSflow/kindyn/blob/deepRoboy-feature/src/robots/msj_platform.cpp)
- Gym Service provide the functionality to:
  - **Step**
     - This service message type can be seen [here]( https://github.com/Roboy/roboy_communication/blob/master/roboy_simulation_msgs/srv/GymStep.srv).
     - Step Service of takes tendon velocity and step size as argument. Apply the given tendon velocities to each motors. ForwardKinematics function of robot.cpp is called in simulation to get the resulting state of tendon velocity changes with a given step size.
     - Sets the feasible attribute to True or False if the robot is in a feasible state. Feasible state means that the state is in the reachable and collision free state.
  - **Reset**
     - Reset service message type can be seen [here]( https://github.com/Roboy/roboy_communication/blob/master/roboy_simulation_msgs/srv/GymReset.srv).
     - Reset Service does not take a request message. It resets the joint velocity and position, tendon velocity and position to initial state which is assumened as origin.
  - **Read-State**
     - Observation service message type uses the same message type as step.
     - It sents the response message as the joint angle and velocities.
  - **Set Random Goal**
     - Uses goal message type [here](https://github.com/Roboy/roboy_communication/blob/master/roboy_simulation_msgs/srv/GymGoal.srv).
     - Finds a feasible random goal for gym training environment.
     
# Current Progress
We trained an agents to learn to control MsjPlatform robot by actuating the tendons and taking feedback as the joint angles and velocities.
