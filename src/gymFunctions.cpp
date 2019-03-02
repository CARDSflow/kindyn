//
// Created by barisyazici on 2/25/19.
//

#include "../include/kindyn/gymFunctions.h"
#include <iostream>


gymFunctions::gymFunctions(int id){

    cout << "gym FUnctions"<<endl;
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "Gym functions", ros::init_options::NoSigintHandler);
    }
    ros::NodeHandle nh;
    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();
    ROS_INFO("Gym functions");

    gym_step = nh.advertiseService("/instance" + to_string(id) + "/gym_step", &gymFunctions::GymStepService,this);
    gym_read_state = nh.advertiseService("/instance" + to_string(id) + "/gym_read_state", &gymFunctions::GymReadStateService,this);
    gym_reset = nh.advertiseService("/instance" + to_string(id) + "/gym_reset", &gymFunctions::GymResetService,this);

    ros::waitForShutdown();
}

///set the gymstep function response
void gymFunctions::setResponse(VectorXd jointAngles, VectorXd jointVel,
                               roboy_simulation_msgs::GymStep::Response &res){
    for(int i=0; i< number_of_dofs; i++ ){
        res.q.push_back(jointAngles[i]);
        res.qdot.push_back(jointVel[i]);
    }
}

bool gymFunctions::GymStepService(roboy_simulation_msgs::GymStep::Request &req,
                                  roboy_simulation_msgs::GymStep::Response &res){

    for(int i=0; i< number_of_cables; i++){
        //Set the commanded tendon velocity from RL agent to simulation
        Ld[0][i] = req.set_points[i];
    }
    //if(!external_robot_state)
        forwardKinematics(req.step_size);

    ROS_INFO_STREAM_THROTTLE(5, "Ld = " << Ld[0].format(fmt));

    setResponse(q,qd,res);
    return true;
}

bool gymFunctions::GymReadStateService(roboy_simulation_msgs::GymStep::Request &req,
                         roboy_simulation_msgs::GymStep::Response &res){

    setResponse(q,qd,res);
    return true;
}

bool gymFunctions::GymResetService(roboy_simulation_msgs::GymReset::Request &req,
                     roboy_simulation_msgs::GymReset::Response &res){
    integration_time = 0.0;
    VectorXd jointAngle, jointVel;
    jointAngle.resize(number_of_dofs);
    jointVel.resize(number_of_dofs);

    jointAngle.setZero();
    jointVel.setZero();

    setJointAngleAndVelocity(jointAngle, jointVel);

    for(int i=0; i< number_of_cables; i++){
        //Set the commanded tendon velocity from RL agent to simulation
        motor_state[i][0] = 0.0; 	//Length of the ith cable
        motor_state[i][1] = 0.0;	//Velocity of the ith cable
    }

    for(int i=0; i< number_of_dofs; i++ ){
        res.q.push_back(q[i]);
        res.qdot.push_back(qd[i]);
    }

    return true;
}

///set the given joint angle and veloctiy  for each joint
void gymFunctions::setJointAngleAndVelocity(VectorXd jointAngles, VectorXd jointVel){
    for(int i=0; i< number_of_dofs; i++){
        joint_state[i][1] = jointVel[i];		//Setting velocity to zero send the robot to origin..
        joint_state[i][0] = jointAngles[i];		//Position of ith joint
        q[i] = jointAngles[i];
        qd[i] = jointVel[i];
    }
}