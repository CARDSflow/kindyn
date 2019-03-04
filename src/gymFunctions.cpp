#include "../include/kindyn/gymFunctions.h"

gymFunctions::gymFunctions(int id, cardsflow::kindyn::Robot* robot, bool respect_limits){

    training_robot = robot;
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "Gym functions", ros::init_options::NoSigintHandler);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();
    ROS_INFO("Gym functions");

    training_with_limits = respect_limits;
    
    gym_step = nh->advertiseService("/instance" + to_string(id) + "/gym_step", &gymFunctions::GymStepService,this);
    gym_read_state = nh->advertiseService("/instance" + to_string(id) + "/gym_read_state", &gymFunctions::GymReadStateService,this);
    gym_reset = nh->advertiseService("/instance" + to_string(id) + "/gym_reset", &gymFunctions::GymResetService,this);
    gym_goal = nh->advertiseService("/instance" + to_string(id) + "/gym_goal", &gymFunctions::GymGoalService,this);
    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");
}


bool gymFunctions::GymStepService(roboy_simulation_msgs::GymStep::Request &req,
                                  roboy_simulation_msgs::GymStep::Response &res){
    training_robot->update();
    for(int i=0; i< training_robot->number_of_cables; i++){
        //Set the commanded tendon velocity from RL agent to simulation
        training_robot->Ld[0][i] = req.set_points[i];
    }

    if(!training_robot->getExternalRobotState())
        training_robot->forwardKinematics(req.step_size);

    ROS_INFO_STREAM_THROTTLE(5, "Ld = " << training_robot->Ld[0].format(fmt));

    if(training_with_limits){
        res.feasible = isFeasible(training_robot->getLimitVector(0), training_robot->getLimitVector(1), training_robot->q[0], training_robot->q[1]);
        if(!res.feasible){
            VectorXd closestLimit = findClosestJointLimit(training_robot->q[0],training_robot->q[1],training_robot->q[2]); //Find closest boundary point where we can teleport
            VectorXd jointVel;                                             //We hit the boundary so zero velocity.

            jointVel.resize(training_robot->number_of_dofs);
            jointVel.setZero();

            setJointAngleAndVelocity(closestLimit, jointVel);
        }
    }
    setResponse(training_robot->q,training_robot->qd,res);
    return true;
}

bool gymFunctions::GymReadStateService(roboy_simulation_msgs::GymStep::Request &req,
                                       roboy_simulation_msgs::GymStep::Response &res){

    setResponse(training_robot->q,training_robot->qd,res);
    if(training_with_limits)
        res.feasible = isFeasible(training_robot->getLimitVector(0),training_robot->getLimitVector(1),training_robot->q[0],training_robot->q[1]);
    return true;
}

bool gymFunctions::GymResetService(roboy_simulation_msgs::GymReset::Request &req,
                                   roboy_simulation_msgs::GymReset::Response &res){

    training_robot->setIntegrationTime(0.0);

    VectorXd jointAngle, jointVel;
    jointAngle.resize(training_robot->number_of_dofs);
    jointVel.resize(training_robot->number_of_dofs);

    jointAngle.setZero();
    jointVel.setZero();

    setJointAngleAndVelocity(jointAngle, jointVel);

    for(int i=0; i< training_robot->number_of_cables; i++){
        training_robot->setMotorState(i, 0, 0.0);   //Length of the ith cable
        training_robot->setMotorState(i, 1, 0.0);   //Velocity of the ith cable
    }

    for(int i=0; i< training_robot->number_of_dofs; i++ ){
        res.q.push_back(training_robot->q[i]);
        res.qdot.push_back(training_robot->qd[i]);
    }

    return true;
}

bool gymFunctions::GymGoalService(roboy_simulation_msgs::GymGoal::Request &req,
                                  roboy_simulation_msgs::GymGoal::Response &res){
    bool not_feasible = true;
    VectorXd target_q;
    target_q.resize(training_robot->number_of_dofs);
    srand(static_cast<unsigned int>(clock()));
    if(training_with_limits) {
        while (not_feasible) {
            for (int i = 0; i < training_robot->number_of_dofs; i++) {
                target_q[i] = training_robot->getMin(i) + rand() / (static_cast<float> (RAND_MAX /  (training_robot->getMax(i) - training_robot->getMin(i))));
            }
            not_feasible = !isFeasible(training_robot->getLimitVector(0), training_robot->getLimitVector(1), target_q[0], target_q[1]);
        }
    }
    else {  //Create random goals with respecting model limits
        for (int i = 0; i < training_robot->number_of_dofs; i++) {
            target_q[i] = training_robot->q_min[i] + rand() / (static_cast<float> (RAND_MAX /  (training_robot->q_max[i] - training_robot->q_min[i])));
        }
    }

    training_robot->q_target = target_q;
    training_robot->update();

    for (int i = 0; i < training_robot->number_of_dofs; i++) {
        res.q.push_back(target_q[i]);
    }
    return true;

}

int gymFunctions::isFeasible(vector<double> limits_x, vector<double> limits_y, double testx, double testy){
    int i, j, c = 0;
    for (i = 0, j = limits_x.size()-1; i < limits_x.size(); j = i++) {
        if ( ((limits_y[i]>testy) != (limits_y[j]>testy)) &&
             (testx < (limits_x[j]-limits_x[i]) * (testy-limits_y[i]) / (limits_y[j]-limits_y[i]) + limits_x[i]) )
            c = !c;
    }
    return c;
}


///sets the gymstep function response
void gymFunctions::setResponse(VectorXd jointAngles, VectorXd jointVel,
                               roboy_simulation_msgs::GymStep::Response &res){
    for(int i=0; i< training_robot->number_of_dofs; i++ ){
        res.q.push_back(jointAngles[i]);
        res.qdot.push_back(jointVel[i]);
    }
}

///sets the given joint angle and veloctiy  for each joint
void gymFunctions::setJointAngleAndVelocity(VectorXd jointAngles, VectorXd jointVel){
    for(int i=0; i< training_robot->number_of_dofs; i++){
        training_robot->setJointState(i,1,jointVel[i]);
        training_robot->setJointState(i,0,jointAngles[i]);
        training_robot->qd[i] = jointVel[i];
        training_robot->q[i] = jointAngles[i];
    }
}

///finds the closest limit point when the robot is in infeasible state
VectorXd gymFunctions::findClosestJointLimit(double q0, double q1, double q3){

    VectorXd closestLimit;
    closestLimit.resize(training_robot->number_of_dofs);
    closestLimit.setZero();

    double smallestDistance = numeric_limits<double>::max();

    for(int i=0; i < training_robot->getLimitVector(0).size(); i++){
        VectorXd jointAngle = Vector2d::Zero(), jointLimits = Vector2d::Zero();
        jointAngle << q0, q1;
        jointLimits << training_robot->getLimit(0,i) ,training_robot->getLimit(1, i);
        double distance = (jointAngle - jointLimits).norm();
        if (distance < smallestDistance){
            smallestDistance = distance;
            closestLimit[0] = jointLimits[0];
            closestLimit[1] = jointLimits[1];
            closestLimit[2] = q3;
        }
    }
    return closestLimit;
}
