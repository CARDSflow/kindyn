#include "kindyn/GymServices.h"

GymServices::GymServices(cardsflow::kindyn::Robot* robot, int id, bool respect_limits){
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    boost::shared_ptr <ros::AsyncSpinner> spinner;
    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();
    ROS_INFO("Gym functions");

    training_robot = robot;
    training_with_limits = respect_limits;

    gym_step = nh->advertiseService("/instance" + to_string(id) + "/gym_step", &GymServices::gymStepHandler,this);
    gym_read_state = nh->advertiseService("/instance" + to_string(id) + "/gym_read_state", &GymServices::gymReadStateHandler,this);
    gym_reset = nh->advertiseService("/instance" + to_string(id) + "/gym_reset", &GymServices::gymResetHandler,this);
    gym_goal = nh->advertiseService("/instance" + to_string(id) + "/gym_goal", &GymServices::gymGoalHandler,this);
    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");
}


bool GymServices::gymStepHandler(roboy_simulation_msgs::GymStep::Request &req,
                                  roboy_simulation_msgs::GymStep::Response &res){
    training_robot->update();

    Map<VectorXd> action(req.set_points.data()  , training_robot->number_of_cables);
    //training_robot->Ld[0]= action;  //Commanding cable velocity for simulation
    training_robot->l = action;     //Commanding cable length for hardware

    if(!training_robot->isExternalRobotExist())
        training_robot->forwardKinematics(req.step_size);

    ROS_INFO_STREAM_THROTTLE(5, "Ld = " << training_robot->Ld[0].format(fmt));

    training_robot->write();

    if(training_with_limits){
        res.feasible = isFeasible(training_robot->getBallJointLimitVector(0), training_robot->getBallJointLimitVector(1), training_robot->q[0], training_robot->q[1]);
        if(!res.feasible){
            VectorXd closestLimitPos = findClosestJointLimit(training_robot->q); //Find closest boundary point where we can teleport
            VectorXd jointVel;                                                //Robot hit the boundary, set velocity to zero.

            jointVel.resize(training_robot->number_of_dofs);
            jointVel.setZero();

            setJointAngleAndVelocity(closestLimitPos, jointVel);
        }
    }
    setResponse(training_robot->q,training_robot->qd,res);
    return true;
}

bool GymServices::gymReadStateHandler(roboy_simulation_msgs::GymStep::Request &req,
                                       roboy_simulation_msgs::GymStep::Response &res){
    setResponse(training_robot->q,training_robot->qd,res);
    if(training_with_limits)
        res.feasible = isFeasible(training_robot->getBallJointLimitVector(0),training_robot->getBallJointLimitVector(1),training_robot->q[0],training_robot->q[1]);
    return true;
}

bool GymServices::gymResetHandler(roboy_simulation_msgs::GymReset::Request &req,
                                   roboy_simulation_msgs::GymReset::Response &res){
    VectorXd jointAngle, jointVel, zeroTendonLength, zeroTendonVels;

    training_robot->setIntegrationTime(0.0);

    jointAngle.resize(training_robot->number_of_dofs);
    jointVel.resize(training_robot->number_of_dofs);
    zeroTendonLength.resize(training_robot->number_of_cables);
    zeroTendonVels.resize(training_robot->number_of_cables);

    zeroTendonLength.setZero();
    zeroTendonVels.setZero();

    jointAngle.setZero();
    jointVel.setZero();

    setJointAngleAndVelocity(jointAngle, jointVel);

    training_robot->setMotorCableLengths(zeroTendonLength);
    training_robot->setMotorCableVelocities(zeroTendonVels);


    for(int i=0; i< training_robot->number_of_dofs; i++ ){
        res.q.push_back(training_robot->q[i]);
        res.qdot.push_back(training_robot->qd[i]);
    }

    return true;
}

bool GymServices::gymGoalHandler(roboy_simulation_msgs::GymGoal::Request &req,
                                  roboy_simulation_msgs::GymGoal::Response &res){
    bool not_feasible = true;
    VectorXd target_q;
    target_q.resize(training_robot->number_of_dofs);
    srand(static_cast<unsigned int>(clock()));
    if(training_with_limits) {
        while (not_feasible) {
            for (int i = 0; i < training_robot->number_of_dofs; i++) {
                target_q[i] = training_robot->getMinBallJointLimit(i) + rand() / (static_cast<float> (RAND_MAX /  (training_robot->getMaxBallJointLimit(i) - training_robot->getMinBallJointLimit(i))));
            }
            not_feasible = !isFeasible(training_robot->getBallJointLimitVector(0), training_robot->getBallJointLimitVector(1), target_q[0], target_q[1]);
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

int GymServices::isFeasible(vector<double> limits_x, vector<double> limits_y, double testx, double testy){
    int i, j, c = 0;
    for (i = 0, j = limits_x.size()-1; i < limits_x.size(); j = i++) {
        if ( ((limits_y[i]>testy) != (limits_y[j]>testy)) &&
             (testx < (limits_x[j]-limits_x[i]) * (testy-limits_y[i]) / (limits_y[j]-limits_y[i]) + limits_x[i]) )
            c = !c;
    }
    return c;
}


///sets the gymstep function response
void GymServices::setResponse(VectorXd jointAngles, VectorXd jointVel,
                               roboy_simulation_msgs::GymStep::Response &res){
    for(int i=0; i< training_robot->number_of_dofs; i++ ){
        res.q.push_back(jointAngles[i]);
        res.qdot.push_back(jointVel[i]);
    }
}

///sets the given joint angle and veloctiy  for each joint
void GymServices::setJointAngleAndVelocity(VectorXd jointAngles, VectorXd jointVels){
    training_robot->setJointPositions(jointAngles);
    training_robot->setJointVelocities(jointVels);
}

///finds the closest limit point when the robot is in infeasible state
VectorXd GymServices::findClosestJointLimit(VectorXd q){
    VectorXd closestLimit;
    closestLimit.resize(training_robot->number_of_dofs);
    double smallestDistance = numeric_limits<double>::max();
    
    for(int i=0; i < training_robot->getBallJointLimitVector(0).size(); i++){
        VectorXd jointAngle = q;
        VectorXd jointLimits;
        jointLimits.resize(training_robot->number_of_dofs);
        jointLimits << training_robot->getBallJointLimit(0,i) , training_robot->getBallJointLimit(1, i), jointAngle[2];  //FIXME right now hard coded for msjplatform, make it general
        double distance = (jointAngle - jointLimits).norm();
        if (distance < smallestDistance){
            smallestDistance = distance;
            closestLimit = jointLimits;
        }
    }
    return closestLimit;
}