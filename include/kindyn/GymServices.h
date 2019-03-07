#ifndef PROJECT_GYMSERVICES_H
#define PROJECT_GYMSERVICES_H

#include "kindyn/robot.hpp"
#include <roboy_simulation_msgs/GymStep.h>
#include <roboy_simulation_msgs/GymReset.h>
#include <roboy_simulation_msgs/GymGoal.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class GymServices {
public:

    GymServices(cardsflow::kindyn::Robot* training_robot, int id = 1, bool respect_limits = false);

    bool gymStepHandler(roboy_simulation_msgs::GymStep::Request &req,
                        roboy_simulation_msgs::GymStep::Response &res);

    bool gymReadStateHandler(roboy_simulation_msgs::GymStep::Request &req,
                             roboy_simulation_msgs::GymStep::Response &res);

    bool gymResetHandler(roboy_simulation_msgs::GymReset::Request &req,
                         roboy_simulation_msgs::GymReset::Response &res);

    bool gymGoalHandler(roboy_simulation_msgs::GymGoal::Request &req,
                        roboy_simulation_msgs::GymGoal::Response &res);


private:

    Eigen::IOFormat fmt; /// formator for terminal printouts
    cardsflow::kindyn::Robot* training_robot;
    ros::ServiceServer gym_step; /// OpenAI Gym training environment step function, ros service instance
    ros::ServiceServer gym_read_state; ///OpenAI Gym training environment observation function, returns q, qdot and feasibility
    ros::ServiceServer gym_reset; /// OpenAI Gym training environment reset function, ros service instance
    ros::ServiceServer gym_goal; /// OpenAI Gym training environment sets new feasible goal function, ros service instance

    VectorXd last_tendon_length;
    bool training_with_limits;
    void setResponse(VectorXd jointAngles,VectorXd jointVel,roboy_simulation_msgs::GymStep::Response &res);
    void setJointAngleAndVelocity(VectorXd jointAngles, VectorXd jointVel);
    VectorXd findClosestJointLimit(VectorXd q);
    int isFeasible(vector<double> limits_x, vector<double> limits_y, double testx, double testy);
};


#endif //PROJECT_GYMSERVICES_H
