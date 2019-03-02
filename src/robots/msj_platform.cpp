#include "kindyn/robot.hpp"
#include <thread>
#include "../../include/kindyn/gymFunctions.h"
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_simulation_msgs/GymStep.h>
#include <roboy_simulation_msgs/GymReset.h>
#include <roboy_simulation_msgs/GymGoal.h>
#include <common_utilities/CommonDefinitions.h>
#include <stdlib.h> /* atoi*/
#include <limits>


#define NUMBER_OF_MOTORS 8
#define SPINDLERADIUS 0.00575
#define msjMeterPerEncoderTick(encoderTicks) (((encoderTicks)/(4096.0)*(2.0*M_PI*SPINDLERADIUS)))
#define msjEncoderTicksPerMeter(meter) ((meter)*(4096.0)/(2.0*M_PI*SPINDLERADIUS))

using namespace std;
using namespace Eigen;

class MsjPlatform: public cardsflow::kindyn::Robot{
public:

    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    MsjPlatform(string urdf, string cardsflow_xml, int id){

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "msj_platform");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        spinner.reset(new ros::AsyncSpinner(0));
        spinner->start();
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);


        //initService(id);

        //readJointLimits();

        vector<string> joint_names; // first we retrieve the active joint names from the parameter server
        nh->getParam("joint_names", joint_names);

        init(urdf,cardsflow_xml,joint_names); // then we initialize the robot with the cardsflow xml and the active joints

        nh->getParam("external_robot_state", external_robot_state); // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state

        update();

        for(int i=0;i<NUMBER_OF_MOTORS;i++)
            l_offset[i] = l[i];

        gymFunctions gf(id);
    };

    ///Open AI Gym services
    void initService(int id){
        gym_step = nh->advertiseService("/instance" + to_string(id) + "/gym_step", &MsjPlatform::GymStepService,this);
        gym_read_state = nh->advertiseService("/instance" + to_string(id) + "/gym_read_state", &MsjPlatform::GymReadStateService,this);
        gym_reset = nh->advertiseService("/instance" + to_string(id) + "/gym_reset", &MsjPlatform::GymResetService,this);
        gym_goal = nh->advertiseService("/instance" + to_string(id) + "/gym_goal", &MsjPlatform::GymGoalService,this);
    }

    void readJointLimits(){
        // Get the limits of joints
        string path = ros::package::getPath("robots");
        path+="/msj_platform/joint_limits.txt";
        FILE*       file = fopen(path.c_str(),"r");
        if (NULL != file) {
            fscanf(file, "%*[^\n]\n");
            float qx,qy;
            int i =0;
            while(fscanf(file,"%f %f\n",&qx,&qy) == 2){
                limits[0].push_back(qx);
                limits[1].push_back(qy);
                i++;
            }
            printf("read %d joint limit values\n", i);
        }else{
            cout << "could not open " << path << endl;
        }
        for(int i=0;i<limits[0].size();i++){
            if(limits[0][i]<min[0])
                min[0] = limits[0][i];
            if(limits[1][i]<min[1])
                min[1] = limits[1][i];
            if(limits[0][i]>max[0])
                max[0] = limits[0][i];
            if(limits[1][i]>max[1])
                max[1] = limits[1][i];
        }
    }
    /**
     * Updates the robot model and if we do not use gazebo for simulation, we integrate using the forwardKinematics function
     * with a small step length
     */
    void read(){
        update();
        if(!external_robot_state)
            forwardKinematics(0.01);
    };

    /**
     * Sends motor commands to the real robot
     */
    void write(){
        roboy_middleware_msgs::MotorCommand msg;
        msg.id = 5;
//        stringstream str;
//        for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
//            msg.motors.push_back(i);
//            double l_change = l[i]-l_offset[i];
//            msg.set_points.push_back(-msjEncoderTicksPerMeter(l_change)); //
//            str << l_change << "\t";
//        }
        for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
            msg.motors.push_back(i);
//            double l_change = l_target[i]-l_offset[i];
            msg.set_points.push_back(l_target[i]); //
//            str << l_target[i] << "\t";
        }
//        for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
//                msg.motors.push_back(i);
//                msg.set_points.push_back(myoMuscleEncoderTicksPerMeter(Ld[0][i]));
//        }
//		ROS_INFO_STREAM_THROTTLE(1,str.str());

        motor_command.publish(msg);
    };

    int isFeasible(vector<double> limits_x, vector<double> limits_y, double testx, double testy){
        int i, j, c = 0;
        for (i = 0, j = limits_x.size()-1; i < limits_x.size(); j = i++) {
            if ( ((limits_y[i]>testy) != (limits_y[j]>testy)) &&
                 (testx < (limits_x[j]-limits_x[i]) * (testy-limits_y[i]) / (limits_y[j]-limits_y[i]) + limits_x[i]) )
                c = !c;
        }
        return c;
    }
    bool GymGoalService(roboy_simulation_msgs::GymGoal::Request &req,
                        roboy_simulation_msgs::GymGoal::Response &res){
        bool not_feasible = true;
        float q0= 0.0,q1= 0.0,q2 = 0.0;
        srand(static_cast<unsigned int>(clock()));
        while(not_feasible) {
            q0 = min[0] + static_cast<float> (rand() /(static_cast<float> (RAND_MAX/(max[0]-min[0]))));
            q1 = min[1] + static_cast<float> (rand() /(static_cast<float> (RAND_MAX/(max[1]-min[1]))));
            q2 = min[2] + static_cast<float> (rand() /(static_cast<float> (RAND_MAX/(max[2]-min[2]))));
            not_feasible = !isFeasible(limits[0], limits[1], q0, q1);
        }

        q_target << static_cast<double> (q0), static_cast<double> (q1), static_cast<double> (q2);

        update();
        res.q.push_back(q0);
        res.q.push_back(q1);
        res.q.push_back(q2);

        return true;

    }

    ///find the closest limit point when the robot is in infeasible state
    VectorXd findClosestJointLimit(double q0, double q1, double q3){
        VectorXd closestLimit;
        closestLimit.resize(number_of_dofs);
        closestLimit.setZero();

        double smallestDistance = numeric_limits<double>::max();

        for(int i=0; i < limits[0].size(); i++){
            VectorXd jointAngle = Vector2d::Zero(), jointLimits = Vector2d::Zero();
            jointAngle << q0, q1;
            jointLimits << limits[0][i] ,limits[1][i];
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

    ///set the given joint angle and veloctiy  for each joint
    void setJointAngleAndVelocity(VectorXd jointAngles, VectorXd jointVel){
        for(int i=0; i< number_of_dofs; i++){
            joint_state[i][1] = jointVel[i];		//Setting velocity to zero send the robot to origin..
            joint_state[i][0] = jointAngles[i];		//Position of ith joint
            q[i] = jointAngles[i];
            qd[i] = jointVel[i];
        }
    }

    ///set the gymstep function response
    void setResponse(VectorXd jointAngles,VectorXd jointVel,roboy_simulation_msgs::GymStep::Response &res){
        for(int i=0; i< number_of_dofs; i++ ){
            res.q.push_back(jointAngles[i]);
            res.qdot.push_back(jointVel[i]);
        }
    }

    ///Return only qdot, q and feasibility to gym environment
    bool GymReadStateService(roboy_simulation_msgs::GymStep::Request &req,
                        roboy_simulation_msgs::GymStep::Response &res){

        setResponse(q,qd,res);
        res.feasible = isFeasible(limits[0],limits[1],q[0],q[1]);
        return true;
    }

    bool GymStepService(roboy_simulation_msgs::GymStep::Request &req,
                        roboy_simulation_msgs::GymStep::Response &res){

        update();
        for(int i=0; i< number_of_cables; i++){
            //Set the commanded tendon velocity from RL agent to simulation
            Ld[0][i] = req.set_points[i];
        }
        if(!external_robot_state)
            forwardKinematics(req.step_size);

        ROS_INFO_STREAM_THROTTLE(5, "Ld = " << Ld[0].format(fmt));
        write();
        res.feasible = isFeasible(limits[0],limits[1],q[0],q[1]);
        if(!res.feasible){
            VectorXd closestLimit = findClosestJointLimit(q[0],q[1],q[2]); //Find closest boundary point where we can teleport
            VectorXd jointVel;                                             //We hit the boundary so zero velocity.

            jointVel.resize(number_of_dofs);
            jointVel.setZero();

            setJointAngleAndVelocity(closestLimit, jointVel);
        }
        setResponse(q,qd,res);
        return true;
    }
    bool GymResetService(roboy_simulation_msgs::GymReset::Request &req,
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

	 	update();

        for(int i=0; i< number_of_dofs; i++ ){
        	res.q.push_back(q[i]);
        	res.qdot.push_back(qd[i]);
        }
        return true;
    }

private:

    bool external_robot_state; /// indicates if we get the robot state externally
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher

    ros::ServiceServer gym_step; //OpenAI Gym training environment step function, ros service instance
    ros::ServiceServer gym_read_state; //OpenAI Gym training environment observation function, returns q, qdot and feasbility
    ros::ServiceServer gym_reset; //OpenAI Gym training environment reset function, ros service instance
    ros::ServiceServer gym_goal; //OpenAI Gym training environment sets new feasible goal function, ros service instance

    vector<double> limits[3];
    double l_offset[NUMBER_OF_MOTORS];
    boost::shared_ptr <ros::AsyncSpinner> spinner;
    double min[3] = {0,0,-1}, max[3] = {0,0,1};
};

/**
 * controller manager update thread. Here you can define how fast your controllers should run
 * @param cm pointer to the controller manager
 */
void update(controller_manager::ControllerManager *cm) {
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(100); // changing this value affects the control speed of your running controllers
    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        cm->update(time, period);
        prev_time = time;
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "cardsflow_example_robot");
    }
    ros::NodeHandle nh;
    string urdf, cardsflow_xml;
    if(nh.hasParam("urdf_file_path") && nh.hasParam("cardsflow_xml")) {
        nh.getParam("urdf_file_path", urdf);
        nh.getParam("cardsflow_xml", cardsflow_xml);
    }else {
        ROS_FATAL("USAGE: rosrun kindyn test_robot path_to_urdf path_to_viapoints_xml");
        return 1;
    }
    ROS_INFO("\nurdf file path: %s\ncardsflow_xml %s", urdf.c_str(), cardsflow_xml.c_str());

    int workers = atoi( argv[1]);
    cout << "\nNUMBER OF WORKERS " << workers << endl;

    vector<boost::shared_ptr<MsjPlatform>> platforms;
    for(int id = 0; id < workers; id++) {
        boost::shared_ptr<MsjPlatform> platform(new MsjPlatform(urdf, cardsflow_xml, id + 1));
        platforms.push_back(platform);
    }


    ros::waitForShutdown();

    ROS_INFO("TERMINATING...");

    return 0;
}
