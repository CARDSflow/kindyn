#include "kindyn/robot.hpp"
#include <thread>
#include "kindyn/GymServices.h"
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
    MsjPlatform(string urdf, string cardsflow_xml){

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "msj_platform");
        }
        ros::NodeHandlePtr nh(new ros::NodeHandle);
        boost::shared_ptr <ros::AsyncSpinner> spinner;
        spinner.reset(new ros::AsyncSpinner(0));
        spinner->start();

        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);

        readJointLimits();

        vector<string> joint_names; // first we retrieve the active joint names from the parameter server
        nh->getParam("joint_names", joint_names);

        init(urdf,cardsflow_xml,joint_names); // then we initialize the robot with the cardsflow xml and the active joints

        nh->getParam("external_robot_state", external_robot_state); // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state

        update();

        for(int i=0;i<NUMBER_OF_MOTORS;i++)
            l_initial[i] = l[i];


    };

    /**
     * Read joint limits of the robots which have the shoulder as part of their kinematics.
     *
     */
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
        stringstream str;
        for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
            msg.motors.push_back(i);
            double l_change = l[i]-l_initial[i];
            msg.set_points.push_back(-msjEncoderTicksPerMeter(l_change)); //
            str << l_change << "\t";
        }

        motor_command.publish(msg);
    };

    /**
     * Virtual getter functions for ball joint limits and max
     */
    double getBallJointLimit(int joint_index, int limit_index){
        return limits[joint_index][limit_index];
    }

    vector<double> getBallJointLimitVector(int joint_index){
        return limits[joint_index];
    }

    double getMaxBallJointLimit(int joint_index){
        return max[joint_index];
    }

    double getMinBallJointLimit(int joint_index){
        return min[joint_index];
    }

    bool isExternalRobotExist(){
        return external_robot_state;
    }

private:

    ros::Publisher motor_command; /// motor command publisher

    vector<double> limits[3];
    double min[3] = {0,0,-1}, max[3] = {0,0,1};
    bool external_robot_state; /// indicates if we get the robot state externally

    double l_initial[NUMBER_OF_MOTORS];

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
    vector<boost::shared_ptr<GymServices>> gymFuncs;

    for(int id = 0; id < workers; id++) {
        boost::shared_ptr<MsjPlatform> platform(new MsjPlatform(urdf, cardsflow_xml));
        cardsflow::kindyn::Robot* ref = &*platform;
        boost::shared_ptr<GymServices> gym(new GymServices(ref, id + 1 ,true));
        platforms.push_back(platform);
        gymFuncs.push_back(gym);
    }

    ros::waitForShutdown();

    ROS_INFO("TERMINATING...");

    return 0;
}
