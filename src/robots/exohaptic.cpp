#include "kindyn/vrpuppet.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>
#include <std_srvs/Empty.h>

using namespace std;

class Exohaptic: public cardsflow::vrpuppet::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    Exohaptic(string urdf, string cardsflow_xml){

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "exohaptic");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        spinner = new ros::AsyncSpinner(0);
        spinner->start();

        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        nh->getParam("external_robot_state", external_robot_state);
        ROS_INFO_STREAM("External robot state: " << external_robot_state);
        init(urdf,cardsflow_xml,joint_names);
        update();
        ROS_INFO_STREAM("Finished setup");
    };

    /**
     * Updates the robot model
     */
    void read(){
        update();
    };
    /**
     * Sends motor commands to the real robot
     */
    void write(){

    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::Subscriber motor_status_sub;
    ros::ServiceServer init_pose;
    ros::AsyncSpinner *spinner;
    map<string,ros::ServiceClient> motor_control_mode, motor_config;
    vector<string> body_parts = {"head", "shoulder_right", "shoulder_left"};
    map<string, vector<string>> endeffector_jointnames;

    map<string, bool> motor_status_received;
    map<string, int> init_mode, init_setpoint;
    map<string,vector<int>> real_motor_ids, sim_motor_ids, motor_type;
    map<string,vector<double>> l_offset, position, velocity, displacement;
    map<string,int> bodyPartIDs;
    map<string,bool> use_motor_config;
};

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_icecream");
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


    Exohaptic robot(urdf, cardsflow_xml);

    if (nh.hasParam("simulated")) {
      nh.getParam("simulated", robot.simulated);
    }

    ros::Rate rate(20);
    while(ros::ok()){
        robot.read();
        if (!robot.simulated)
          robot.write();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("TERMINATING...");

    return 0;
}
