#include "kindyn/vrpuppet.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace std;

class RightArmTestbed: public cardsflow::vrpuppet::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    RightArmTestbed(string urdf, string cardsflow_xml){

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "right_arm_testbed");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        spinner = new ros::AsyncSpinner(0);
        spinner->start();

        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        nh->getParam("external_robot_state", external_robot_state);
        ROS_INFO_STREAM("External robot state: " << external_robot_state);
        init(urdf,cardsflow_xml,joint_names);
        listener.reset(new tf::TransformListener);
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
        tf::StampedTransform right_hand,left_hand, right_elbow, left_elbow, right_upper_arm, left_upper_arm;
        bool left_hand_available = true, right_hand_available = true;
        try{
            listener->lookupTransform("0_1", "0_4",
                                     ros::Time(0), right_hand);
            listener->lookupTransform("0_1", "0_3",
                                      ros::Time(0), right_elbow);
            listener->lookupTransform("0_2", "0_3",
                                      ros::Time(0), right_upper_arm);
        }
        catch (tf::TransformException ex){
            ROS_ERROR_THROTTLE(5,"%s",ex.what());
            left_hand_available = false;
        }
        try{
            listener->lookupTransform("0_1", "0_7",
                                      ros::Time(0), left_hand);
            listener->lookupTransform("0_1", "0_6",
                                      ros::Time(0), left_elbow);
            listener->lookupTransform("0_5", "0_6",
                                      ros::Time(0), left_upper_arm);
        }
        catch (tf::TransformException ex){
            ROS_ERROR_THROTTLE(5,"%s",ex.what());
            right_hand_available = false;
        }

//        if(left_hand_available) {
//            {
//                roboy_middleware_msgs::InverseKinematicsMultipleFrames msg;
//                msg.request.endeffector = "hand_left";
//                msg.request.target_frames = {"hand_left", "elbow_left_link1"};
//                msg.request.type = 1;
//                geometry_msgs::Pose pose;
//                pose.position.x = left_hand.getOrigin().x();
//                pose.position.y = left_hand.getOrigin().y();
//                pose.position.z = left_hand.getOrigin().z();
//                pose.orientation.w = 1;
//                msg.request.poses.push_back(pose);
//                pose.position.x = left_elbow.getOrigin().x();
//                pose.position.y = left_elbow.getOrigin().y();
//                pose.position.z = left_elbow.getOrigin().z();
//                pose.orientation.w = 1;
//                msg.request.poses.push_back(pose);
//                msg.request.weights = {0.9, 0.9};
//
//                if (InverseKinematicsMultipleFramesService(msg.request, msg.response)) {
//                    for (int i = 0; i < msg.response.joint_names.size(); i++) {
//                        q_target[joint_index[msg.response.joint_names[i]]] = (0.3 * msg.response.angles[i] + 0.7 *
//                                                                                                             q_target[joint_index[msg.response.joint_names[i]]]);
//                    }
//                }
//            }
//            tf::StampedTransform upper_arm_left, lower_arm_left;
//            try{
//                listener->lookupTransform("world", "upper_arm_left",
//                                          ros::Time(0), upper_arm_left);
//                listener->lookupTransform("world", "lower_arm_left",
//                                          ros::Time(0), lower_arm_left);
//            }
//            catch (tf::TransformException ex){
//                ROS_ERROR_THROTTLE(5,"%s",ex.what());
//            }
//        }
//        if(right_hand_available) {
//            roboy_middleware_msgs::InverseKinematicsMultipleFrames msg;
//            msg.request.endeffector = "hand_right";
//            msg.request.target_frames = {"hand_right", "elbow_right_link1"};
//            msg.request.type = 1;
//            geometry_msgs::Pose pose;
//            pose.position.x = right_hand.getOrigin().x();
//            pose.position.y = right_hand.getOrigin().y();
//            pose.position.z = right_hand.getOrigin().z();
//            pose.orientation.w = 1;
//            msg.request.poses.push_back(pose);
//            pose.position.x = right_elbow.getOrigin().x();
//            pose.position.y = right_elbow.getOrigin().y();
//            pose.position.z = right_elbow.getOrigin().z();
//            pose.orientation.w = 1;
//            msg.request.poses.push_back(pose);
//            msg.request.weights = {0.9, 0.9};
//
//            if (InverseKinematicsMultipleFramesService(msg.request, msg.response)) {
//                for (int i = 0; i < msg.response.joint_names.size(); i++) {
//                    q_target[joint_index[msg.response.joint_names[i]]] = (0.3*msg.response.angles[i]+0.7*q_target[joint_index[msg.response.joint_names[i]]]);
//                }
//            }
//        }
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
    boost::shared_ptr<tf::TransformListener> listener;
    float upper_arm_model_length = 0.262;
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


    RightArmTestbed robot(urdf, cardsflow_xml);

    if (nh.hasParam("simulated")) {
      nh.getParam("simulated", robot.simulated);
    }

    ros::Rate rate(30);
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
