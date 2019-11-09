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
#include <tf/transform_broadcaster.h>
#include <darkroom/TrackedObject.hpp>
#include <darkroom/LighthouseSimulator.hpp>
#include "boost/filesystem.hpp"

using namespace std;
using namespace boost::filesystem;

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

        if (nh->hasParam("simulated")) {
            nh->getParam("simulated", simulated);
        }

        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        nh->getParam("external_robot_state", external_robot_state);
        ROS_INFO_STREAM("External robot state: " << external_robot_state);
        init(urdf,cardsflow_xml,joint_names);
        listener.reset(new tf::TransformListener);
        update();
        bool darkroom_tracking_enabled = false;
        nh->getParam("darkroom_tracking_enabled",darkroom_tracking_enabled);
        if(darkroom_tracking_enabled){
            ROS_INFO("darkroom tracking enabled");
            string model_name;
            nh->getParam("model_name",model_name);
            path p(ros::package::getPath("robots")+"/"+model_name+"/lighthouseSensors/");

            directory_iterator end_itr;
            vector<boost::filesystem::path> config_files;
            // cycle through the directory
            for (directory_iterator itr(p); itr != end_itr; ++itr)
            {
                // If it's not a directory, list it. If you want to list directories too, just remove this check.
                if (is_regular_file(itr->path())) {
                    // assign current file name to current_file and echo it out to the console.
                    string current_file = itr->path().string();
                    ROS_INFO("adding trackedObect %s",current_file.c_str());
                    TrackedObjectPtr ptr(new TrackedObject(nh));
                    trackedObjects.push_back(ptr);
                    ptr->init(current_file.c_str());
                    ptr->poseestimating_multiLighthouse = true;
                    ptr->object_pose_estimation_multi_lighthouse_thread = boost::shared_ptr<boost::thread>(
                            new boost::thread([this, ptr]() {
                                ptr->estimateObjectPoseMultiLighthouse();
                            }));
                    ptr->object_pose_estimation_multi_lighthouse_thread->detach();
                    ptr->tracking = true;
                    ptr->tracking_thread = boost::shared_ptr<boost::thread>(
                            new boost::thread(
                                    [this, ptr]() { ptr->triangulateSensors(); }
                            ));
                    ptr->tracking_thread->detach();
                    config_files.push_back(itr->path());
                }
            }
            if(simulated){
                for(int i=0;i<2;i++){
                    LighthouseSimulatorPtr ptr(new LighthouseSimulator(i,config_files));
                    lighthouseSimulator.push_back(ptr);
                    ptr->startSensorPublisher();
                }
                pose_correction_sub = nh->subscribe("/roboy/middleware/DarkRoom/LighthousePoseCorrection", 1,
                                                    &Exohaptic::correctPose, this);
                tf::Quaternion quat;
                quat.setRPY(0, 0, 0);
                lighthouse1.setRotation(quat);
                lighthouse2.setRotation(quat);
                lighthouse1.setOrigin(tf::Vector3(0, -1, 0));
                lighthouse2.setOrigin(tf::Vector3(-0.825, -1, 0));
                publish_transform = true;
                transform_thread = boost::shared_ptr<std::thread>(new std::thread(&Exohaptic::transformPublisher, this));
                transform_thread->detach();

                interactive_marker_sub = nh->subscribe("/interactive_markers/feedback", 1,
                                                       &Exohaptic::interactiveMarkersFeedback, this);

                make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, lighthouse1.getOrigin(),
                               true, 0.1, "world", "lighthouse1", "");
                make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, lighthouse2.getOrigin(),
                               true, 0.1, "world", "lighthouse2", "");
            }
        }

        ROS_INFO_STREAM("Finished setup");
    };

    void correctPose(const roboy_middleware_msgs::LighthousePoseCorrection &msg) {
        mux.lock();
        tf::Transform tf;
        tf::transformMsgToTF(msg.tf, tf);
        if (msg.id == LIGHTHOUSE_A) {
            if (msg.type == 0) // relativ
                lighthouse1 = tf * lighthouse1;
            else    // absolut
                lighthouse1 = tf;
        } else {
            if (msg.type == 0) // relativ
                lighthouse2 = tf * lighthouse2;
            else    // absolut
                lighthouse2 = tf;
        }
        mux.unlock();
    }

    void transformPublisher() {
        ros::Rate rate(60);
        while (publish_transform && ros::ok()) {
            tf_broadcaster.sendTransform(
                    tf::StampedTransform(lighthouse1, ros::Time::now(), "world", "lighthouse1"));
            tf_broadcaster.sendTransform(
                    tf::StampedTransform(lighthouse2, ros::Time::now(), "world", "lighthouse2"));
            rate.sleep();
        }
    }

    /**
     * Updates the robot model
     */
    void read() override{
        update();
    };
    /**
     * Sends motor commands to the real robot
     */
    void write() override{
        static bool dir = false;
        if(simulated){
            if(dir){
                q_target[joint_index["left_joint0"]]+=0.01;
                q_target[joint_index["right_joint0"]]-=0.01;
            }else{
                q_target[joint_index["left_joint0"]]-=0.01;
                q_target[joint_index["right_joint0"]]+=0.01;
            }
            if(q_target[joint_index["left_joint0"]]<=-1){
                dir = true;
            }
            if(q_target[joint_index["left_joint0"]]>=1){
                dir = false;
            }
        }else {
            tf::StampedTransform right_hand, left_hand, right_elbow, left_elbow, right_upper_arm, left_upper_arm;
            bool left_hand_available = true, right_hand_available = true;
            try {
                listener->lookupTransform("0_1", "0_4",
                                          ros::Time(0), right_hand);
                listener->lookupTransform("0_1", "0_3",
                                          ros::Time(0), right_elbow);
                listener->lookupTransform("0_2", "0_3",
                                          ros::Time(0), right_upper_arm);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR_ONCE("%s", ex.what());
                left_hand_available = false;
            }
            try {
                listener->lookupTransform("0_1", "0_7",
                                          ros::Time(0), left_hand);
                listener->lookupTransform("0_1", "0_6",
                                          ros::Time(0), left_elbow);
                listener->lookupTransform("0_5", "0_6",
                                          ros::Time(0), left_upper_arm);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR_ONCE("%s", ex.what());
                right_hand_available = false;
            }

            if (left_hand_available) {
                {
                    roboy_middleware_msgs::InverseKinematicsMultipleFrames msg;
                    msg.request.endeffector = "hand_left";
                    msg.request.target_frames = {"hand_left", "elbow_left_link1"};
                    msg.request.type = 1;
                    geometry_msgs::Pose pose;
                    pose.position.x = left_hand.getOrigin().x();
                    pose.position.y = left_hand.getOrigin().y();
                    pose.position.z = left_hand.getOrigin().z();
                    pose.orientation.w = 1;
                    msg.request.poses.push_back(pose);
                    pose.position.x = left_elbow.getOrigin().x();
                    pose.position.y = left_elbow.getOrigin().y();
                    pose.position.z = left_elbow.getOrigin().z();
                    pose.orientation.w = 1;
                    msg.request.poses.push_back(pose);
                    msg.request.weights = {0.9, 0.9};

                    if (InverseKinematicsMultipleFramesService(msg.request, msg.response)) {
                        for (int i = 0; i < msg.response.joint_names.size(); i++) {
                            q_target[joint_index[msg.response.joint_names[i]]] = (0.3 * msg.response.angles[i] + 0.7 *
                                                                                                                 q_target[joint_index[msg.response.joint_names[i]]]);
                        }
                    }
                }
                tf::StampedTransform upper_arm_left, lower_arm_left;
                try {
                    listener->lookupTransform("world", "upper_arm_left",
                                              ros::Time(0), upper_arm_left);
                    listener->lookupTransform("world", "lower_arm_left",
                                              ros::Time(0), lower_arm_left);
                }
                catch (tf::TransformException ex) {
                    ROS_ERROR_ONCE("%s", ex.what());
                }
                {
                    roboy_middleware_msgs::InverseKinematicsMultipleFrames msg;
                    msg.request.endeffector = "left_link6";
                    msg.request.target_frames = {"left_link3", "left_link6"};
                    msg.request.type = 1;
                    geometry_msgs::Pose pose;
                    pose.position.x = upper_arm_left.getOrigin().x();
                    pose.position.y = upper_arm_left.getOrigin().y();
                    pose.position.z = upper_arm_left.getOrigin().z();
                    pose.orientation.w = 1;
                    msg.request.poses.push_back(pose);
                    pose.position.x = lower_arm_left.getOrigin().x();
                    pose.position.y = lower_arm_left.getOrigin().y();
                    pose.position.z = lower_arm_left.getOrigin().z();
                    pose.orientation.w = 1;
                    msg.request.poses.push_back(pose);
                    msg.request.weights = {0.9, 0.9};

                    if (InverseKinematicsMultipleFramesService(msg.request, msg.response)) {
                        for (int i = 0; i < msg.response.joint_names.size(); i++) {
                            q_target[joint_index[msg.response.joint_names[i]]] = (0.3 * msg.response.angles[i] + 0.7 *
                                                                                                                 q_target[joint_index[msg.response.joint_names[i]]]);
                        }
                    }
                }
            }
            if (right_hand_available) {
                roboy_middleware_msgs::InverseKinematicsMultipleFrames msg;
                msg.request.endeffector = "hand_right";
                msg.request.target_frames = {"hand_right", "elbow_right_link1"};
                msg.request.type = 1;
                geometry_msgs::Pose pose;
                pose.position.x = right_hand.getOrigin().x();
                pose.position.y = right_hand.getOrigin().y();
                pose.position.z = right_hand.getOrigin().z();
                pose.orientation.w = 1;
                msg.request.poses.push_back(pose);
                pose.position.x = right_elbow.getOrigin().x();
                pose.position.y = right_elbow.getOrigin().y();
                pose.position.z = right_elbow.getOrigin().z();
                pose.orientation.w = 1;
                msg.request.poses.push_back(pose);
                msg.request.weights = {0.9, 0.9};

                if (InverseKinematicsMultipleFramesService(msg.request, msg.response)) {
                    for (int i = 0; i < msg.response.joint_names.size(); i++) {
                        q_target[joint_index[msg.response.joint_names[i]]] = (0.3 * msg.response.angles[i] + 0.7 *
                                                                                                             q_target[joint_index[msg.response.joint_names[i]]]);
                    }
                }
            }
        }
    };

    void interactiveMarkersFeedback(const visualization_msgs::InteractiveMarkerFeedback &msg) {
        mux.lock();
        tf::Vector3 position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        tf::Quaternion orientation(msg.pose.orientation.x, msg.pose.orientation.y,
                                   msg.pose.orientation.z, msg.pose.orientation.w);
        if (strcmp(msg.marker_name.c_str(), "lighthouse1") == 0) {
            lighthouse1.setOrigin(position);
            lighthouse1.setRotation(orientation);
        } else if (strcmp(msg.marker_name.c_str(), "lighthouse2") == 0) {
            lighthouse2.setOrigin(position);
            lighthouse2.setRotation(orientation);
        } else if (strcmp(msg.marker_name.c_str(), "trackedObject") == 0) {
            for (auto &object:trackedObjects) {
                object->pose.setOrigin(position);
                object->pose.setRotation(orientation);
            }
        }
        mux.unlock();
    }

    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::Subscriber pose_correction_sub, interactive_marker_sub;
    ros::AsyncSpinner *spinner;
    boost::shared_ptr<tf::TransformListener> listener;
    vector<TrackedObjectPtr> trackedObjects;
    vector<LighthouseSimulatorPtr> lighthouseSimulator;
    tf::Transform lighthouse1, lighthouse2;
    bool publish_transform = true;
    boost::shared_ptr<std::thread> transform_thread;
    tf::TransformBroadcaster tf_broadcaster;
    mutex mux;
};

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "exohaptic");
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

    ros::Rate rate(30);
    while(ros::ok()){
        robot.read();
        robot.write();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("TERMINATING...");

    return 0;
}
