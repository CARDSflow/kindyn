//
// Created by roboy on 01.07.21.
//

#ifndef ROBOY3_ROBOT_NEW_H
#define ROBOY3_ROBOT_NEW_H

#pragma once

#include "kindyn/kinematics.hpp"

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "kindyn/cable.hpp"
#include "kindyn/EigenExtension.hpp"
#include "kindyn/controller/cardsflow_state_interface.hpp"
#include "kindyn/controller/cardsflow_command_interface.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <roboy_simulation_msgs/Tendon.h>
#include <roboy_simulation_msgs/ControllerType.h>
#include <roboy_simulation_msgs/JointState.h>
#include <roboy_middleware_msgs/ForwardKinematics.h>
#include <roboy_middleware_msgs/InverseKinematics.h>
#include <roboy_middleware_msgs/InverseKinematicsMultipleFrames.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_control_msgs/MoveEndEffectorAction.h>
#include <roboy_control_msgs/Strings.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/LoadController.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <boost/numeric/odeint.hpp>

#include <common_utilities/rviz_visualization.hpp>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <thread>

using namespace std;
using namespace Eigen;

namespace cardsflow {
    namespace kindyn {
        class Robot : public hardware_interface::RobotHW, public rviz_visualization {
        public:
            /**
             * Constructor
             */
            Robot();

            /**
             * Destructor
             */
            ~Robot();

            /**
             * initializes everything, call before use!
             * @param urdf_file_path path to robot urdf
             * @param viapoints_file_path path to viapoints xml
             * @param joint_names a vector of joint_names to be considered from the model
             */
            void init(string urdf_file_path, string viapoints_file_path, vector <string> joint_names);

            /**
             * Updates the model
             */
            void update();

            /**
             * This is the read function and should implement reading the state of your robot
             */
            virtual void read(){
                ROS_WARN_STREAM_THROTTLE(1, "reading virtual, "
                                            "you probably forgot to implement your own read function?!");
            };
            /**
             * This is the write function and should implement writing commands to your robot
             */
            virtual void write(){
                ROS_WARN_STREAM_THROTTLE(1, "writing virtual, "
                                            "you probably forgot to implement your own write function?!");
            };

            bool simulated = false;
        protected:

            void updatePublishers();
            void updateSubscribers();
            void publishViz();

            /**
             * Callback for the joint state of the robot. This can come from gazebo, the real robot or else where.
             * @param msg message containing joint_name/angle information
             */
            void JointState(const sensor_msgs::JointStateConstPtr &msg);

            void JointTarget(const sensor_msgs::JointStateConstPtr &msg);

            /**
             * Callback for controller type change. The controller type defines how the forwardKinematics function
             * integrates the robot states
             * @param msg message containing the joint_name/type pair
             */
            void controllerType(const roboy_simulation_msgs::ControllerTypeConstPtr &msg);
            void ZeroJoints(const roboy_control_msgs::StringsPtr &msg);
            bool FreezeService(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res);

            cardsflow::kindyn::Kinematics kinematics;
            ros::NodeHandlePtr nh; /// ROS node handle
            boost::shared_ptr <ros::AsyncSpinner> spinner; /// async ROS spinner

            bool torque_position_controller_active = false, force_position_controller_active = false, cable_length_controller_active = false;
            hardware_interface::CardsflowStateInterface cardsflow_state_interface; /// cardsflow state interface
            hardware_interface::CardsflowCommandInterface cardsflow_command_interface; /// cardsflow command interface
            vector<int> controller_type; /// currently active controller type

            ros::Publisher robot_state_pub, tendon_state_pub, tendon_ext_state_pub, joint_state_pub, cardsflow_joint_states_pub; /// ROS robot pose and tendon publisher
            ros::Publisher robot_state_target_pub, tendon_state_target_pub, joint_state_target_pub; /// target publisher
            ros::Subscriber controller_type_sub, joint_state_sub, floating_base_sub, interactive_marker_sub, joint_target_sub, zero_joints_sub; /// ROS subscribers
            ros::ServiceServer ik_srv, ik_two_frames_srv, fk_srv, freeze_srv;
            string topic_root;

            VectorXd q, qd, qdd; /// joint positon, velocity, acceleration
            VectorXd q_target, qd_target, qdd_target; /// joint positon, velocity, acceleration targets
            VectorXd q_target_prev, qd_target_prev, qdd_target_prev; /// joint positon, velocity, acceleration targets
            VectorXd l_next, l_target;
            vector<VectorXd> Ld; // tendon velocity per endeffector
            vector<VectorXd> ld; /// tendon length changes for each controller

            bool first_update = true;
            ros::Time last_visualization; /// timestamp for visualization at reasonable intervals
            Eigen::IOFormat fmt; /// formator for terminal printouts
            bool external_robot_state; /// indicates if we get the robot state externally
            bool external_robot_target = false;
            bool debug_ = false;
            double k_dt = 0.005;
            VectorXd Kp_, Kd_;
            vector<double> param_kp, param_kd;
        };
    };
}


#endif //ROBOY3_ROBOT_NEW_H
