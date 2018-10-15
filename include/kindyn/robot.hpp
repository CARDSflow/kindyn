#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>

#include "tinyxml.h"

#include "kindyn/cable.hpp"
#include "kindyn/EigenExtension.hpp"
#include "kindyn/controller/cardsflow_state_interface.hpp"
#include "kindyn/controller/cardsflow_command_interface.hpp"

#include <roboy_communication_middleware/ForwardKinematics.h>
#include <roboy_communication_middleware/InverseKinematics.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <sensor_msgs/JointState.h>

#include <common_utilities/measureExecutionTime.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <grid_map_ros/GridMapMsgHelpers.hpp>
#include <common_utilities/rviz_visualization.hpp>

#include <qpOASES.hpp>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/LoadController.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

using namespace qpOASES;
using namespace std;
using namespace Eigen;
using iDynTree::toEigen;
using iDynTree::fromEigen;

namespace cardsflow {
    namespace kindyn {

        class Robot:public rviz_visualization, public hardware_interface::RobotHW{
        public:
            /**
             * Constructor
             * @param urdf_file_path path to robot urdf
             * @param viapoints_file_path path to viapoints xml
             */
            Robot(string urdf_file_path, string viapoints_file_path);
            ~Robot();

            bool parseViapoints(const string &viapoints_file_path, vector<Cable> &cables);

            void update(double period);

            void forwardKinematics(double dt);

            void updateController();

            bool ForwardKinematicsService(roboy_communication_middleware::ForwardKinematics::Request &req,
                                          roboy_communication_middleware::ForwardKinematics::Response &res);

            bool InverseKinematicsService(roboy_communication_middleware::InverseKinematics::Request &req,
                                          roboy_communication_middleware::InverseKinematics::Response &res);

            vector<Matrix4d> world_to_link_transform;

            void init();

        protected:
            VectorXd resolve_function(MatrixXd &A_eq, VectorXd &b_eq, VectorXd &f_min, VectorXd &f_max);

            void update_V();

            void update_S();

            void update_P();

            bool getTransform(const char *to, const char *from, Matrix4d &transform);

            int message_counter = 6666;

            void publishTendons();

            void publishForces();

            ros::NodeHandlePtr nh;
            boost::shared_ptr <ros::AsyncSpinner> spinner;

            // robot model
            iDynTree::KinDynComputations kinDynComp;
            size_t number_of_dofs = 0; /// number of degrees of freedom for kinematic chain
            size_t number_of_joints = 0; /// number of joints
            size_t number_of_cables = 0; /// number of cables, ie muscles, for kinematic chain
            size_t number_of_links = 0; /// number of links for kinematic chain
            Matrix4d world_H_base;
            Eigen::Matrix<double,6,1> baseVel;
            VectorXd q, qd, qdd;
            VectorXd q_target, qd_target, qdd_target;
            Vector3d gravity;
            MatrixXd M;
            iDynTree::FreeFloatingGeneralizedTorques bias;
            iDynTree::MatrixDynSize Mass;
            VectorXd CG;
            MatrixXd S, P, V, W, L, L_t;
            VectorXd l;
            vector<VectorXd> ld;
            VectorXd x, x_dot, x_dot_relative;
            VectorXd e, de, dde;

            vector<Cable> cables;
            vector <VectorXd> joint_axis;
            vector <string> link_names, joint_names;
            map<string, int> link_index, joint_index;

            bool first_time_solving = true;
            SQProblem qp_solver; /// qpoases quadratic problem solver
            real_t *H, *g, *A, *lb, *ub, *b, *FOpt; /// quadratic problem variables

            double Kp = 100, Kd = 10;
        private:
            vector <vector<pair < ViaPointPtr, ViaPointPtr>>> segments;
            int joint_angle_mask;
            int controller = 1;
        private:
            ofstream log_file;
            Eigen::IOFormat fmt;
            static int instance;
            bool verbose, log;
            tf::TransformListener tf_listener;
            tf::TransformBroadcaster tf_broadcaster;
            ros::Publisher robot_state_pub, joint_state_pub;
        private:
            hardware_interface::CardsflowStateInterface cardsflow_state_interface;
            hardware_interface::CardsflowCommandInterface cardsflow_command_interface;
            VectorXd cmd;
        };
    }
}

typedef boost::shared_ptr<cardsflow::kindyn::Robot> RobotPtr;