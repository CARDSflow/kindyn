/*
    BSD 3-Clause License
    Copyright (c) 2018, Roboy
            All rights reserved.
    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    author: Simon Trendel ( st@gi.ai ), 2018
    description: The Robot class implementing the Cable model of CARDSflow. Users should inherit from this class and
                 reimplement the read and write functions to match with their robots
*/

#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/InverseKinematics.h>

#include "tinyxml.h"

#include "kindyn/cable.hpp"
#include "kindyn/EigenExtension.hpp"
#include "kindyn/controller/cardsflow_state_interface.hpp"
#include "kindyn/controller/cardsflow_command_interface.hpp"

#include <roboy_communication_middleware/ForwardKinematics.h>
#include <roboy_communication_middleware/InverseKinematics.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorStatus.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <grid_map_ros/GridMapMsgHelpers.hpp>

#include <qpOASES.hpp>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/LoadController.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <roboy_communication_simulation/Tendon.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <roboy_communication_simulation/ControllerType.h>

#include <boost/numeric/odeint.hpp>

#include <common_utilities/rviz_visualization.hpp>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

using namespace qpOASES;
using namespace std;
using namespace Eigen;
using iDynTree::toEigen;
using iDynTree::fromEigen;

namespace cardsflow {
    namespace kindyn {
        class Robot:public hardware_interface::RobotHW, public rviz_visualization{
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
            void init(string urdf_file_path, string viapoints_file_path, vector<string> joint_names);
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

        private:
            /**
             * parses the cardsflow xml file for viapoint definitions
             * @param viapoints_file_path path to the cardsflow.xml file
             * @param cables will be filled with the parsed viapoints of the defined cables
             * @return success
             */
            bool parseViapoints(const string &viapoints_file_path, vector<Cable> &cables);
            /**
             * Forward kinematic service for endeffectors
             * @param req endeffector, requested joint angles
             * @param res 3d resulting position of endeffector
             * @return success
             */
            bool ForwardKinematicsService(roboy_communication_middleware::ForwardKinematics::Request &req,
                                          roboy_communication_middleware::ForwardKinematics::Response &res);
            /**
             * Inverse kinematic service for endeffectors
             * @param req endeffector and ik type
             * @param res joint configuration solution
             * @return success
             */
            bool InverseKinematicsService(roboy_communication_middleware::InverseKinematics::Request &req,
                                          roboy_communication_middleware::InverseKinematics::Response &res);
            /**
             * Callback for Interactive Marker Feedback of endeffectors. When the Interactive Marker is released in rviz,
             * the IK routine is called and the solution directly applied to the robot q_target angles
             * @param msg Interactive Marker Feedback message
             */
            void InteractiveMarkerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg);

            /**
             * Callback for the joint state of the robot. This can come from gazebo, the real robot or else where.
             * @param msg message containing joint_name/angle information
             */
            void JointState(const sensor_msgs::JointStateConstPtr &msg);

            /**
             * Callback for the floating base world pose. This can come from gazebo, the real robot or else where.
             * @param msg message containing the 6 DoF pose of the floating base
             */
            void FloatingBase(const geometry_msgs::PoseConstPtr &msg);

            vector<Matrix4d> world_to_link_transform, link_to_world_transform, frame_transform;
            Matrix3d *link_to_link_transform;

            VectorXd resolve_function(MatrixXd &A_eq, VectorXd &b_eq, VectorXd &f_min, VectorXd &f_max);

            /**
             * Updates the V matrix of the cable model
             */
            void update_V();

            /**
             * Updates the S matrix of the cable model
             */
            void update_S();

            /**
             * Updates the P matrix of the cable model
             */
            void update_P();

            /**
             * Callback for controller type change. The controller type defines how the forwardKinematics function
             * integrates the robot states
             * @param msg message containing the joint_name/type pair
             */
            void controllerType(const roboy_communication_simulation::ControllerTypeConstPtr &msg);

            ros::NodeHandlePtr nh; /// ROS node handle
            boost::shared_ptr <ros::AsyncSpinner> spinner; /// async ROS spinner
            ros::Publisher robot_state, tendon_state; /// ROS robot pose and tendon publisher
            ros::Subscriber controller_type_sub, joint_state_sub, floating_base_sub, interactive_marker_sub; /// ROS subscribers
            ros::ServiceServer ik_srv, fk_srv;

            iDynTree::KinDynComputations kinDynComp; /// the full robot model
            map<string,iDynTree::KinDynComputations> ik_models; /// the robot models for each endeffector
            map<string,iDynTree::InverseKinematics> ik; /// the ik for each endeffector
            map<string,string> ik_base_link; /// the base link of each endeffector
            struct iDynTreeRobotState
            {
                void resize(int nrOfInternalDOFs)
                {
                    jointPos.resize(nrOfInternalDOFs);
                    jointVel.resize(nrOfInternalDOFs);
                }

                iDynTree::Transform world_H_base;
                iDynTree::VectorDynSize jointPos;
                iDynTree::Twist         baseVel;
                iDynTree::VectorDynSize jointVel;
                iDynTree::Vector3       gravity;
            }robotstate;
        public:
            /**
             * Integrates the robot equation of motions using odeint
             * @param dt the integrations step length in seconds
             */
            void forwardKinematics(double dt);
            size_t number_of_dofs = 0; /// number of degrees of freedom of the whole robot
            vector<string> endeffectors; /// names of the endeffectors
            map<string,size_t> endeffector_index;
            vector<size_t> endeffector_number_of_dofs, endeffector_dof_offset; /// number of degrees of freedom of each endeffector
            size_t number_of_joints = 0; /// number of joints of the whole robot
            size_t number_of_cables = 0; /// number of cables, ie muscles of the whole robot
            size_t number_of_links = 0; /// number of links of the whole robot
            Matrix4d world_H_base; /// floating base 6-DoF pose
            Eigen::Matrix<double,6,1> baseVel; /// the velocity of the floating base
            Vector3d gravity; /// gravity vector (default: (0,0,-9.81)
            MatrixXd M; /// The Mass matrix of the robot
            VectorXd CG; /// The Coriolis+Gravity term of the robot
            VectorXd q, qd, qdd; /// joint positon, velocity, acceleration
            VectorXd q_target, qd_target, qdd_target; /// joint positon, velocity, acceleration targets
            VectorXd l, Ld; /// tendon length and length change
            VectorXd torques; /// joint torques
            VectorXd cable_forces; /// the cable forces in Newton
            vector<VectorXd> ld; /// tendon length changes for each controller
            MatrixXd L, L_t; /// L and -L^T

        private:
            iDynTree::FreeFloatingGeneralizedTorques bias; /// Coriolis+Gravity term
            iDynTree::MatrixDynSize Mass; /// Mass matrix

            bool torque_position_controller_active = false, force_position_controller_active = false, cable_length_controller_active = false;
            MatrixXd S, P, V, W; /// matrices of cable model
            vector<Cable> cables; /// all cables of the robot
            vector <VectorXd> joint_axis; /// joint axis of each joint
            vector <string> link_names, joint_names; /// link and joint names of the robot
            map<string, int> link_index, joint_index; /// link and joint indices of the robot
            vector<int> controller_type; /// currently active controller type
            double integration_time =0; /// odeint integration time
            typedef boost::array< double , 2 > state_type; /// second order dynamics integration
            vector<state_type> joint_state, motor_state; /// joint and cable states
            bool first_time_solving = true;
            int nWSR = 1000; /// qp working sets
            VectorXd f_min, f_max;
            int qp_print_level = PL_NONE; /// qpoases print level
            SQProblem qp_solver; /// qpoases quadratic problem solver
            real_t *H, *g, *A, *lb, *ub, *b, *FOpt; /// quadratic problem variables
            ros::Time last_visualization; /// timestamp for visualization at reasonable intervals
            vector <vector<pair < ViaPointPtr, ViaPointPtr>>> segments; /// cable segments
            Eigen::IOFormat fmt; /// formator for terminal printouts
            hardware_interface::JointStateInterface joint_state_interface; /// ros control joint state interface
            hardware_interface::EffortJointInterface joint_command_interface; /// ros control joint command interface
            hardware_interface::CardsflowStateInterface cardsflow_state_interface; /// cardsflow state interface
            hardware_interface::CardsflowCommandInterface cardsflow_command_interface; /// cardsflow command interface
        };
    }
}

typedef boost::shared_ptr<cardsflow::kindyn::Robot> RobotPtr; /// typedef for convenience