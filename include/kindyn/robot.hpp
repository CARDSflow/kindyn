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

//#include <qpOASES.hpp>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/LoadController.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <roboy_communication_simulation/Tendon.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <roboy_communication_simulation/ControllerState.h>

#include <boost/numeric/odeint.hpp>

#include <common_utilities/rviz_visualization.hpp>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

//using namespace qpOASES;
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
            ~Robot();

            /**
             * initializes everything, call before use!
             * @param urdf_file_path path to robot urdf
             * @param viapoints_file_path path to viapoints xml
             * @param joint_names a vector of joint_names to be considered from the model
             */
            void init(string urdf_file_path, string viapoints_file_path, vector<string> joint_names);

            void update();

            virtual void read(){
                ROS_WARN_STREAM_THROTTLE(1, "reading virtual, "
                        "you probably forgot to implement your own read function?!");
            };

            virtual void write(){
                ROS_WARN_STREAM_THROTTLE(1, "writing virtual, "
                        "you probably forgot to implement your own write function?!");
            };

        private:
            bool parseViapoints(const string &viapoints_file_path, vector<Cable> &cables);

            bool ForwardKinematicsService(roboy_communication_middleware::ForwardKinematics::Request &req,
                                          roboy_communication_middleware::ForwardKinematics::Response &res);

            bool InverseKinematicsService(roboy_communication_middleware::InverseKinematics::Request &req,
                                          roboy_communication_middleware::InverseKinematics::Response &res);

            void InteractiveMarkerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg);

            void JointState(const sensor_msgs::JointStateConstPtr &msg);

            void FloatingBase(const geometry_msgs::PoseConstPtr &msg);

            vector<Matrix4d> world_to_link_transform, link_to_world_transform, frame_transform;
            Matrix3d *link_to_link_transform;

//            VectorXd resolve_function(MatrixXd &A_eq, VectorXd &b_eq, VectorXd &f_min, VectorXd &f_max);

            void update_V();

            void update_S();

            void update_P();

            void controllerType(const roboy_communication_simulation::ControllerStateConstPtr &msg);

            ros::NodeHandlePtr nh;
            boost::shared_ptr <ros::AsyncSpinner> spinner;
            ros::Publisher robot_state, tendon_state;
            ros::Subscriber controller_type_sub, joint_state_sub, floating_base_sub, interactive_marker_sub;
            ros::ServiceServer ik_srv, fk_srv;

            // robot model
            iDynTree::KinDynComputations kinDynComp;
            map<string,iDynTree::KinDynComputations> ik_models;
            map<string,iDynTree::InverseKinematics> ik;
            map<string,string> ik_base_link;
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
            void forwardKinematics(double dt);
            size_t number_of_dofs = 0; /// number of degrees of freedom of the whole robot
            vector<string> endeffectors; /// names of the endeffectors
            map<string,size_t> endeffector_index;
            vector<size_t> endeffector_number_of_dofs, endeffector_dof_offset; /// number of degrees of freedom of each endeffector
            size_t number_of_joints = 0; /// number of joints of the whole robot
            size_t number_of_cables = 0; /// number of cables, ie muscles of the whole robot
            size_t number_of_links = 0; /// number of links of the whole robot
            Matrix4d world_H_base;
            Eigen::Matrix<double,6,1> baseVel;
            Vector3d gravity;
            MatrixXd M;
            VectorXd CG;
            VectorXd q, qd, qdd;
            VectorXd q_target, qd_target, qdd_target;
            VectorXd l, Ld;
            VectorXd torques;
            VectorXd cable_forces;
            vector<VectorXd> ld;
            MatrixXd L, L_t;

        private:
            iDynTree::FreeFloatingGeneralizedTorques bias;
            iDynTree::MatrixDynSize Mass;

            MatrixXd S, P, V, W;
            vector<Cable> cables;
            vector <VectorXd> joint_axis;
            vector <string> link_names, joint_names;
            map<string, int> link_index, joint_index;
            vector<int> controller_type;
            double integration_time =0;
            typedef boost::array< double , 2 > state_type; // second order
            vector<state_type> joint_state, motor_state;
            bool first_time_solving = true;
//            SQProblem qp_solver; /// qpoases quadratic problem solver
//            real_t *H, *g, *A, *lb, *ub, *b, *FOpt; /// quadratic problem variables
            ros::Time last_visualization;
        private:
            vector <vector<pair < ViaPointPtr, ViaPointPtr>>> segments;
        private:
            Eigen::IOFormat fmt;
        private:
            hardware_interface::JointStateInterface joint_state_interface;
            hardware_interface::EffortJointInterface joint_command_interface;
            hardware_interface::CardsflowStateInterface cardsflow_state_interface;
            hardware_interface::CardsflowCommandInterface cardsflow_command_interface;
        };
    }
}

typedef boost::shared_ptr<cardsflow::kindyn::Robot> RobotPtr;