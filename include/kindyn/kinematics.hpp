//
// Created by roboy on 01.07.21.
//

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

#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace Eigen;
using iDynTree::toEigen;
using iDynTree::fromEigen;

namespace cardsflow {
    namespace kindyn {
        class Kinematics {
        public:
            /**
             * Constructor
             */
            Kinematics();
            /**
             * Destructor
             */
            ~Kinematics();
            /**
             * initializes everything, call before use!
             * @param urdf_file_path path to robot urdf
             * @param viapoints_file_path path to viapoints xml
             * @param joint_names a vector of joint_names to be considered from the model
             */
            void init(string urdf_file_path, string viapoints_file_path, vector <string> joint_names);

            void setRobotState(VectorXd q_in, VectorXd qd_in);

            vector<Matrix4d> getRobotPosesFromJoints();
            Matrix4d getPoseFromJoint(int index);
            void getRobotCableFromJoints(VectorXd &l_out);

            /**
             * update Jacobians matrix
             */
            void updateJacobians();

            /**
             * Do forward kinematics
             */
            vector<VectorXd> oneStepForward(VectorXd& q_in, VectorXd& qd_in, vector<VectorXd> Ld);

            int GetJointIdByName(string joint);

            size_t number_of_dofs = 0; /// number of degrees of freedom of the whole robot
            vector<string> endeffectors; /// names of the endeffectors
            map<string,size_t> endeffector_index;
            vector<size_t> endeffector_number_of_dofs, endeffector_dof_offset; /// number of degrees of freedom of each endeffector
            size_t number_of_joints = 0; /// number of joints of the whole robot
            size_t number_of_cables = 0; /// number of cables, ie muscles of the whole robot
            size_t number_of_links = 0; /// number of links of the whole robot

            vector<string> link_relation_name;
            vector<vector<string>> joint_relation_name;
            vector<double> joint_dt;

            vector<Cable> cables; /// all cables of the robot
            vector <VectorXd> joint_axis; /// joint axis of each joint
            vector <string> link_names, joint_names; /// link and joint names of the robot
            map<string, int> link_index, joint_index; /// link and joint indices of the robot

            MatrixXd L, L_t; /// L and -L^T
            VectorXd q_min, q_max; /// joint limits

            MatrixXd M; /// The Mass matrix of the robot
            VectorXd CG; /// The Coriolis+Gravity term of the robot
            VectorXd torques; /// joint torques
            VectorXd cable_forces; /// the cable forces in Newton

        private:

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
             * parses the cardsflow xml file for viapoint definitions
             * @param viapoints_file_path path to the cardsflow.xml file
             * @param cables will be filled with the parsed viapoints of the defined cables
             * @return success
             */
            bool parseViapoints(const string &viapoints_file_path, vector<Cable> &cables);

            iDynTree::KinDynComputations kinDynComp, kinDynCompTarget; /// the full robot model
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
            } robotstate;

            typedef boost::array< double , 2 > state_type; /// second order dynamics integration
            vector<state_type> joint_state;

            Matrix4d world_H_base; /// floating base 6-DoF pose
            vector<Matrix4d> world_to_link_transform, link_to_world_transform, frame_transform;
            Matrix3d *link_to_link_transform;

            Eigen::Matrix<double,6,1> baseVel; /// the velocity of the floating base
            Vector3d gravity; /// gravity vector (default: (0,0,-9.81)

            /**
             * Critical variables
             */
            VectorXd q, qd, qdd; /// joint positon, velocity, acceleration
            VectorXd q_next, qd_next, qdd_next; /// previous joint positon, velocity, acceleration

            MatrixXd S, P, V, W; /// matrices of cable model
            vector <vector<pair < ViaPointPtr, ViaPointPtr>>> segments; /// cable segments

            /**
             * Auxiliary variables
             */
            Eigen::IOFormat fmt; /// formator for terminal printouts
            double integration_time =0;
        };
    }
}

