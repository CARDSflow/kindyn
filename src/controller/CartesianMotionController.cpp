//
// Created by roboy on 03.04.20.
//
////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#include "kindyn/controller/CartesianMotionController.h"
#include <map>
#include <sstream>
#include <boost/algorithm/clamp.hpp>
#include <eigen_conversions/eigen_kdl.h>

// KDL
#include <kdl/jntarrayvel.hpp>
#include <kdl/framevel.hpp>


#include <urdf/model.h>

CartesianMotionController::CartesianMotionController()
{
}

CartesianMotionController::~CartesianMotionController(){}

bool CartesianMotionController::init(ros::NodeHandle& nh)
{
//    if (m_already_initialized)
//    {
//        return true;
//    }


    std::string robot_description;
    urdf::Model robot_model;
    KDL::Tree   robot_tree;
    KDL::Chain  robot_chain;
//    auto m_robot_base_link = "torso";
//    auto m_end_effector_link = "hand_right";
    // Get controller specific configuration
    if (!nh.getParam("/robot_description",robot_description))
    {
        ROS_ERROR("Failed to load '/robot_description' from parameter server");
        return false;
    }
//    if (!nh.getParam("robot_base_link",m_robot_base_link))
//    {
//        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/robot_base_link" << " from parameter server");
//        return false;
//    }
//    if (!nh.getParam("end_effector_link",m_end_effector_link))
//    {
//        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/end_effector_link" << " from parameter server");
//        return false;
//    }

    // Build a kinematic chain of the robot
    if (!robot_model.initString(robot_description))
    {
        ROS_ERROR("Failed to parse urdf model from 'robot_description'");
        return false;
    }
    if (!kdl_parser::treeFromUrdfModel(robot_model,robot_tree))
    {
        const std::string error = ""
                                  "Failed to parse KDL tree from urdf model";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }
    if (!robot_tree.getChain(m_robot_base_link,m_end_effector_link,robot_chain))
    {
        const std::string error = ""
                                  "Failed to parse robot chain from urdf model. "
                                  "Are you sure that both your 'robot_base_link' and 'end_effector_link' exist?";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }



//    // Get names of controllable joints from the parameter server
//    if (!nh.getParam("joints",m_joint_names))
//    {
//        const std::string error = ""
//                                  "Failed to load " + nh.getNamespace() + "/joints" + " from parameter server";
//        ROS_ERROR_STREAM(error);
//        throw std::runtime_error(error);
//    }
    q_target.resize(m_joint_names.size());
    q_target.setZero();

    // Parse joint limits
    KDL::JntArray upper_pos_limits(m_joint_names.size());
    KDL::JntArray lower_pos_limits(m_joint_names.size());
    for (size_t i = 0; i < m_joint_names.size(); ++i)
    {
        if (!robot_model.getJoint(m_joint_names[i]))
        {
            const std::string error = ""
                                      "Joint " + m_joint_names[i] + " does not appear in /robot_description";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }
        upper_pos_limits(i) = robot_model.getJoint(m_joint_names[i])->limits->upper;
        lower_pos_limits(i) = robot_model.getJoint(m_joint_names[i])->limits->lower;
    }

    // Get the joint handles to use in the control loop
    for (size_t i = 0; i < m_joint_names.size(); ++i)
    {
        //m_joint_handles.push_back(hw->getHandle(m_joint_names[i]));
    }

    // Initialize solvers
    initSolver(robot_chain,upper_pos_limits,lower_pos_limits);
    KDL::Tree tmp("not_relevant");
    tmp.addChain(robot_chain,"not_relevant");
    m_forward_kinematics_solver.reset(new KDL::TreeFkSolverPos_recursive(tmp));

    // Initialize Cartesian pd controllers
    m_spatial_controller.init(nh);

    // Connect dynamic reconfigure and overwrite the default values with values
    // on the parameter server. This is done automatically if parameters with
    // the according names exist.
    if (!nh.getParam("error_scale",m_error_scale)) {
        m_error_scale = 5.0;
    }
    if (!nh.getParam("iterations", m_iterations)) {
        m_iterations = 1;
    }
    if (!nh.getParam("period", m_period)) {
        m_period = 0.1;
    }

    m_current_frame = getEndEffectorPose();

    // Start where we are
    m_target_frame = m_current_frame;

    m_already_initialized = true;

    // Start with normal ROS control behavior
    m_paused = false;

    return true;
}

const KDL::Frame& CartesianMotionController::getEndEffectorPose() const
{
    return m_end_effector_pose;
}

void CartesianMotionController::updateKinematics()
{
    // Keep feed forward simulation running
    m_last_positions = m_current_positions;

    // Pose w. r. t. base
    m_fk_pos_solver->JntToCart(m_current_positions,m_end_effector_pose);

    // Absolute velocity w. r. t. base
    KDL::FrameVel vel;
    m_fk_vel_solver->JntToCart(KDL::JntArrayVel(m_current_positions,m_current_velocities),vel);
    m_end_effector_vel[0] = vel.deriv().vel.x();
    m_end_effector_vel[1] = vel.deriv().vel.y();
    m_end_effector_vel[2] = vel.deriv().vel.z();
    m_end_effector_vel[3] = vel.deriv().rot.x();
    m_end_effector_vel[4] = vel.deriv().rot.y();
    m_end_effector_vel[5] = vel.deriv().rot.z();
}

trajectory_msgs::JointTrajectoryPoint CartesianMotionController::getJointControlCmds(
        ros::Duration period,
        const ctrl::Vector6D& net_force)
{

    // Compute joint space inertia matrix
    m_jnt_space_inertia_solver->JntToMass(m_current_positions,m_jnt_space_inertia);

    // Compute joint jacobian
    m_jnt_jacobian_solver->JntToJac(m_current_positions,m_jnt_jacobian);

    // Compute joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f) \f$
    m_current_accelerations.data = m_jnt_space_inertia.data.inverse() * m_jnt_jacobian.data.transpose() * net_force;

    // Integrate once, starting with zero motion
    m_current_velocities.data = 0.5 * m_current_accelerations.data * period.toSec();

    // Integrate twice, starting with zero motion
    m_current_positions.data = m_last_positions.data + 0.5 * m_current_velocities.data * period.toSec();

    // Make sure positions stay in allowed margins
    for (int i = 0; i < m_number_joints; ++i)
    {
        m_current_positions(i) = boost::algorithm::clamp(
                m_current_positions(i),m_lower_pos_limits(i),m_upper_pos_limits(i));
    }

    // Apply results
    trajectory_msgs::JointTrajectoryPoint control_cmd;
    for (int i = 0; i < m_number_joints; ++i)
    {
        control_cmd.positions.push_back(m_current_positions(i));
        control_cmd.velocities.push_back(m_current_velocities(i));

        // Accelerations should be left empty. Those values will be interpreted
        // by most hardware joint drivers as max. tolerated values. As a
        // consequence, the robot will move very slowly.
    }
    control_cmd.time_from_start = period; // valid for this duration

    return control_cmd;
}

const KDL::JntArray& CartesianMotionController::getPositions() const
{
    return m_current_positions;
}


bool CartesianMotionController::setStartState(VectorXd jointPos)
{
    // Copy into internal buffers.
    for (int i = 0; i < m_joint_names.size(); ++i)
    {
        m_current_positions(i)      = jointPos[i];
//        m_current_velocities(i)     = joint_handles[i].getVelocity();
        m_current_accelerations(i)  = 0.0;
        m_last_positions(i)         = m_current_positions(i);
    }
    return true;
}


bool CartesianMotionController::initSolver(
        const KDL::Chain& chain,
        const KDL::JntArray& upper_pos_limits,
        const KDL::JntArray& lower_pos_limits)
{
    if (!buildGenericModel(chain))
    {
        ROS_ERROR("ForwardDynamicsSolver: Something went wrong in setting up the internal model.");
        return false;
    }

    // Initialize
    m_number_joints              = m_chain.getNrOfJoints();
    m_current_positions.data     = ctrl::VectorND::Zero(m_number_joints);
    m_current_velocities.data    = ctrl::VectorND::Zero(m_number_joints);
    m_current_accelerations.data = ctrl::VectorND::Zero(m_number_joints);
    m_last_positions.data        = ctrl::VectorND::Zero(m_number_joints);
    m_upper_pos_limits           = upper_pos_limits;
    m_lower_pos_limits           = lower_pos_limits;

    // Forward kinematics
    m_fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
    m_fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(m_chain));

    // Forward dynamics
    m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
    m_jnt_space_inertia_solver.reset(new KDL::ChainDynParam(m_chain,KDL::Vector::Zero()));
    m_jnt_jacobian.resize(m_number_joints);
    m_jnt_space_inertia.resize(m_number_joints);

    ROS_INFO("Forward dynamics solver initialized");
    ROS_INFO("Forward dynamics solver has control over %i joints", m_number_joints);

    return true;
}

bool CartesianMotionController::buildGenericModel(const KDL::Chain& input_chain)
{
    m_chain = input_chain;

    // Set all masses and inertias to minimal (yet stable) values.
    double m_min = 0.001;
    double ip_min = 0.000001;
    for (size_t i = 0; i < m_chain.segments.size(); ++i)
    {
        // Fixed joint segment
        if (m_chain.segments[i].getJoint().getType() == KDL::Joint::None)
        {
            m_chain.segments[i].setInertia(
                    KDL::RigidBodyInertia::Zero());
        }
        else  // relatively moving segment
        {
            m_chain.segments[i].setInertia(
                    KDL::RigidBodyInertia(
                            m_min,                // mass
                            KDL::Vector::Zero(),  // center of gravity
                            KDL::RotationalInertia(
                                    ip_min,             // ixx
                                    ip_min,             // iyy
                                    ip_min              // izz
                                    // ixy, ixy, iyz default to 0.0
                            )));
        }
    }

    // Only give the last segment a generic mass and inertia.
    // See https://arxiv.org/pdf/1908.06252.pdf for a motivation for this setting.
    double m = 1;
    double ip = 1;
    m_chain.segments[m_chain.segments.size()-1].setInertia(
            KDL::RigidBodyInertia(
                    m,
                    KDL::Vector::Zero(),
                    KDL::RotationalInertia(ip, ip, ip)));

    return true;
}

void CartesianMotionController::writeJointControlCmds()
{
    // Don't update position commands when paused.
    // Note: CartesianMotionControllers don't take any feedback from the joint
    // handles. These controllers will drift if the target frame they are
    // following isn't also paused.
    if (m_paused)
    {
        return;
    }

    // Take position commands
    for (size_t i = 0; i < m_joint_names.size(); ++i)
    {
        q_target[i] = m_simulated_joint_motion.positions[i];
//        m_joint_handles[i].setCommand(m_simulated_joint_motion.positions[i]);
    }
}

void CartesianMotionController::computeJointControlCmds(const ctrl::Vector6D& error, const ros::Duration& period)
{
    if (m_paused)
    {
        return;
    }

    // PD controlled system input
    m_cartesian_input = m_error_scale * m_spatial_controller(error,period);

    // Simulate one step forward
    m_simulated_joint_motion = getJointControlCmds(
            period,
            m_cartesian_input);

    // Update according to control policy for next cycle
    updateKinematics();
}

VectorXd CartesianMotionController::updateController(iDynTree::VectorDynSize rs)
{
    robotState.jointPos = rs;
    for (int i = 0; i < m_joint_names.size(); ++i) {
        m_current_positions(i) = rs.getVal(i);
    }
    // Forward Dynamics turns the search for the according joint motion into a
    // control process. So, we control the internal model until we meet the
    // Cartesian target motion. This internal control needs some simulation time
    // steps.
    ROS_INFO_STREAM_THROTTLE(1,"m_iterations: " << m_iterations);
    for (int i = 0; i < m_iterations; ++i)
    {
        // The internal 'simulation time' is deliberately independent of the outer
        // control cycle.
        ros::Duration internal_period(m_period);

        // Compute the motion error = target - current.
        ctrl::Vector6D error = computeMotionError();

        // Turn Cartesian error into joint motion
        computeJointControlCmds(error,internal_period);
    }

    // Write final commands to the hardware interface
    writeJointControlCmds();
    return q_target;
}

ctrl::Vector6D CartesianMotionController::computeMotionError()
{
    // Compute motion error wrt robot_base_link

    m_current_frame = getEndEffectorPose();

    ROS_INFO_STREAM_THROTTLE(1, "current frame " << m_current_frame.p[0] << "\t" << m_current_frame.p[1] << "\t" << m_current_frame.p[2]);
    ROS_INFO_STREAM_THROTTLE(1, "target_frame " << m_target_frame.p[0] << "\t" << m_target_frame.p[1] << "\t" << m_target_frame.p[2]);
    // Transformation from target -> current corresponds to error = target - current
    KDL::Frame error_kdl;
    error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
    error_kdl.p = m_target_frame.p - m_current_frame.p;

    // Use Rodrigues Vector for a compact representation of orientation errors
    // Only for angles within [0,Pi)
    KDL::Vector rot_axis = KDL::Vector::Zero();
    double angle    = error_kdl.M.GetRotAngle(rot_axis);   // rot_axis is normalized
    double distance = error_kdl.p.Normalize();

    // Clamp maximal tolerated error.
    // The remaining error will be handled in the next control cycle.
    const double max_angle = 1.0;
    const double max_distance = 1.0;
    angle    = boost::algorithm::clamp(angle,-max_angle,max_angle);
    distance = boost::algorithm::clamp(distance,-max_distance,max_distance);

    // Scale errors to allowed magnitudes
    rot_axis = rot_axis * angle;
    error_kdl.p = error_kdl.p * distance;

    // Reassign values
    ctrl::Vector6D error;
    error(0) = error_kdl.p.x();
    error(1) = error_kdl.p.y();
    error(2) = error_kdl.p.z();
    error(3) = rot_axis(0);
    error(4) = rot_axis(1);
    error(5) = rot_axis(2);

    return error;
}

void CartesianMotionController::targetFrameCallback(const geometry_msgs::PoseStamped& target)
{
    if (target.header.frame_id != m_robot_base_link)
    {
        ROS_WARN_STREAM_THROTTLE(3, "Got target pose in wrong reference frame. Expected: "
                << m_robot_base_link << " but got "
                << target.header.frame_id);
        return;
    }

    m_target_frame = KDL::Frame(
            KDL::Rotation::Quaternion(
                    target.pose.orientation.x,
                    target.pose.orientation.y,
                    target.pose.orientation.z,
                    target.pose.orientation.w),
            KDL::Vector(
                    target.pose.position.x,
                    target.pose.position.y,
                    target.pose.position.z));
}