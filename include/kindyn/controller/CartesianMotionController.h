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

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>

#include <vector>
#include <boost/shared_ptr.hpp>

// KDL
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include "kindyn/controller/SpatialPDController.h"
#include <Eigen/Dense>

#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>

#include "kindyn/controller/cardsflow_command_interface.hpp"
using namespace std;
class CartesianMotionController
{
public:
    CartesianMotionController();
    ~CartesianMotionController();

    bool init(ros::NodeHandle& nh);
    /**
     * @brief Compute joint target commands with approximate forward dynamics
     *
     * The resulting motion is the output of a forward dynamics simulation. It
     * can be forwarded to a real controller to mimic the simulated behavior.
     *
     * @param period The duration in sec for this simulation step
     * @param net_force The applied net force, expressed in the root frame
     *
     * @return A point holding positions, velocities and accelerations of each joint
     */
    trajectory_msgs::JointTrajectoryPoint getJointControlCmds(
            ros::Duration period,
            const ctrl::Vector6D& net_force);

    /**
     * @brief Get the current end effector pose of the simulated robot
     *
     * The last link in the chain from the init() function is taken as end
     * effector. If \ref setStartState() has been called immediately before,
     * then the returned pose represents the real robots end effector pose.
     *
     * @return The end effector pose with respect to the robot base link. This
     * link is the same as the one implicitly given in the init() function.
     */
    const KDL::Frame& getEndEffectorPose() const;

    /**
     * @brief Get the current end effector velocity of the simulated robot
     *
     * The last link in the chain from the init() function is taken as end
     * effector.
     *
     * @return The end effector vel with respect to the robot base link. The
     * order is first translation, then rotation.
     */
    const ctrl::Vector6D& getEndEffectorVel() const;

    /**
     * @brief Get the current joint positions of the simulated robot
     *
     * @return The current joint positions
     */
    const KDL::JntArray& getPositions() const;

    //! Set initial joint configuration
    bool setStartState(vector<double> jointPos);

    /**
     * @brief Initialize the solver
     *
     * @param chain The kinematic chain of the robot
     * @param upper_pos_limits Tuple with max positive joint angles
     * @param lower_pos_limits Tuple with max negative joint angles
     *
     * @return True, if everything went well
     */
    bool initSolver(const KDL::Chain& chain,
              const KDL::JntArray& upper_pos_limits,
              const KDL::JntArray& lower_pos_limits);

    /**
     * @brief Update the robot kinematics of the solver
     *
     * This template has two specializations for two distinct controller
     * policies, depending on the hardware interface used:
     *
     * 1) PositionJointInterface: The solver's internal simulation is continued
     * on each call without taking the real robot state into account.
     *
     * 2) VelocityJointInterface: The internal simulation is updated with the
     * real robot state. On each call, the solver starts with its internal
     * simulation in sync with the real robot.
     *
     * @tparam HardwareInterface
     * @param joint_handles
     */
    void updateKinematics();

    void writeJointControlCmds();

    void computeJointControlCmds(const ctrl::Vector6D& error, const ros::Duration& period);

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
    }robotState;

    VectorXd updateController(vector<double> rs);
    ctrl::Vector6D computeMotionError();
    void targetFrameCallback(const geometry_msgs::PoseStamped& target);

    boost::shared_ptr<KDL::TreeFkSolverPos_recursive>
            m_forward_kinematics_solver;
//    ForwardDynamicsSolver   m_forward_dynamics_solver;
    std::string             m_end_effector_link = "hand_right";
    std::string             m_robot_base_link = "torso";

    bool m_paused;
    int m_iterations;
    VectorXd q_target;
    KDL::Frame      m_target_frame;
    double m_period;

private:
    std::vector<hardware_interface::JointHandle>      m_joint_handles;
    std::vector<std::string>                          m_joint_names = { "shoulder_right_axis0",
                                                                   "shoulder_right_axis1",
                                                                    "shoulder_right_axis2",
                                                                    "elbow_right_axis0",
                                                                    "elbow_right_axis1",
                                                                    "wrist_right_axis0",
                                                                    "wrist_right_axis1",
                                                                    "wrist_right_axis2"};

    trajectory_msgs::JointTrajectoryPoint             m_simulated_joint_motion;
    SpatialPDController                              m_spatial_controller;
    ctrl::Vector6D                                    m_cartesian_input;
    double m_error_scale;

    // Against multi initialization in multi inheritance scenarios
    bool m_already_initialized;;

    // Dynamic reconfigure
//    typedef cartesian_controller_base::CartesianControllerConfig ControllerConfig;

//    void dynamicReconfigureCallback(ControllerConfig& config, uint32_t level);

//    boost::shared_ptr<dynamic_reconfigure::Server<ControllerConfig> > m_dyn_conf_server;
//    dynamic_reconfigure::Server<ControllerConfig>::CallbackType m_callback_type;

    //! Build a generic robot model for control
    bool buildGenericModel(const KDL::Chain& input_chain);

    //! The underlying physical system
    KDL::Chain m_chain;

    //! Number of controllable joint
    int m_number_joints;

    // Internal buffers
    KDL::JntArray m_current_positions;
    KDL::JntArray m_current_velocities;
    KDL::JntArray m_current_accelerations;
    KDL::JntArray m_last_positions;

    // Joint limits
    KDL::JntArray m_upper_pos_limits;
    KDL::JntArray m_lower_pos_limits;

    // Forward kinematics
    boost::shared_ptr<
    KDL::ChainFkSolverPos_recursive>  m_fk_pos_solver;
    boost::shared_ptr<
    KDL::ChainFkSolverVel_recursive>  m_fk_vel_solver;
    KDL::Frame                          m_end_effector_pose;
    ctrl::Vector6D                      m_end_effector_vel;

    // Forward dynamics
    boost::shared_ptr<KDL::ChainJntToJacSolver> m_jnt_jacobian_solver;
    boost::shared_ptr<KDL::ChainDynParam>       m_jnt_space_inertia_solver;
    KDL::Jacobian                               m_jnt_jacobian;
    KDL::JntSpaceInertiaMatrix                  m_jnt_space_inertia;


    KDL::Frame      m_current_frame;

};