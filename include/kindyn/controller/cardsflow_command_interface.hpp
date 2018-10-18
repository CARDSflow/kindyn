#pragma once

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "kindyn/controller/cardsflow_state_interface.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
    class CardsflowHandle : public CardsflowStateHandle
    {
    public:
        CardsflowHandle();

        /**
         * \param js This joint's state handle
         * \param cmd A pointer to the storage for this joint's output command
         */
        CardsflowHandle(const CardsflowStateHandle& js, double* joint_position_cmd, double* joint_velocity_cmd, VectorXd *motor_cmd);

        void setMotorCommand(VectorXd command);
        double getJointPositionCommand() const;
        double getJointVelocityCommand() const;
        void setJointPositionCommand(double cmd);
        void setJointVelocityCommand(double cmd);

    private:
        double *joint_position_cmd_, *joint_velocity_cmd_;
        VectorXd *motor_cmd_;
    };

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortJointInterface etc.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
    class CardsflowCommandInterface : public HardwareResourceManager<CardsflowHandle, ClaimResources> {};

}