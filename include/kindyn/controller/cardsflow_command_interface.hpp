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
    description: A custom ros control command interface for CARDSflow robots
*/

#pragma once

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include "kindyn/controller/cardsflow_state_interface.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

namespace CARDSflow {
    enum ControllerType {
        cable_length_controller,
        torque_position_controller,
        force_position_controller
    };
}

namespace hardware_interface {

    class CardsflowHandle : public CardsflowStateHandle {
    public:
        CardsflowHandle();

        /**
         * @param js joint state handle
         * @param joint_position_cmd joint position command
         * @param joint_velocity_cmd joint velocity command
         * @param joint_torque_cmd joint torque command
         * @param motor_cmd cable command
         */
        CardsflowHandle(const CardsflowStateHandle &js, double *joint_position_cmd, double *joint_velocity_cmd,
                        double* joint_torque_cmd, VectorXd *motor_cmd);

        /**
         * Cable length command
         * @param command
         */
        void setMotorCommand(VectorXd command);

        /**
         * Returns the joint position command
         * @return joint position command
         */
        double getJointPositionCommand() const;

        /**
         * Returns the joint velocity command
         * @return joint velocity command
         */
        double getJointVelocityCommand() const;

        /**
         * Returns the joint torque command
         * @return joint torque command
         */
        double getJointTorqueCommand() const;

        /**
         * Sets the joint position command
         * @param cmd joint position command
         */
        void setJointPositionCommand(double cmd);

        /**
         * Sets the joint velocity command
         * @param cmd joint velocity command
         */
        void setJointVelocityCommand(double cmd);

        /**
         * Sets the joint torque command
         * @param cmd joint torque command
         */
        void setJointTorqueCommand(double cmd);

    private:
        double *joint_position_cmd_, *joint_velocity_cmd_, *joint_torque_cmd_; /// joint position/velocity/torque command
        VectorXd *motor_cmd_; /// cable command
    };

    class CardsflowCommandInterface : public HardwareResourceManager<CardsflowHandle, ClaimResources> {
    };

}