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

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

namespace hardware_interface {

    class CardsflowStateHandle {
    public:
        CardsflowStateHandle();

        /**
         * @param name The name of the joint
         * @param joint_index index of the joint in the robot model
         * @param pos joint position
         * @param vel joint velocity
         * @param acc joint acceleration
         * @param L pointer to the L matrix
         * @param M pointer to the mass matrix
         * @param CG pointer to the Coriolis+Gravity vector
        */
        CardsflowStateHandle(const std::string &name, int joint_index, const double *pos, const double *vel,
                             const double *acc, const MatrixXd *L, const MatrixXd *M, const VectorXd *CG);

        /**
         * Returns the joint name
         * @return joint name
         */
        std::string getName() const;

        /**
         * Returns the joint index in the robot model
         * @return joint index
         */
        int getJointIndex() const;

        /**
         * Returns the currect joint position
         * @return joint position
         */
        double getPosition() const;

        /**
         * Returns the currect joint velocity
         * @return joint velocity
         */
        double getVelocity() const;

        /**
         * Returns the currect joint acceleration
         * @return joint acceleration
         */
        double getAcceleration() const;

        /**
         * Returns the cable model L matrix
         * @return L
         */
        MatrixXd getL() const;

        /**
         * Returns the robot Mass matrix
         * @return M
         */
        MatrixXd getM() const;

        /**
         * Returns the robot Coriolis+Gravity vector
         * @return CG
         */
        VectorXd getCG() const;

    private:
        std::string name_; /// joint name
        int joint_index_; /// joint index
        const double *pos_; /// joint position
        const double *vel_; /// joint velocity
        const double *acc_; /// joint acceleration
        const VectorXd *CG_; /// Coriolis+Gravity vector
        const MatrixXd *L_, *M_; /// L/M matrices
    };

    class CardsflowStateInterface : public HardwareResourceManager<CardsflowStateHandle> {
    };

}