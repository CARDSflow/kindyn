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

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_box.h>
#include <string>
#include "kindyn/controller/PDController.h"

namespace ctrl{

    typedef Eigen::Matrix<double,6,1> Vector6D;

    typedef Eigen::VectorXd VectorND;

    typedef Eigen::Vector3d Vector3D;

    typedef Eigen::MatrixXd MatrixND;

    typedef Eigen::Matrix3d Matrix3D;

    typedef Eigen::Matrix<double,6,6> Matrix6D;

}


class SpatialPDController
{
public:
    SpatialPDController(){};

    bool init(ros::NodeHandle& nh){
        // Initialize pd controllers for each Cartesian dimension
        for (int i = 0; i < 6; ++i) // 3 transition, 3 rotation
        {
            m_pd_controllers.push_back(PDController());
        }

        // Load default controller gains
        std::string solver_config = nh.getNamespace() + "/pd_gains";

        m_pd_controllers[0].init(solver_config + "/trans_x");
        m_pd_controllers[1].init(solver_config + "/trans_y");
        m_pd_controllers[2].init(solver_config + "/trans_z");
        m_pd_controllers[3].init(solver_config + "/rot_x");
        m_pd_controllers[4].init(solver_config + "/rot_y");
        m_pd_controllers[5].init(solver_config + "/rot_z");

        return true;
    };

    /**
     * @brief Call operator for one control cycle
     *
     * @param error The control error to reduce. Target - current.
     * @param period The period for this control step.
     *
     * @return The controlled 6-dim vector (translational, rotational).
     */
    ctrl::Vector6D operator()(const ctrl::Vector6D& error, const ros::Duration& period) {
        // Perform pd control separately on each Cartesian dimension
        for (int i = 0; i < 6; ++i) // 3 transition, 3 rotation
        {
            m_cmd(i) = m_pd_controllers[i](error[i], period);
        }
        return m_cmd;
    };

private:
    ctrl::Vector6D m_cmd;
    std::vector<PDController> m_pd_controllers;

};
