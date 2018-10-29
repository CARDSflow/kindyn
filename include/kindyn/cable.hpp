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
    description: Cable and Viapoint classes for CARDSflow robots
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

namespace cardsflow {
    namespace kindyn {

        struct ViaPoint {
            /**
             *
             * @param link_name
             * @param local_coordinates
             * @param fixed_to_world
             */
            ViaPoint(string link_name, Vector3d &local_coordinates, bool fixed_to_world = false) :
                    link_name(link_name), local_coordinates(local_coordinates),
                    global_coordinates(local_coordinates),fixed_to_world(fixed_to_world) {};
            string link_name; /// the name of the link the viapoint belongs to
            int link_index; /// the index of the link the viapoint belongs to wrt. the robot model
            bool fixed_to_world; /// indicates if the viapoint if moving with the link or is fixed to the world
            Vector3d local_coordinates; /// the local coordinates in the link frame
            Vector3d global_coordinates; /// the globale coordinates of the viapoint
        };

        typedef boost::shared_ptr<ViaPoint> ViaPointPtr;

        struct Cable {
            string name; /// name of the cable
            vector<ViaPointPtr> viaPoints; /// viapoints it consists of
        };

    }
}