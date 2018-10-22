#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

namespace cardsflow {
    namespace kindyn {

        struct ViaPoint {
            ViaPoint(string link_name, Vector3d &local_coordinates, bool fixed_to_world = false) :
                    link_name(link_name), local_coordinates(local_coordinates),
                    global_coordinates(local_coordinates),fixed_to_world(fixed_to_world) {};
            string link_name;
            int link_index;
            bool fixed_to_world;
            Vector3d local_coordinates;
            Vector3d global_coordinates;
        };

        typedef boost::shared_ptr<ViaPoint> ViaPointPtr;

        struct Cable {
            string name;
            vector<ViaPointPtr> viaPoints;
        };

    }
}