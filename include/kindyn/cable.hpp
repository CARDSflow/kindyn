#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

namespace cardsflow {
    namespace kindyn {

        struct ViaPoint {
            ViaPoint(string link_name, Vector3d &local_coordinates) :
                    link_name(link_name), local_coordinates(local_coordinates),
                    global_coordinates(local_coordinates) {};
            string link_name;
            int link_index;
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