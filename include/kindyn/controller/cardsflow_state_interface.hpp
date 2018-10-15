#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

namespace hardware_interface
{

/** A handle used to read the state of a single joint. */
    class CardsflowStateHandle
    {
    public:
        CardsflowStateHandle();

        /**
         * \param name The name of the joint
         * \param pos joint position
         * \param vel joint velocity
         * \param dis motor displacements
         * \param L cardsflow matrix
         * \param M mass matrix
         * \param CG bias vector
         */
        CardsflowStateHandle(const std::string& name, int joint_index, const double* pos, const double* vel, const double* acc,
                             const MatrixXd *L, const MatrixXd *M, const VectorXd *CG);

        std::string getName() const;
        int getJointIndex()  const;
        double getPosition()  const;
        double getVelocity()  const;
        double getAcceleration()  const;
        MatrixXd getL()    const;
        MatrixXd getM()    const;
        VectorXd getCG()    const;

    private:
        std::string name_;
        int joint_index_;
        const double* pos_;
        const double* vel_;
        const double* acc_;
        const VectorXd *CG_;
        const MatrixXd *L_, *M_;
    };

/** \brief Hardware interface to support reading the state of an array of joints
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * joints, each of which has some position, velocity, and effort (force or
 * torque).
 *
 */
    class CardsflowStateInterface : public HardwareResourceManager<CardsflowStateHandle> {};

}