#include "kindyn/controller/cardsflow_command_interface.hpp"

namespace hardware_interface
{


    CardsflowHandle::CardsflowHandle() : CardsflowStateHandle(){}

    /**
     * @param js This joint's state handle
     * @param cmd A pointer to the storage for this joint's output command
     */
    CardsflowHandle::CardsflowHandle(const CardsflowStateHandle& js, double* joint_position_cmd,
                                     double* joint_velocity_cmd, double* joint_torque_cmd, VectorXd *motor_cmd)
            : CardsflowStateHandle(js), joint_position_cmd_(joint_position_cmd), joint_velocity_cmd_(joint_velocity_cmd),
              joint_torque_cmd_(joint_torque_cmd), motor_cmd_(motor_cmd)
    {
    }

    void CardsflowHandle::setMotorCommand(VectorXd command) {*motor_cmd_ = command;}
    double CardsflowHandle::getJointPositionCommand() const {return *joint_position_cmd_;}
    double CardsflowHandle::getJointVelocityCommand() const {return *joint_velocity_cmd_;}
    double CardsflowHandle::getJointTorqueCommand() const {return *joint_torque_cmd_;}
    void CardsflowHandle::setJointPositionCommand(double cmd){*joint_position_cmd_ = cmd;}
    void CardsflowHandle::setJointVelocityCommand(double cmd){*joint_velocity_cmd_ = cmd;}
    void CardsflowHandle::setJointTorqueCommand(double cmd){*joint_torque_cmd_ = cmd;}
}
