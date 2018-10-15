#include <type_traits>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "kindyn/robot.hpp"
#include "kindyn/controller/cardsflow_state_interface.hpp"

using namespace std;

class CableLengthController : public controller_interface::Controller<hardware_interface::CardsflowCommandInterface> {
public:
    CableLengthController() {

    };

    bool init(hardware_interface::CardsflowCommandInterface *hw, ros::NodeHandle &n) {
        nh = n;
        // get joint name from the parameter server
        if (!n.getParam("joint_name", joint_name)) {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        joint = hw->getHandle(joint_name); // throws on failure
        joint_index = joint.getJointIndex();
        return true;
    }

    void update(const ros::Time &time, const ros::Duration &period) {
        double q = joint.getPosition();
        double qd = joint.getVelocity();
        double q_target = joint.getJointPositionCommand();
        double qd_target = joint.getJointVelocityCommand();
        MatrixXd L = joint.getL();
        VectorXd ld = L.col(joint_index) * (Kd * (qd_target - qd) + Kp * (q_target - q));
//        ROS_INFO_STREAM_THROTTLE(1, ld);
        joint.setMotorCommand(ld);
    }

private:
    double Kp = 100, Kd = 0;
    ros::NodeHandle nh;
    hardware_interface::CardsflowHandle joint;
    string joint_name;
    int joint_index;
};
PLUGINLIB_EXPORT_CLASS(CableLengthController, controller_interface::ControllerBase);
