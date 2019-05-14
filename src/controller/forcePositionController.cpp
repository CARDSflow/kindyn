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
    description: A Force controller for joint position targets using PD control
*/

#include <type_traits>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "kindyn/robot.hpp"
#include "kindyn/controller/cardsflow_state_interface.hpp"
#include <roboy_simulation_msgs/ControllerType.h>
#include <std_msgs/Float32.h>
#include <roboy_control_msgs/SetControllerParameters.h>

using namespace std;
using namespace Eigen;

class ForcePositionController : public controller_interface::Controller<hardware_interface::CardsflowCommandInterface> {
public:
    /**
     * Constructor
     */
    ForcePositionController() {};

    /**
     * Initializes the controller. Will be call by controller_manager when loading this controller
     * @param hw pointer to the hardware interface
     * @param n the nodehandle
     * @return success
     */
    bool init(hardware_interface::CardsflowCommandInterface *hw, ros::NodeHandle &n) {
        nh = n;
        // get joint name from the parameter server
        if (!n.getParam("joint", joint_name)) {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }
        spinner.reset(new ros::AsyncSpinner(0));
        spinner->start();
        controller_state = nh.advertise<roboy_simulation_msgs::ControllerType>("/controller_type",1);
        ros::Rate r(10);
        while(controller_state.getNumSubscribers()==0)
            r.sleep();
        joint = hw->getHandle(joint_name); // throws on failure
        joint_command = nh.subscribe((joint_name+"/target").c_str(),1,&ForcePositionController::JointPositionCommand, this);
        joint_index = joint.getJointIndex();
        controller_parameter_srv = nh.advertiseService((joint_name+"/params").c_str(),& ForcePositionController::setControllerParameters, this);

        return true;
    }

    /**
     * Called regularily by controller manager. The torque for a joint wrt to a PD controller on the joint target
     * position is calculated.
     * @param time current time
     * @param period period since last control
     */
    void update(const ros::Time &time, const ros::Duration &period) {
        double q = joint.getPosition();
        double p_error = q - q_target;
        double q_dd_cmd = Kp * p_error + Kd *((p_error - p_error_prev)/period.toSec());
        MatrixXd M = joint.getM();
        VectorXd CG = joint.getCG();
        double torque = M(joint_index,joint_index) * q_dd_cmd + CG[joint_index];
        joint.setJointTorqueCommand(torque);
//        ROS_INFO_THROTTLE(1, "%s target %lf current %lf", joint_name.c_str(), q_target, q);
        p_error_prev = p_error;
    }

    /**
     * Called by controller manager when the controller is about to be started
     * @param time current time
     */
    void starting(const ros::Time& time) {
        ROS_WARN("force position controller started for %s", joint_name.c_str());
        roboy_simulation_msgs::ControllerType msg;
        msg.joint_name = joint_name;
        msg.type = CARDSflow::ControllerType::force_position_controller;
        controller_state.publish(msg);
    }

    /**
     * Called by controller manager when the controller is about to be stopped
     * @param time current time
     */
    void stopping(const ros::Time& time) { ROS_WARN("force length controller stopped for %s", joint_name.c_str());}

    /**
     * Joint position command callback for this joint
     * @param msg joint position target in radians
     */
    void JointPositionCommand(const std_msgs::Float32ConstPtr &msg){
        q_target = msg->data;
    }

    /**
     * Controller Parameters service
     * @param req requested gains
     * @param res success
     * @return success
     */
    bool setControllerParameters( roboy_control_msgs::SetControllerParameters::Request &req,
                                  roboy_control_msgs::SetControllerParameters::Response &res){
        Kp = req.kp;
        Kd = req.kd;
        res.success = true;
        return true;
    }
private:
    double q_target = 0; /// joint position target
    double p_error_prev = 0;
    double Kp = 1, Kd = 0; /// PD gains
    ros::NodeHandle nh; /// ROS nodehandle
    ros::Publisher controller_state; /// publisher for controller state
    ros::ServiceServer controller_parameter_srv; /// service for controller parameters
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    hardware_interface::CardsflowHandle joint; /// cardsflow joint handle for access to joint/cable model state
    ros::Subscriber joint_command; /// joint command subscriber
    string joint_name; /// name of the controlled joint
    int joint_index;
};
PLUGINLIB_EXPORT_CLASS(ForcePositionController, controller_interface::ControllerBase);
