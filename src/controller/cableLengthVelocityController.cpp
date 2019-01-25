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
    description: A Cable length controller for joint position targets using PD control
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

class CableLengthVelocityController : public controller_interface::Controller<hardware_interface::CardsflowCommandInterface> {
public:
    /**
     * Constructor
     */
    CableLengthVelocityController() {};

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
        while(controller_state.getNumSubscribers()==0) // we wait until the controller state is available
            r.sleep();
        joint = hw->getHandle(joint_name); // throws on failure
        joint_index = joint.getJointIndex();
        last_update = ros::Time::now();
        joint_command = nh.subscribe((joint_name+"/target").c_str(),1,&CableLengthVelocityController::JointVelocityCommand, this);
        controller_parameter_srv = nh.advertiseService((joint_name+"/params").c_str(),& CableLengthVelocityController::setControllerParameters, this);
        return true;
    }

    /**
     * Called regularily by controller manager. The length change of the cables wrt to a PD controller on the joint target
     * position is calculated.
     * @param time current time
     * @param period period since last control
     */
    void update(const ros::Time &time, const ros::Duration &period) {
        double qd = joint.getVelocity();
        double qd_target = joint.getJointVelocityCommand();
        MatrixXd L = joint.getL();
        double p_error = qd - qd_target;
        // we use the joint_index column of the L matrix to calculate the result for this joint only
        VectorXd ld = L.col(joint_index) * (Kd * (p_error - p_error_last)/period.toSec() + Kp * p_error);
//        ROS_INFO_STREAM_THROTTLE(1, ld.transpose());
        joint.setMotorCommand(ld);
        p_error_last = p_error;
        last_update = time;
    }

    /**
     * Called by controller manager when the controller is about to be started
     * @param time current time
     */
    void starting(const ros::Time& time) {
        ROS_WARN("cable velocity controller started for %s with index %d", joint_name.c_str(), joint_index);
        roboy_simulation_msgs::ControllerType msg;
        msg.joint_name = joint_name;
        msg.type = CARDSflow::ControllerType::cable_length_controller;
        controller_state.publish(msg);
    }
    /**
     * Called by controller manager when the controller is about to be stopped
     * @param time current time
     */
    void stopping(const ros::Time& time) { ROS_WARN("cable velocity controller stopped for %s", joint_name.c_str());}

    /**
     * Joint position command callback for this joint
     * @param msg joint position target in radians
     */
    void JointVelocityCommand(const std_msgs::Float32ConstPtr &msg){
        joint.setJointVelocityCommand(msg->data);
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
    double Kp = 0.1, Kd = 0; /// PD gains
    double p_error_last = 0; /// last error
    ros::NodeHandle nh; /// ROS nodehandle
    ros::Publisher controller_state; /// publisher for controller state
    ros::ServiceServer controller_parameter_srv; /// service for controller parameters
    boost::shared_ptr<ros::AsyncSpinner> spinner; /// ROS async spinner
    hardware_interface::CardsflowHandle joint; /// cardsflow joint handle for access to joint/cable model state
    ros::Subscriber joint_command; /// joint command subscriber
    string joint_name; /// name of the controlled joint
    int joint_index; /// index of the controlled joint in the robot model
    ros::Time last_update; /// time of last update
};
PLUGINLIB_EXPORT_CLASS(CableLengthVelocityController, controller_interface::ControllerBase);
