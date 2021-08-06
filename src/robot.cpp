//
// Created by roboy on 01.07.21.
//

#include "kindyn/robot.hpp"

using namespace cardsflow::kindyn;

Robot::Robot() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "CARDSflow robot", ros::init_options::NoSigintHandler);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();
}

Robot::~Robot() {
}

void Robot::updatePublishers() {
    ROS_INFO_STREAM("advertising " << topic_root+"/robot_state");
    robot_state_pub = nh->advertise<geometry_msgs::PoseStamped>(topic_root+"control/robot_state", 1);
    tendon_state_pub = nh->advertise<roboy_simulation_msgs::Tendon>(topic_root+"control/tendon_state", 1);
    tendon_ext_state_pub = nh->advertise<roboy_simulation_msgs::Tendon>(topic_root+"control/tendon_state_ext", 1);

    joint_state_pub = nh->advertise<roboy_simulation_msgs::JointState>(topic_root+"control/rviz_joint_states", 1);
    cardsflow_joint_states_pub = nh->advertise<sensor_msgs::JointState>(topic_root+"control/cardsflow_joint_states", 1);

    robot_state_target_pub = nh->advertise<geometry_msgs::PoseStamped>(topic_root+"control/robot_state_target", 1);
    tendon_state_target_pub = nh->advertise<roboy_simulation_msgs::Tendon>(topic_root+"control/tendon_state_target", 1);
    joint_state_target_pub = nh->advertise<roboy_simulation_msgs::JointState>(topic_root+"control/joint_state_target", 1);
}

void Robot::updateSubscribers(){
    if (external_robot_state) {
        ROS_WARN("Subscribing to external joint state");
        joint_state_sub = nh->subscribe(topic_root+"/sensing/external_joint_states", 1, &Robot::JointState, this);
    }
    joint_target_sub = nh->subscribe(topic_root+"control/joint_targets", 100, &Robot::JointTarget, this);
    controller_type_sub = nh->subscribe("/controller_type", 100, &Robot::controllerType, this);
    freeze_srv = nh->advertiseService(topic_root+"control/freeze", &Robot::FreezeService, this);
    zero_joints_sub = nh->subscribe(topic_root+"control/zero_joints", 1, &Robot::ZeroJoints,this);
}

void Robot::init(string urdf_file_path, string viapoints_file_path, vector<string> joint_names_ordered) {

    updatePublishers();
    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");
    nh->getParam("external_robot_target", external_robot_target);
    nh->getParam("external_robot_state", external_robot_state);

    nh->getParam("link_relation_names", kinematics.link_relation_name);
    for (string lr:kinematics.link_relation_name) {
        ROS_INFO_STREAM(lr);
        vector<string> joint_names;
        nh->getParam(("link_relation/" + lr + "/joint_names"), joint_names);
        kinematics.joint_relation_name.push_back(joint_names);
    }

    kinematics.init(urdf_file_path, viapoints_file_path, joint_names_ordered);

    q.resize(kinematics.number_of_dofs);
    qd.resize(kinematics.number_of_dofs);
    qdd.resize(kinematics.number_of_dofs);

    q_target.resize(kinematics.number_of_dofs);
    qd_target.resize(kinematics.number_of_dofs);
    qdd_target.resize(kinematics.number_of_dofs);

    q_target_prev.resize(kinematics.number_of_dofs);
    qd_target_prev.resize(kinematics.number_of_dofs);
    qdd_target_prev.resize(kinematics.number_of_dofs);

    l_next.resize(kinematics.number_of_cables);
    l_target.resize(kinematics.number_of_cables);
    ld.resize(kinematics.number_of_dofs);
    for (int i = 0; i < kinematics.number_of_dofs; i++) {
        ld[i].resize(kinematics.number_of_cables);
        ld[i].setZero();
    }

    q_target.setZero();
    qd_target.setZero();
    qdd_target.setZero();

    q_target_prev.setZero();
    qd_target_prev.setZero();
    qdd_target_prev.setZero();
    l_next.setZero();
    l_target.setZero();

    controller_type.resize(kinematics.number_of_cables, CARDSflow::ControllerType::cable_length_controller);

    /**
     * Register ROS control
     */
    // PD gains for cable length controller

    kinematics.joint_dt.resize(kinematics.number_of_dofs);
    Kp_.resize(kinematics.number_of_dofs);
    Kd_.resize(kinematics.number_of_dofs);
    param_kp.resize(kinematics.number_of_dofs);
    param_kd.resize(kinematics.number_of_dofs);

    nh->getParam("joint_dt", kinematics.joint_dt);
    nh->getParam("joint_kp", param_kp);
    nh->getParam("joint_kd", param_kd);
    for (int joint = 0; joint < kinematics.number_of_dofs; joint++) {
        Kp_[joint] = param_kp[joint];
        Kd_[joint] = param_kd[joint];
        ROS_INFO_STREAM(kinematics.joint_names[joint] << "\tdt=" << kinematics.joint_dt[joint] <<
                        "\tkp=" << param_kp[joint] << "\tkd=" << param_kd[joint]);
    }

    for (int joint = 0; joint < kinematics.number_of_dofs; joint++) {
        ROS_INFO("initializing controllers for joint %d %s", joint, kinematics.joint_names[joint].c_str());
        // connect and register the cardsflow state interface
        hardware_interface::CardsflowStateHandle state_handle(kinematics.joint_names[joint], joint, &q[joint], &qd[joint],
                                                              &qdd[joint], &kinematics.L, &kinematics.M, &kinematics.CG

        );
        cardsflow_state_interface.registerHandle(state_handle);

        // connect and register the cardsflow command interface
        hardware_interface::CardsflowHandle pos_handle(cardsflow_state_interface.getHandle(kinematics.joint_names[joint]),
                                                       &q_target[joint], &qd_target[joint], &kinematics.torques[joint], &ld[joint],
                                                       &Kp_, &Kd_);
        cardsflow_command_interface.registerHandle(pos_handle);
    }
    registerInterface(&cardsflow_command_interface);

    /**
     * Initialize at zero for simulation
     */
    if(!this->external_robot_state){
        q.setZero();
        qd.setZero();
        qdd.setZero();

        kinematics.setRobotState(q, qd);
        kinematics.updateJacobians();
    }

    try {
        last_visualization =
                ros::Time::now() - ros::Duration(10); // triggers immediate visualization in first iteratiom
    }
    catch(std::runtime_error& ex) {
        ROS_ERROR("Exception: [%s]", ex.what());
    }

    int k=0;
    nh->getParam("endeffectors", kinematics.endeffectors);
    kinematics.endeffector_dof_offset.push_back(0);
    Ld.resize(kinematics.endeffectors.size());
    for (string ef:kinematics.endeffectors) {
        ROS_INFO_STREAM("configuring endeffector " << ef);
        vector<string> ik_joints;
        nh->getParam((ef + "/joints"), ik_joints);
        if (ik_joints.empty()) {
            ROS_WARN(
                    "endeffector %s has no joints defined, check your endeffector.yaml or parameter server.  skipping...",
                    ef.c_str());
            continue;
        }
        kinematics.endeffector_index[ef] = k;
        kinematics.endeffector_number_of_dofs.push_back(ik_joints.size());
        if(k>0)
            kinematics.endeffector_dof_offset.push_back(kinematics.endeffector_dof_offset[k-1]+kinematics.endeffector_number_of_dofs[k-1]);

        Ld[k].resize(kinematics.number_of_cables);
        Ld[k].setZero();
        k++;
    }

    updateSubscribers();
}

void Robot::update(){

    if (debug_) {
        if (nh->hasParam("joint_dt"))
            nh->getParam("joint_dt", kinematics.joint_dt);
        if (nh->hasParam("joint_kp")){
            nh->getParam("joint_kp", param_kp);
            for (int joint = 0; joint < kinematics.number_of_dofs; joint++) {
                Kp_[joint] = param_kp[joint];
            }
        }
        if (nh->hasParam("joint_kd")) {
            nh->getParam("joint_kd", param_kd);
            for (int joint = 0; joint < kinematics.number_of_dofs; joint++) {
                Kd_[joint] = param_kd[joint];
            }
        }
    }

    // TODO: Run the below code in critical section to avoid Mutex with the JointState and PROBABLY the controller

    /**
     * Update Current robot state
     */

    kinematics.setRobotState(q, qd);

    /**
     * Update Jacobians matrix L with current joint state
     */
    kinematics.updateJacobians();

    /**
     * Visualization
     */
    publishViz();

    // ----------------------------------------------------------------------------

    /**
     * Update current tendon length from controller
     */

    // Doing the reduce_sum, the latter step of doing dot product in the controller
    for(int i = 0; i< kinematics.endeffectors.size();i++) {
        Ld[i].setZero();
        int dof_offset = kinematics.endeffector_dof_offset[i];
        for (int j = dof_offset; j < kinematics.endeffector_number_of_dofs[i] + dof_offset; j++) {
            Ld[i] -= ld[j];
        }
    }

    // ----------------------------------------------------------------------------
    // Do one step forward Kinematics with current tendon velocity Ld and current state
    vector<VectorXd> state_next = kinematics.oneStepForward(q, qd, Ld);

    if(!this->external_robot_state){
        for(int i=0;i<kinematics.number_of_joints;i++){
            q[i] = state_next[0][i];
            qd[i] = state_next[1][i];
        }
    }

    if(!simulated){
        kinematics.setRobotState(state_next[0], state_next[1]);
        kinematics.getRobotCableFromJoints(l_next);
    }
    ROS_INFO_STREAM_THROTTLE(5, "q_target " << q_target.transpose().format(fmt));
}

void Robot::publishViz(){
    if ((1.0 / (ros::Time::now() - last_visualization).toSec()) < 30) {
        { // tendon state publisher
            roboy_simulation_msgs::Tendon msg;
            VectorXd l;
            l.resize(kinematics.number_of_cables);
            kinematics.getRobotCableFromJoints(l);
            for (int i = 0; i < kinematics.number_of_cables; i++) {
                msg.name.push_back(kinematics.cables[i].name);
                msg.force.push_back(kinematics.cable_forces[i]);
                msg.l.push_back(l[i]);
                msg.ld.push_back(Ld[0][i]); // TODO: only first endeffector Ld is send here
                msg.number_of_viapoints.push_back(kinematics.cables[i].viaPoints.size());
                for (auto vp:kinematics.cables[i].viaPoints) {
                    geometry_msgs::Vector3 VP;
                    tf::vectorEigenToMsg(vp->global_coordinates, VP);
                    msg.viapoints.push_back(VP);
                }
            }
            tendon_state_pub.publish(msg);
        }
        { // robot state publisher
            static int seq = 0;
            vector<Matrix4d> robot_poses = kinematics.getRobotPosesFromJoints();
            for (int i = 0; i < kinematics.number_of_links; i++) {
                geometry_msgs::PoseStamped msg;
                msg.header.seq = seq++;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = kinematics.link_names[i];
                Isometry3d iso(robot_poses[i]);
                tf::poseEigenToMsg(iso, msg.pose);
                robot_state_pub.publish(msg);
            }
        }
        { // robot target publisher
            if((q_target-q_target_prev).norm()>0.001 || (qd_target-qd_target_prev).norm()>0.001 || first_update) { // only if target changed
                if(first_update)
                    first_update = false;
                q_target_prev = q_target;
                qd_target_prev = qd_target;

                kinematics.setRobotState(q_target, qd_target);
                vector<Matrix4d> target_poses = kinematics.getRobotPosesFromJoints();
                static int seq = 0;
                for (int i = 0; i < kinematics.number_of_links; i++) {

                    geometry_msgs::PoseStamped msg;
                    msg.header.seq = seq++;
                    msg.header.stamp = ros::Time::now();
                    msg.header.frame_id = kinematics.link_names[i];
                    Isometry3d iso(target_poses[i]);
                    tf::poseEigenToMsg(iso, msg.pose);
                    robot_state_target_pub.publish(msg);
                }

                kinematics.getRobotCableFromJoints(l_target);
            }
        }
        { // joint state publisher
            sensor_msgs::JointState cf_msg;
            roboy_simulation_msgs::JointState msg;
            msg.names = kinematics.joint_names;
            cf_msg.name = kinematics.joint_names;
            cf_msg.header.stamp = ros::Time::now();

            kinematics.setRobotState(q, qd);

            for (int i = 1; i < kinematics.number_of_links; i++) {

                Matrix4d pose = kinematics.getPoseFromJoint(i);
                Vector3d axis;
                axis << kinematics.joint_axis[i - 1][3], kinematics.joint_axis[i - 1][4], kinematics.joint_axis[i - 1][5];
                axis = pose.block(0, 0, 3, 3) * axis;

                msg.origin.push_back(convertEigenToGeometry(pose.topRightCorner(3, 1)));
                msg.axis.push_back(convertEigenToGeometry(axis));
                msg.torque.push_back(kinematics.torques[i - 1]);
                msg.q.push_back(q[i-1]);
                msg.qd.push_back(qd[i-1]);

                cf_msg.position.push_back(q[i-1]);
                cf_msg.velocity.push_back(qd[i-1]);

            }
            joint_state_pub.publish(msg);
            cardsflow_joint_states_pub.publish(cf_msg);

        }
    }
}

void Robot::JointTarget(const sensor_msgs::JointStateConstPtr &msg){
    int i = 0;
    if (msg->position.size() == msg->name.size())
    {
        for (string joint:msg->name) {
            int joint_index = kinematics.GetJointIdByName(joint);
            if (joint_index != -1) {
                if (msg->position[i] > kinematics.q_max(joint_index)) {
                    q_target(joint_index)  = kinematics.q_max(joint_index);
                }
                else if (msg->position[i] < kinematics.q_min(joint_index)) {
                    q_target(joint_index)  = kinematics.q_min(joint_index);
                }
                else {
                    q_target(joint_index) = msg->position[i];
                }
//            ROS_WARN_STREAM(q_target(joint_index));
//            qd_target(joint_index) = msg->velocity[i];
            } else {
                ROS_WARN_THROTTLE(5.0, "joint %s not found in model", joint.c_str());
            }
            i++;
        }
    }

}

void Robot::JointState(const sensor_msgs::JointStateConstPtr &msg) {
    ROS_WARN_STREAM_THROTTLE(10,"external joint states sub");
    int i = 0;
    for (string joint:msg->name) {
        if (std::count(kinematics.joint_names.begin(), kinematics.joint_names.end(), joint)) {
            int joint_index = kinematics.GetJointIdByName(joint);
            if (joint_index != -1) {
                q(joint_index) = msg->position[i];
//                qd(joint_index) = msg->velocity[i];
            } else {
                ROS_ERROR("joint %s not found in model", joint.c_str());
            }
        }
        i++;
    }
}

void Robot::controllerType(const roboy_simulation_msgs::ControllerTypeConstPtr &msg) {
    auto it = find(kinematics.joint_names.begin(), kinematics.joint_names.end(),msg->joint_name);
    if(it!=kinematics.joint_names.end()) {
        ROS_INFO("%s changed controller to %s", msg->joint_name.c_str(),
                 (msg->type==CARDSflow::ControllerType::cable_length_controller?"cable_length_controller":
                  msg->type==CARDSflow::ControllerType::torque_position_controller?"torque_position_controller":
                  msg->type==CARDSflow::ControllerType::force_position_controller?"force_position_controller":"UNKNOWN"));
        controller_type[distance(kinematics.joint_names.begin(), it)] = msg->type;
    }
}

bool Robot::FreezeService(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res) {
    for (int i = 1; i < kinematics.number_of_links; i++) {
        q_target = q;
    }
    res.message = "Robot stopped until the next joint target message";
    res.success = true;
    return true;

}

void Robot::ZeroJoints(const roboy_control_msgs::StringsPtr &msg) {
    //ROS_WARN(msg->names);
    if (msg->names.empty()) {
        ROS_WARN_STREAM("Setting all joint targets to zero");
        //for (int i = 1; i < number_of_links; i++) {
        q_target.setZero();
        //}
    }
    else {
        for (string joint: msg->names) {
            ROS_INFO_STREAM("zero " << joint);
            int joint_index = kinematics.GetJointIdByName(joint);
            if (joint_index != iDynTree::JOINT_INVALID_INDEX) {
                q_target(joint_index) = 0;
                //ROS_INFO_STREAM("done");
            }
        };
    }
}

