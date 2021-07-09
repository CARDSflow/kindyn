//
// Created by roboy on 01.07.21.
//

#include "kindyn/robot_new.hpp"

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
    ekf_joint_states_pub = nh->advertise<sensor_msgs::JointState>(topic_root+"control/ekf_joint_states", 1);

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
//     joint_state_sub = nh->subscribe("/joint_states", 100, &Robot::JointState, this);
//    floating_base_sub = nh->subscribe(topic_root+"control/floating_base", 100, &Robot::FloatingBase, this);
//    ik_srv = nh->advertiseService(topic_root+"control/ik", &Robot::InverseKinematicsService, this);
//    ik_two_frames_srv = nh->advertiseService(topic_root+"control/ik_multiple_frames", &Robot::InverseKinematicsMultipleFramesService, this);
//    fk_srv = nh->advertiseService(topic_root+"control/fk", &Robot::ForwardKinematicsService, this);
//    freeze_srv = nh->advertiseService(topic_root+"control/freeze", &Robot::FreezeService, this);
//    interactive_marker_sub = nh->subscribe(topic_root+"control/interactive_markers/feedback",1,&Robot::InteractiveMarkerFeedback, this);
//    zero_joints_sub = nh->subscribe(topic_root+"control/zero_joints", 1, &Robot::ZeroJoints,this);
}

void Robot::init(string urdf_file_path, string viapoints_file_path, vector<string> joint_names_ordered) {

    updatePublishers();
    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");
    nh->getParam("external_robot_target", external_robot_target);
    nh->getParam("external_robot_state", external_robot_state);
    nh->setParam("vr_puppet",false);

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
    Kp_dl = new double(100.0);
    Kd_dl = new double(0.0);
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
                                                       Kp_dl, Kd_dl);
        cardsflow_command_interface.registerHandle(pos_handle);
    }
    registerInterface(&cardsflow_command_interface);

    ekf_ = new BFL::KinDynEKF(kinematics.number_of_joints);

    /**
     * Initialize at zero for simulation
     */
    if(simulated){
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

    if (nh->hasParam("k_dt"))
        nh->getParam("k_dt", k_dt);
    if (nh->hasParam("Kp_dl"))
        nh->getParam("Kp_dl", *Kp_dl);
    if (nh->hasParam("Kd_dl"))
        nh->getParam("Kd_dl", *Kd_dl);

    // TODO: Run the below code in critical section to avoid Mutex with the JointState and PROBABLY the controller

    /**
     * Update Current robot state
     */

    VectorXd q_ekf, qd_ekf;
    q_ekf.resize(kinematics.number_of_dofs);
    qd_ekf.resize(kinematics.number_of_dofs);

    if(!simulated) {
        if (ekf_->isInitialized()) {
            ekf_->sensor_update(q);
            ekf_->getEstimate(q_ekf, qd_ekf);
        } else {
            ros::Duration(1.0).sleep();
            ekf_->initialize(q, k_dt);

            q_ekf = q;
            qd_ekf = qd;
        }

        kinematics.setRobotState(q_ekf, qd_ekf);
    }else{
        kinematics.setRobotState(q, qd);
    }

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

    if(simulated){
        // Do one step forward Kinematics with current tendon velocity Ld and current state
        vector<VectorXd> state_next = kinematics.oneStepForward(k_dt, q, qd, Ld);
        for(int i=0;i<kinematics.number_of_joints;i++){
            q[i] = state_next[0][i];
            qd[i] = state_next[1][i];
        }
    }else{
        vector<VectorXd> state_next = kinematics.oneStepForward(k_dt, q_ekf, qd_ekf, Ld);
        // Currently qd is zero, so the filter is basically more try to use the most recent q
        // The correct way should be state_next[1]. However, when it's not moving,
        // state_next[1] try to capture the target, but q_external holds it back again !!!
        ekf_->model_update(k_dt, qd);

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

