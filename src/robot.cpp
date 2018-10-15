#include "kindyn/robot.hpp"

using namespace cardsflow::kindyn;

Robot::Robot(string urdf_file_path, string viapoints_file_path) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "CARDSflow robot", ros::init_options::NoSigintHandler);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    robot_state_pub = nh->advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1);
    joint_state_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);

    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");

    // Helper class to load the model from an external format
    iDynTree::ModelLoader mdlLoader;

    bool ok = mdlLoader.loadModelFromFile(urdf_file_path);

    if( !ok )
    {
        ROS_FATAL_STREAM("KinDynComputationsWithEigen: impossible to load model from " << urdf_file_path);
        return;
    }

    // Create a KinDynComputations class from the model
    ok = kinDynComp.loadRobotModel(mdlLoader.model());

    if( !ok )
    {
        ROS_FATAL_STREAM("KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString());
        return;
    }

    ROS_INFO_STREAM(kinDynComp.getDescriptionOfDegreesOfFreedom());

//    kinDynComp.setFloatingBase("base");
    const iDynTree::Model & model = kinDynComp.model();
    number_of_dofs = model.getNrOfDOFs();
    number_of_joints = model.getNrOfJoints();
    number_of_links = model.getNrOfLinks();

    if(!parseViapoints(viapoints_file_path, cables)){
        ROS_FATAL("something went wrong parsing the viapoints");
        return;
    }
    number_of_cables = cables.size();

    ROS_INFO_STREAM("robot:\ndofs: " << number_of_dofs <<  "\njoints: " << number_of_joints << "\nlinks: " << number_of_links << "\nnumber_of_cables: " << number_of_cables);

    baseVel.setZero();
    world_H_base.setIdentity();
    gravity << 0,0,-9.81;

    number_of_cables = 0;
    // get the link names of the kinematic chain
    int j = 0;
    for (int link = 0; link < number_of_links; link++) {
        string link_name = model.getLinkName(link);
        link_names.push_back(link_name);
        ROS_INFO_STREAM(link_name);
        link_index[link_name] = link;
    }
    for (int joint = 0; joint < number_of_dofs; joint++) {
        iDynTree::Vector6 s = model.getJoint(joint)->getMotionSubspaceVector(0,model.getJoint(joint)->getSecondAttachedLink(),model.getJoint(joint)->getFirstAttachedLink()).asVector();
        string joint_name = model.getJointName(joint);
        joint_names.push_back(joint_name);
        VectorXd axis = iDynTree::toEigen(s);
        joint_axis.push_back(axis);
        ROS_INFO_STREAM(joint_name << " " << axis.format(fmt));
        joint_index[joint_name] = joint;
//        if (joint_axis.back()(0) == 0 && joint_axis.back()(1) == 0 && joint_axis.back()(2) == 0)
//            joint_types.push_back(PRISMATIC);
//        else
//            joint_types.push_back(REVOLUTE);
//        j++;
    }

    init();
}

void Robot::init(){
    q.resize(number_of_dofs);
    q.setZero();
    qd.resize(number_of_dofs);
    qd.setZero();
    qdd.resize(number_of_dofs);
    qdd.setZero();

    q_target.resize(number_of_dofs);
    q_target.setZero();
    q_target[0] = 0.1;
    qd_target.resize(number_of_dofs);
    qd_target.setZero();
    qdd_target.resize(number_of_dofs);
    qdd_target.setZero();

    e.resize(number_of_dofs);
    e.setZero();
    de.resize(number_of_dofs);
    de.setZero();
    dde.resize(number_of_dofs);
    dde.setZero();

    torques.resize(number_of_dofs);
    torques.setZero();

    M.resize(number_of_dofs+6,number_of_dofs+6);
    CG.resize(number_of_dofs);
    CG.setZero();

    bias = iDynTree::FreeFloatingGeneralizedTorques(kinDynComp.model());
    Mass = iDynTree::MatrixDynSize(number_of_dofs+6,number_of_dofs+6);

    V.resize(number_of_cables, 6 * number_of_links);
    V.setZero(number_of_cables, 6 * number_of_links);
    P.resize(6 * number_of_links, 6 * number_of_links);
    P.setZero(6 * number_of_links, 6 * number_of_links);
    S.resize(6 * number_of_links, number_of_dofs);
    S.setZero(6 * number_of_links, number_of_dofs);

    segments.resize(number_of_cables);

    int muscle_index = 0;
    for (auto muscle:cables) {
        for (int i = 1; i < muscle.viaPoints.size(); i++) {
            pair<ViaPointPtr, ViaPointPtr> segment(muscle.viaPoints[i - 1], muscle.viaPoints[i]);
            segments[muscle_index].push_back(segment);
        }
        muscle_index++;
    }

    update_S();
    update_V();
    update_P();

    W = P * S;
    L = V * W;
    L_t = -L.transpose();
}

void Robot::update(double period) {
    forwardKinematics(period);

    const iDynTree::Model & model = kinDynComp.model();

    int i = 0;
    for (auto &link_name:link_names) {
        Matrix4d pose = iDynTree::toEigen(model.getFrameTransform(i).asHomogeneousTransform());
        Vector3d com = iDynTree::toEigen(model.getLink(i)->getInertia().getCenterOfMass());
        pose.block(0, 3, 3, 1) += pose.block(0, 0, 3, 3) * com;
        world_to_link_transform[link_name] = pose.inverse();
        i++;
    }

    for (auto muscle:cables) {
        for (auto vp:muscle.viaPoints) {
            Matrix4d transform = world_to_link_transform[vp->link_name].inverse();
            vp->global_coordinates = transform.block(0, 3, 3, 1) + transform.block(0, 0, 3, 3) * vp->local_coordinates;
        }
    }

    update_V();
    update_P();

    W = P * S;
    L = V * W;
    L_t = -L.transpose();

//    std_msgs::Float64MultiArray msg[7];
//
//    grid_map::matrixEigenCopyToMultiArrayMessage(M, msg[0]);
//    grid_map::matrixEigenCopyToMultiArrayMessage(G, msg[1]);
//    grid_map::matrixEigenCopyToMultiArrayMessage(C, msg[2]);
//    grid_map::matrixEigenCopyToMultiArrayMessage(L, msg[3]);
//    grid_map::matrixEigenCopyToMultiArrayMessage(V, msg[4]);
//    grid_map::matrixEigenCopyToMultiArrayMessage(S, msg[5]);
//    grid_map::matrixEigenCopyToMultiArrayMessage(P, msg[6]);
//
//    M_pub.publish(msg[0]);
//    C_pub.publish(msg[2]);
//    G_pub.publish(msg[1]);
//    L_pub.publish(msg[3]);
//    V_pub.publish(msg[4]);
//    S_pub.publish(msg[5]);
//    P_pub.publish(msg[6]);
}

void Robot::updateController() {
    static int counter = 0;

    vector<double> q_target_;
    nh->getParam("q_target", q_target_);
    for(int i=0;i<q_target.rows();i++)
        q_target[i] = q_target_[i];

    nh->getParam("Kp", Kp);
    nh->getParam("Kd", Kd);

    e = q_target - q;
    de = qd_target - qd;

    VectorXd q_dd_cmd = qdd_target + Kp * e + Kd * de;
    if (!isfinite(q_dd_cmd.norm()))
        q_dd_cmd.setZero();

//    VectorXd w_ext = L.transpose() * force;

    torques = M.block(6,6,number_of_dofs,number_of_dofs) * q_dd_cmd + CG;// + w_ext

}

void Robot::forwardKinematics(double dt) {
    const iDynTree::Model & model = kinDynComp.model();
    switch (controller) {
        case 0:
            qdd = M.block(6,6,number_of_dofs,number_of_dofs).inverse() * (-L.transpose() * cable_forces + CG);
            qd += qdd * dt;
            q += qd * dt;
            ROS_INFO_STREAM_THROTTLE(1, "torques from tendons:" << endl << (L.transpose() * cable_forces).transpose());
            break;
        case 1:
            qdd = M.block(6,6,number_of_dofs,number_of_dofs).inverse() * (torques + CG);
            qd += qdd * dt;
            q += qd * dt;
            break;
        case 2:
            qd = EigenExtension::Pinv(L) * l_dot;
            q += qd * dt;
            break;
    }

    iDynTree::Transform world_H_base_;
    iDynTree::VectorDynSize jointPos, jointVel;
    jointPos.resize(number_of_dofs);
    jointVel.resize(number_of_dofs);
    iDynTree::Twist baseVel_;
    iDynTree::Vector3 gravity_;
    iDynTree::fromEigen(world_H_base_,world_H_base);
    iDynTree::toEigen(jointPos) = q;
    iDynTree::fromEigen(baseVel_,baseVel);
    toEigen(jointVel) = qd;
    toEigen(gravity_)  = gravity;

    kinDynComp.setRobotState(world_H_base_,jointPos,baseVel_,jointVel,gravity_);
    kinDynComp.generalizedBiasForces(bias);
    kinDynComp.getFreeFloatingMassMatrix(Mass);
    CG = toEigen(bias.jointTorques());
    M = toEigen(Mass);

    ROS_INFO_STREAM_THROTTLE(1,"torques " << torques.transpose());
    ROS_INFO_STREAM_THROTTLE(1,"M " << M.format(fmt));
    ROS_INFO_STREAM_THROTTLE(1,"C+G " << CG.transpose());
    ROS_INFO_STREAM_THROTTLE(1,"qdd " << qdd.transpose());
    ROS_INFO_STREAM_THROTTLE(1,"qd " << qd.transpose());
    ROS_INFO_STREAM_THROTTLE(1,"q " << q.transpose());
    moveit_msgs::DisplayRobotState msg;
    sensor_msgs::JointState msg2;
    static int id = 0;
    msg.state.joint_state.header.frame_id = "world";
    msg.state.joint_state.header.seq = id++;
    msg.state.joint_state.header.stamp = ros::Time::now();
    int j = 0;

    for (auto joint:joint_names) {
        if (((joint_angle_mask >> j) & 0x1) == 0) {
            moveit_msgs::ObjectColor color;
            color.color.a = 1;
            color.color.r = 1;
            color.color.g = 1;
            color.color.b = 1;
            msg.highlight_links.push_back(color);

            msg.state.joint_state.name.push_back(joint);
            msg.state.joint_state.position.push_back(q[j]);
            msg.state.joint_state.velocity.push_back(qd[j]);
            msg.state.joint_state.effort.push_back(qdd[j]);
        }
        j++;
    }
    msg2 = msg.state.joint_state;
    robot_state_pub.publish(msg);
    joint_state_pub.publish(msg2);

//    int i = 1;
//    for (auto joint:joint_names) {
//        tf::Transform trans;
//        Affine3d aff;
//        aff.matrix() = toEigen(model.get);
//        tf::transformEigenToTF(aff, trans);
//        tf_broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "world", joint.c_str()));
//        i++;
//    }

    for (auto link:link_names) {
        tf::Transform trans;
        Affine3d aff;
        aff.matrix() = world_to_link_transform[link].inverse();
        tf::transformEigenToTF(aff, trans);
        tf_broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "world", link.c_str()));
    }

    message_counter = 6666;
    publishTendons();
    publishForces();
}

void Robot::update_V() {
    static int counter = 0;
    V.setZero(number_of_cables, 6 * number_of_links);
    for (int muscle_index = 0; muscle_index < cables.size(); muscle_index++) {
        for (auto &segment:segments[muscle_index]) {
            if (segment.first->link_name != segment.second->link_name) { // ignore redundant cables
                Vector4d temp_vec;
                // V term associated with segment translation
                Vector4d v0(segment.first->global_coordinates[0], segment.first->global_coordinates[1],
                            segment.first->global_coordinates[2], 1);
                Vector4d v1(segment.second->global_coordinates[0], segment.second->global_coordinates[1],
                            segment.second->global_coordinates[2], 1);
                temp_vec = v1 - v0;
                Vector3d segmentVector(temp_vec(0), temp_vec(1), temp_vec(2));
                segmentVector.normalize();

                int k = link_index[segment.first->link_name];
                if (k > 0) {
                    // convert to link coordinate frame
                    Matrix4d transformMatrix = world_to_link_transform[link_names[k]];
                    Matrix3d rotate_to_link_frame = transformMatrix.block(0, 0, 3, 3);

                    Vector3d segmentVector_k = rotate_to_link_frame * segmentVector;

                    // Total V term in translations
                    Vector3d V_ijk_T = Vector3d(segmentVector_k(0), segmentVector_k(1), segmentVector_k(2));

                    Vector3d temp_vec2 = segment.first->local_coordinates;

                    Vector3d V_itk_T = temp_vec2.cross(V_ijk_T);

                    V.block(muscle_index, 6 * k, 1, 3) = V_ijk_T.transpose();
                    V.block(muscle_index, 6 * k + 3, 1, 3) = V_itk_T.transpose();
                    if (log && (counter % 100 == 0)) {
                        log_file << "-------------" << cables[muscle_index].name << "\tk = " << k
                                 << " startpoint" << endl;
                        log_file << "v0 " << v0.transpose() << endl;
                        log_file << "v1 " << v1.transpose() << endl;
                        log_file << "segmentVector " << segmentVector.transpose() << endl;
                        log_file << "R_0k " << rotate_to_link_frame << endl;
                        log_file << "segmentVector_k " << segmentVector_k.transpose() << endl;
                        log_file << "V_ijk_T " << V_ijk_T.transpose() << endl;
                        log_file << "V_itk_T " << V_itk_T.transpose() << endl;
                        log_file << "V " << V << endl;
                    }
                }

                k = link_index[segment.second->link_name];
                if (k > 0) {
                    // convert to link coordinate frame
                    Matrix4d transformMatrix = world_to_link_transform[link_names[k]];
                    Matrix3d rotate_to_link_frame = transformMatrix.block(0, 0, 3, 3);

                    Vector3d segmentVector_k = rotate_to_link_frame * segmentVector;

                    // Total V term in translations
                    Vector3d V_ijk_T = Vector3d(segmentVector_k(0), segmentVector_k(1), segmentVector_k(2));

                    Vector3d temp_vec2 = segment.second->local_coordinates;

                    Vector3d V_itk_T = temp_vec2.cross(V_ijk_T);

                    V.block(muscle_index, 6 * k, 1, 3) = V.block(muscle_index, 6 * k, 1, 3) + V_ijk_T.transpose();
                    V.block(muscle_index, 6 * k + 3, 1, 3) =
                            V.block(muscle_index, 6 * k + 3, 1, 3) + V_itk_T.transpose();
                    if (log && (counter % 100 == 0)) {
                        log_file << "-------------" << cables[muscle_index].name << "\tk = " << k
                                 << " endpoint" << endl;
                        log_file << "v0 " << v0.transpose() << endl;
                        log_file << "v1 " << v1.transpose() << endl;
                        log_file << "segmentVector " << segmentVector.transpose() << endl;
                        log_file << "R_0k " << rotate_to_link_frame << endl;
                        log_file << "segmentVector_k " << segmentVector_k.transpose() << endl;
                        log_file << "V_ijk_T " << V_ijk_T.transpose() << endl;
                        log_file << "V_itk_T " << V_itk_T.transpose() << endl;
                        log_file << "V " << V << endl;
                    }
                }
            }
        }
    }
    counter++;
}

void Robot::update_S() {
    S.setZero(6 * number_of_links, number_of_dofs);
    int k = 1, j = 0;
    for (auto &joint:joint_names) {
        S.block(6 * k, k - 1, 6, 1) = joint_axis[j];
        k++;
        j++;
    }
    ROS_INFO_STREAM("S_t = " << S.transpose().format(fmt));
}

void Robot::update_P() {
    P.setZero(6 * number_of_links, 6 * number_of_links);
    P.block(0, 0, 6, 6).setIdentity(6, 6);

    Matrix3d R_ka;
    Eigen::Matrix<double, 6, 6> Pak;

    const iDynTree::Model & model = kinDynComp.model();

    static int counter = 0;
    for (int k = 1; k < number_of_links; k++) {
        Matrix4d transformMatrix_k = world_to_link_transform[link_names[k]];
        Matrix3d R_k0 = transformMatrix_k.block(0, 0, 3, 3);

        for (int a = 1; a <= k; a++) {
            Matrix4d transformMatrix_a = world_to_link_transform[link_names[a]];
            Matrix3d R_0a = transformMatrix_a.block(0, 0, 3, 3).transpose();
            R_ka = R_k0 * R_0a;

            Matrix3d R_pe;
            Vector3d r_OP, r_OG;
            r_OP.setZero();

            R_pe = AngleAxisd(q[a - 1], joint_axis[a - 1].block(0, 0, 3, 1));

            // Calculate forward position kinematics
            Eigen::MatrixXd pose = iDynTree::toEigen(model.getFrameTransform(a).asHomogeneousTransform());
            r_OP = R_0a.transpose() * pose.block(0,3,3,1);

            r_OG = R_k0 * transformMatrix_k.inverse().block(0, 3, 3, 1);

            Pak.block(0, 0, 3, 3) = R_ka * R_pe.transpose();
            Pak.block(0, 3, 3, 3) = -R_ka * EigenExtension::SkewSymmetric2(-r_OP + R_ka.transpose() * r_OG);
            Pak.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
            Pak.block(3, 3, 3, 3) = R_ka;
            P.block(6 * k, 6 * a, 6, 6) = Pak;

            if (log && (counter % 100 == 0)) {
                Eigen::IOFormat fmt(4, 0, " ", ";\n", "", "", "[", "]");
                log_file << "---------------------" << link_names[k] << "---" << link_names[a] << endl;
                log_file << "R_0k = " << R_k0.format(fmt) << endl;
                log_file << "R_0a = " << R_0a.format(fmt) << endl;
                log_file << "R_ka = " << R_ka.format(fmt) << endl;
                log_file << "R_pe = " << R_pe.format(fmt) << endl;
                log_file << "r_OP = " << r_OP.format(fmt) << endl;
                log_file << "r_OG = " << r_OG.format(fmt) << endl;
                log_file << "Pak = " << Pak.format(fmt) << endl;
            }
        }
    }

    counter++;
}

bool Robot::getTransform(const char *to, const char *from, Matrix4d &transform) {
    tf::StampedTransform trans;
    try {
        tf_listener.lookupTransform(to, from, ros::Time(0), trans);
    }
    catch (tf::TransformException ex) {
        ROS_WARN_THROTTLE(1, "%s", ex.what());
        return false;
    }

    Eigen::Affine3d trans_;
    tf::transformTFToEigen(trans, trans_);
    transform = trans_.matrix();
    return true;
}

void Robot::publishTendons() {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "tendons_" ;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.003;
    line_strip.color.r = 0;
    line_strip.color.g = 0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.pose.orientation.w = 1.0;
    line_strip.lifetime = ros::Duration(0);
    for (uint muscle = 0; muscle < cables.size(); muscle++) {
        line_strip.points.clear();
        line_strip.id = muscle+9999999;
        for (uint i = 1; i < cables[muscle].viaPoints.size(); i++) {
            geometry_msgs::Point p;
            p.x = cables[muscle].viaPoints[i - 1]->global_coordinates[0];
            p.y = cables[muscle].viaPoints[i - 1]->global_coordinates[1];
            p.z = cables[muscle].viaPoints[i - 1]->global_coordinates[2];
            line_strip.points.push_back(p);
            p.x = cables[muscle].viaPoints[i]->global_coordinates[0];
            p.y = cables[muscle].viaPoints[i]->global_coordinates[1];
            p.z = cables[muscle].viaPoints[i]->global_coordinates[2];
            line_strip.points.push_back(p);
//            if (controller == 2) { // position control
//                Vector3d pos = (cables[muscle].viaPoints[i]->global_coordinates +
//                        cables[muscle].viaPoints[i - 1]->global_coordinates) / 2.0;
//                char str[100];
//                sprintf(str, "%.3lf", motor_pos[muscle]);
//                publishText(pos, str, "world", "tendon_length", message_counter++, COLOR(1, 1, 1, 1), 1, 0.01);
//            }
        }
        visualization_pub.publish(line_strip);
//        for (uint i = 0; i < muscles[muscle].viaPoints.size(); i++) {
//            publishSphere(muscles[muscle].viaPoints[i]->global_coordinates,"world","viaPoints",message_counter++,COLOR(0,1,1,1),0.01,1);
//        }
    }
//    int i = 0;
//    for (auto muscle:cables) {
//        Matrix4d transform = world_to_link_transform[muscle.viaPoints.front()->link_name].inverse();
//        muscle.viaPoints.front()->global_coordinates = transform.block(0, 3, 3, 1) + transform.block(0, 0, 3, 3) * muscle.viaPoints.front()->local_coordinates;
//        char str[20];
//        if(active_motors.end() == active_motors.find(id))
//            sprintf(str,"motor%d",i);
//        else
//            sprintf(str,"motor%d",active_motors[id][i]);
//        publishText(muscle.viaPoints.front()->global_coordinates, str, "world", (end_effektor_name+"_motor_ids").c_str(), message_counter++, COLOR(0, 1, 1, 1), 5,0.01);
//        i++;
//    }
}

void Robot::publishForces() {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.ns = "force_";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration(0);
    arrow.scale.x = 0.005;
    arrow.scale.y = 0.01;
    arrow.scale.z = 0.01;
    arrow.pose.orientation.w = 1;
    arrow.pose.orientation.x = 0;
    arrow.pose.orientation.y = 0;
    arrow.pose.orientation.z = 0;
    arrow.action = visualization_msgs::Marker::ADD;

    for (uint muscle = 0; muscle < cables.size(); muscle++) {
        for (uint i = 1; i < cables[muscle].viaPoints.size(); i++) {
            // actio
            arrow.id = message_counter++;
            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.header.stamp = ros::Time::now();
            arrow.points.clear();
            geometry_msgs::Point p;
            p.x = cables[muscle].viaPoints[i - 1]->global_coordinates[0];
            p.y = cables[muscle].viaPoints[i - 1]->global_coordinates[1];
            p.z = cables[muscle].viaPoints[i - 1]->global_coordinates[2];
            Vector3d dir = cables[muscle].viaPoints[i - 1]->global_coordinates-cables[muscle].viaPoints[i]->global_coordinates;
            dir.normalize();
            arrow.points.push_back(p);
            p.x -= dir[0]*cable_forces[muscle] * 0.001; // show fraction of force
            p.y -= dir[1]*cable_forces[muscle] * 0.001;
            p.z -= dir[2]* cable_forces[muscle] * 0.001;
            arrow.points.push_back(p);
            visualization_pub.publish(arrow);
            // reactio
            arrow.id = message_counter++;
            arrow.color.r = 1.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.header.stamp = ros::Time::now();
            arrow.points.clear();
            p.x = cables[muscle].viaPoints[i]->global_coordinates[0];
            p.y = cables[muscle].viaPoints[i]->global_coordinates[1];
            p.z = cables[muscle].viaPoints[i]->global_coordinates[2];
            arrow.points.push_back(p);
            p.x += dir[0]*cable_forces[muscle] * 0.001; // show fraction of force
            p.y += dir[1]* cable_forces[muscle] * 0.001;
            p.z += dir[2]*cable_forces[muscle] * 0.001;
            arrow.points.push_back(p);
            visualization_pub.publish(arrow);
        }
    }
}

//bool Robot::ForwardKinematicsService(roboy_communication_middleware::ForwardKinematics::Request &req,
//                                     roboy_communication_middleware::ForwardKinematics::Response &res) {
//
//    rl::math::Vector target_q = q;
//    int i = 0;
//    for (auto val:req.angles) {
//        target_q[i] = val;
//        i++;
//    }
//    ROS_INFO_STREAM("Serving ForwardKinematics Service robot config " << target_q.transpose());
//    // Calculate forward position kinematics
//    kinematic->setPosition(target_q); // input value
//    kinematic->forwardPosition(); // modify model
//    const rl::math::Transform &x = kinematic->getOperationalPosition(0);
//
//    ROS_INFO_STREAM("endeffector position: " << x.linear());
//    res.pos.x = x.linear()(0, 0);
//    res.pos.y = x.linear()(1, 0);
//    res.pos.z = x.linear()(2, 0);
//    return true;
//}
//
//bool Robot::InverseKinematicsService(roboy_communication_middleware::InverseKinematics::Request &req,
//                                     roboy_communication_middleware::InverseKinematics::Response &res) {
//    kinematic_ik->setPosition(q);
//    rl::mdl::NloptInverseKinematics ik(kinematic_ik);
//    ik.duration = std::chrono::seconds(5);
//    ik.epsilonTranslation = 0.05;
//    ik.epsilonRotation = 0.05;
//    rl::math::Transform x_new;
//    tf::poseMsgToEigen(req.pose, x_new);
//
//    ik.goals.push_back(::std::make_pair(x_new, 0)); // goal frame in world coordinates for first TCP
//
//    bool result = ik.solve();
//    if (result) {
//        rl::math::Vector solution = kinematic_ik->getPosition();
//        ROS_INFO_STREAM("ik solution " << solution.transpose());
//        for (int i = 0; i < solution.rows(); i++) {
//            res.angles.push_back(solution[i]);
//        }
//        kinematic_ik->setPosition(q);
//        return true;
//    } else {
//        ROS_ERROR("unable to solve ik for target %.3lf %.3lf %.3lf", req.pose.position.x, req.pose.position.y, req.pose.position.z);
//        kinematic_ik->setPosition(q);
//        return false;
//    }
//}


bool Robot::parseViapoints(const string &viapoints_file_path, vector<Cable> &cables) {
    // initialize TiXmlDocument doc from file
    TiXmlDocument doc(viapoints_file_path.c_str());
    if (!doc.LoadFile()) {
        ROS_FATAL("Can't parse via points file %s.", viapoints_file_path.c_str());
        return false;
    }

    TiXmlElement *root = doc.RootElement();

    // Constructs the myoMuscles by parsing custom xml.
    TiXmlElement *myoMuscle_it = NULL;
    for (myoMuscle_it = root->FirstChildElement("myoMuscle"); myoMuscle_it;
         myoMuscle_it = myoMuscle_it->NextSiblingElement("myoMuscle")) {
        Cable cable;
        if (myoMuscle_it->Attribute("name")) {
            cable.name = myoMuscle_it->Attribute("name");
            // myoMuscle joint acting on
            TiXmlElement *link_child_it = NULL;
            for (link_child_it = myoMuscle_it->FirstChildElement("link"); link_child_it;
                 link_child_it = link_child_it->NextSiblingElement("link")) {
                string link_name = link_child_it->Attribute("name");
                if (!link_name.empty()) {
                    TiXmlElement *viaPoint_child_it = NULL;
                    for (viaPoint_child_it = link_child_it->FirstChildElement("viaPoint"); viaPoint_child_it;
                         viaPoint_child_it = viaPoint_child_it->NextSiblingElement("viaPoint")) {
                        float x, y, z;
                        if (sscanf(viaPoint_child_it->GetText(), "%f %f %f", &x, &y, &z) != 3) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading [via point] (x y z)");
                            return false;
                        }
                        Vector3d local_coordinates(x, y, z);
                        cable.viaPoints.push_back(ViaPointPtr (new ViaPoint(link_name, local_coordinates)));
                    }
                    if (cable.viaPoints.empty()) {
                        ROS_ERROR_STREAM_NAMED("parser", "No viaPoint element found in myoMuscle '"
                                << cable.name << "' link element.");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No link name attribute specified for myoMuscle'"
                            << cable.name << "'.");
                    continue;
                }
            }
            ROS_INFO("%ld viaPoints for myoMuscle %s", cable.viaPoints.size(), cable.name.c_str());
        }
        cables.push_back(cable);
    }
    return true;
}

//#include "kindyn/model.hpp"
//
//int CASPR::instance = 0;
//
//CASPR::CASPR(string &root_link, string &end_link, vector<MuscInfo> &myoMuscleInfo) :
//        end_effektor_name(end_link) {
//    if (!ros::isInitialized()) {
//        int argc = 0;
//        char **argv = NULL;
//        ros::init(argc, argv, "CASPR_" + end_link, ros::init_options::NoSigintHandler);
//    }
//    nh = ros::NodeHandlePtr(new ros::NodeHandle);
//
//    log_file.open(end_link + ".log");
//    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");
//
//    string urdf_path;
//    if (!nh->hasParam("urdf_path")) {
//        ROS_FATAL("could not find urdf_path");
//        return;
//    }
//
//    nh->getParam("urdf_path", urdf_path);
//    rl::mdl::UrdfFactory factory;
//    model = new rl::mdl::Dynamic;
//    model_ik = new rl::mdl::Dynamic;
//
//    nh->getParam(end_effektor_name + "/joint_angle_mask", joint_angle_mask);
//
//    factory.load(urdf_path, root_link, end_link, model);
//    factory.load(urdf_path, root_link, end_link, model_ik);
//
//    ROS_INFO("Model consist of %ld endeffectors", model->getOperationalDof());
//
//    kinematic = dynamic_cast<rl::mdl::Kinematic *>(model);
//    kinematic_ik = dynamic_cast<rl::mdl::Kinematic *>(model_ik);
//    dynamic = dynamic_cast<rl::mdl::Dynamic *>(model);
//
//    root_link_name = kinematic->getBody(0)->getName();
//
//    number_of_cables = 0;
//    number_of_links = kinematic->getBodies();
//    ROS_INFO("number of links in kinematic chain %d", number_of_links);
//    // get the link names of the kinematic chain
//    int j = 0;
//    for (int link = 0; link < number_of_links; link++) {
//        string link_name = kinematic->getBody(link)->getName();
//        link_names.push_back(link_name);
//        ROS_INFO_STREAM(link_name);
//        links.push_back(kinematic->getBody(link));
//        link_index[link_name] = link;
//    }
//    number_of_dofs = kinematic->getDof();
//    for (int joint = 0; joint < number_of_dofs; joint++) {
//        joints.push_back(kinematic->getJoint(joint));
//        joint_axis.push_back(kinematic->getJoint(joint)->S);
//        joint_names.push_back(kinematic->getJoint(joint)->getName());
//        joint_index[kinematic->getJoint(joint)->getName()] = joint;
//        if (joint_axis.back()(0) == 0 && joint_axis.back()(1) == 0 && joint_axis.back()(2) == 0)
//            joint_types.push_back(PRISMATIC);
//        else
//            joint_types.push_back(REVOLUTE);
//        j++;
//    }
//
//    // we need to check every myoMuscle if it belongs to our kinematic chain
//    ROS_INFO("checking %ld muslces if they belong to kinematic chain", myoMuscleInfo.size());
//    vector<bool> belongs_to_kinematic_chain(myoMuscleInfo.size(), false);
//    int muscle_index = 0;
//    for (auto muscle:myoMuscleInfo) {
//        bool root_link_found = false;
//        string root_link;
//        for (auto link:link_names) {
//            for (auto vp:muscle.viaPoints) {
//                if (vp.link_name.compare(link) == 0) {
//                    if (!root_link_found) {
//                        root_link = link;
//                        root_link_found = true;
//                    } else {
//                        if (root_link.compare(link) != 0) {
//                            ROS_INFO("%s part of kinematic chain %s -> %s found", muscle.name.c_str(),
//                                     root_link.c_str(), link.c_str());
//                            belongs_to_kinematic_chain[muscle_index] = true;
//                            break;
//                        }
//                    }
//                } else {
//                    ROS_DEBUG_STREAM("vp on " << vp.link_name << " does not belong to kinematic chain");
//                }
//            }
//            if (belongs_to_kinematic_chain[muscle_index]) {
//                Muscle m;
//                m.name = myoMuscleInfo[muscle_index].name;
//                m.viaPoints.resize(myoMuscleInfo[muscle_index].viaPoints.size());
//                int j = 0;
//                for (auto v:myoMuscleInfo[muscle_index].viaPoints) {
//                    Vector3d local(v.local_coordinates.x, v.local_coordinates.y, v.local_coordinates.z);
//                    m.viaPoints[j].reset(new ViaPoint(v.link_name, local));
//                    j++;
//                }
//                muscles.push_back(m);
//                break;
//            }
//        }
//        muscle_index++;
//    }
//
//    // because we have 1-DOF joints only, the total number of dofs equals the number of joints
//    number_of_cables = muscles.size();
//
//    ROS_INFO("\nnumber_of_cables:\t%d"
//                     "\nnumber_of_dofs:  \t%d"
//                     "\nnumber_of_links: \t%d", number_of_cables, number_of_dofs, number_of_links);
//
//    init();
//
//    time.reset(new MeasureExecutionTime(end_effektor_name));
//
//    instance++;
//
//    spinner.reset(new ros::AsyncSpinner(0));
//    spinner->start();
//
//    nh->getParam("controller", controller);
//    nh->getParam(end_effektor_name + "/fpga_id", id);
//
//    motorcommand_pub = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand", 1);
//    motorstatus_sub = nh->subscribe("/roboy/middleware/MotorStatus",  1, &CASPR::MotorStatus, this);
//    if(id==SHOULDER_LEFT) {
//        elbow_joint_pub = nh->advertise<std_msgs::Float32>("/roboy/middleware/elbow_left/JointAngle", 1);
//        wrist_joint_pub = nh->advertise<std_msgs::Float32>("/roboy/middleware/wrist_left/JointAngle", 1);
//    }else if(id==SHOULDER_RIGHT){
//        elbow_joint_pub = nh->advertise<std_msgs::Float32>("/roboy/middleware/elbow_right/JointAngle", 1);
//        wrist_joint_pub = nh->advertise<std_msgs::Float32>("/roboy/middleware/wrist_right/JointAngle", 1);
//    }
//
//
////    M_pub = nh->advertise<std_msgs::Float64MultiArray>("/M", 1);
////    C_pub = nh->advertise<std_msgs::Float64MultiArray>("/C", 1);
////    G_pub = nh->advertise<std_msgs::Float64MultiArray>("/G", 1);
////    L_pub = nh->advertise<std_msgs::Float64MultiArray>("/L", 1);
////    V_pub = nh->advertise<std_msgs::Float64MultiArray>("/V", 1);
////    S_pub = nh->advertise<std_msgs::Float64MultiArray>("/S", 1);
////    P_pub = nh->advertise<std_msgs::Float64MultiArray>("/P", 1);
//    nh->getParam("simulate", simulate);
//    if(simulate)
//        robot_state_sub = nh->subscribe("/display_robot_state", 1, &CASPR::updateRobotState, this);
//    else{
//        robot_state_pub = nh->advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1);
//        joint_state_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
//    }
//
//    fk_srv = nh->advertiseService("/CASPR/" + end_effektor_name + "/ForwardKinematics",
//                                  &CASPR::ForwardKinematicsService,
//                                  this);
//    ik_srv = nh->advertiseService("/CASPR/" + end_effektor_name + "/InverseKinematics",
//                                  &CASPR::InverseKinematicsService,
//                                  this);
//    torque_control_srv = nh->serviceClient<roboy_communication_middleware::TorqueControl>(
//            "/roboy/middleware/TorqueControl");
//
//    ROS_INFO("CASPR ready");
//}
//
//void CASPR::update(double period) {
//    if (!simulate) {
//        forwardKinematics(period);
//    }else{
//        if(updated) {
//            updated = false;
//        }else
//            return;
//    }
//
//    int i = 0;
//    for (auto &link_name:link_names) {
//        Matrix4d pose = kinematic->bodies[i]->t.matrix();
//        pose.block(0, 3, 3, 1) += pose.block(0, 0, 3, 3) * kinematic->bodies[i]->cm;
//        world_to_link_transform[link_name] = pose.inverse();
//        i++;
//    }
//
//    for (auto muscle:muscles) {
//        for (auto vp:muscle.viaPoints) {
//            Matrix4d transform = world_to_link_transform[vp->link_name].inverse();
//            vp->global_coordinates = transform.block(0, 3, 3, 1) + transform.block(0, 0, 3, 3) * vp->local_coordinates;
//        }
//    }
//
//    update_V();
//    update_P();
//
//    W = P * S;
//    L = V * W;
//    L_t = -L.transpose();
//
////    std_msgs::Float64MultiArray msg[7];
////
////    grid_map::matrixEigenCopyToMultiArrayMessage(M, msg[0]);
////    grid_map::matrixEigenCopyToMultiArrayMessage(G, msg[1]);
////    grid_map::matrixEigenCopyToMultiArrayMessage(C, msg[2]);
////    grid_map::matrixEigenCopyToMultiArrayMessage(L, msg[3]);
////    grid_map::matrixEigenCopyToMultiArrayMessage(V, msg[4]);
////    grid_map::matrixEigenCopyToMultiArrayMessage(S, msg[5]);
////    grid_map::matrixEigenCopyToMultiArrayMessage(P, msg[6]);
////
////    M_pub.publish(msg[0]);
////    C_pub.publish(msg[2]);
////    G_pub.publish(msg[1]);
////    L_pub.publish(msg[3]);
////    V_pub.publish(msg[4]);
////    S_pub.publish(msg[5]);
////    P_pub.publish(msg[6]);
//}
//
//void CASPR::updateController() {
//    static int counter = 0;
//
//    e = q_target - q;
//    de = qd_target - qd;
//
//    VectorXd q_dd_cmd = qdd_target + Kp * e + Kd * de;
//    if (!isfinite(q_dd_cmd.norm()))
//        q_dd_cmd.setZero();
//
////    VectorXd w_ext = L.transpose() * force;
//
//    torques = M * q_dd_cmd + C + G;// + w_ext
//
//    // for any masked joint we set the target torques to zero
//    for (int i = 0; i < number_of_dofs; i++) {
//        if (((joint_angle_mask >> i) & 0x1) == 1) {
//            torques[i] = 0;
//        }
//    }
//
//
////    if (log && (counter % 100 == 0)) {
////        log_file << "S = " << S.format(fmt) << endl;
////        log_file << "P = " << P.format(fmt) << endl;
////        log_file << "V = " << V.format(fmt) << endl;
////        log_file << "W = " << W.format(fmt) << endl;
////        log_file << "L = " << L.format(fmt) << endl;
////        log_file << "L_t = " << L_t.format(fmt) << endl;
////    }
//
//    nh->getParam("controller", controller);
//
//    static int controller_prev = 3;
//    if (controller != controller_prev) {
//        clearAll();
//    }
//    controller_prev = controller;
//
////    ROS_INFO_STREAM("\nq_target " << q_target.transpose()
////                                  << "\ne  " << e.transpose()
////                                  << "\nde " << de.transpose()
////                                  << "\nq_dd_cmd" << q_dd_cmd.transpose()
////                                  << "\ntorques " << torques.transpose()
////                                  << "\nl_dot "
////                                  << l_dot.transpose());
//
//    switch (controller) {
//        case 0: {
//            ROS_WARN_THROTTLE(5, "caspr controller active");
//            VectorXd f_min, f_max;
//            f_min.resize(number_of_dofs);
//            f_max.resize(number_of_dofs);
//
//            double min_force, max_force;
//            nh->getParam("min_force", min_force);
//            nh->getParam("max_force", max_force);
//
//            f_min = VectorXd::Ones(number_of_cables);
//            f_max = VectorXd::Ones(number_of_cables);
//
//            f_min = min_force * f_min;
//            f_max = max_force * f_max;
//
//            cable_forces = resolve_function(L_t, torques, f_min, f_max);
////            torques.setZero();
//            break;
//        }
//        case 1: {
//            ROS_WARN_THROTTLE(5, "torque controller active");
//            if(simulate) {
//                int i = 0;
//                roboy_communication_middleware::TorqueControl msg;
//                msg.request.joint_names = joint_names;
//                for (auto &joint_name:joint_names) {
//                    if (abs(torques[i]) < 10000000 && isfinite(torques[i]))
//                        msg.request.torque.push_back(torques[i]);
//                    else
//                        msg.request.torque.push_back(0);
//                    i++;
//                }
//                if (!torque_control_srv.call(msg))
//                    ROS_WARN_ONCE("could not set torque, service unavailable");
//            }
//            break;
//        }
//        case 2: { // position control
//            ROS_WARN_THROTTLE(5, "position controller active");
//            l_dot = L * (Kd * (qd_target - qd) + Kp * (q_target - q));
////            torques.setZero();
//            break;
//        }
//        default: { // deactivate
//            torques.setZero();
//            ROS_WARN_THROTTLE(5, "controller not active");
//            break;
//        }
//    }
//    //<< "\nw_ext " << w_ext.transpose()
//}
//
//void CASPR::forwardKinematics(double dt) {
//    switch (controller) {
//        case 0:
//            qdd = M.inverse() * (-L.transpose() * cable_forces + C + G);
////            qd += qdd * dt;
////            q += qd * dt;
//            for(int j = 0; j<number_of_dofs;j++) {
//                boost::numeric::odeint::integrate(
//                        [this,j](const CASPR::state_type &x, CASPR::state_type &dxdt, double t) {
//                            dxdt[1] = qdd[j];
//                            dxdt[0] = x[1];
//                        }, joint_state[j], integration_time, integration_time + dt, dt);
//                if(((joint_angle_mask >> j) & 0x1)==0) {
//                    qd[j] = joint_state[j][1];
//                    q[j] = joint_state[j][0];
//                }
//            }
//            kinematic->clip(q);
////            ROS_INFO_STREAM_THROTTLE(1, "torques from tendons:" << endl << (L.transpose() * cable_forces).transpose());
//            break;
//        case 1:
//            qdd = M.inverse() * (torques + C + G);
////            qd += qdd * dt;
////            q += qd * dt;
//            for(int j = 0; j<number_of_dofs;j++) {
//                boost::numeric::odeint::integrate(
//                        [this,j](const CASPR::state_type &x, CASPR::state_type &dxdt, double t) {
//                            dxdt[1] = qdd[j];
//                            dxdt[0] = x[1];
//                        }, joint_state[j], integration_time, integration_time + dt, dt);
//                if(((joint_angle_mask >> j) & 0x1)==0) {
//                    qd[j] = joint_state[j][1];
//                    q[j] = joint_state[j][0];
//                }
//            }
//            kinematic->clip(q);
//            break;
//        case 2:
//            qd = EigenExtension::Pinv(L) * l_dot;
////            q += qd * dt;
//            for(int j = 0; j<number_of_dofs;j++) {
//                boost::numeric::odeint::integrate(
//                        [this,j](const CASPR::state_type &x, CASPR::state_type &dxdt, double t) {
//                            dxdt[1] = 0;
//                            dxdt[0] = qd[j];
//                        }, joint_state[j], integration_time, integration_time + dt, dt);
//                if(((joint_angle_mask >> j) & 0x1)==0) {
//                    q[j] = joint_state[j][0];
//                }
//                if(q[j]>=kinematic->getJoint(j)->max[0]) {
//                    qd[j] = 0;
//                    q[j] = kinematic->getJoint(j)->max[0];
//                }else if(q[j]<=kinematic->getJoint(j)->min[0]) {
//                    qd[j] = 0;
//                    q[j] = kinematic->getJoint(j)->min[0];
//                }
//            }
//            for (int i = 0; i < number_of_cables; i++) {
//                motor_vel[i] = l_dot[i];
////                motor_pos[i] += l_dot[i] * dt;
//                boost::numeric::odeint::integrate(
//                        [this,i](const CASPR::state_type &x, CASPR::state_type &dxdt, double t) {
//                            dxdt[1] = 0;
//                            dxdt[0] = motor_vel[i];
//                        }, motor_state[i], integration_time, integration_time + dt, dt);
//                motor_pos[i] = motor_state[i][0];
//            }
//            break;
//    }
//
//    // apply to model
//    kinematic->setPosition(q);
//    kinematic->forwardPosition();
//    kinematic->setVelocity(qd);
//    kinematic->forwardVelocity();
//    kinematic->setAcceleration(qdd);
//    kinematic->forwardAcceleration();
//
//    dynamic->setPosition(q);
//    dynamic->forwardPosition();
//    dynamic->setVelocity(qd);
//    dynamic->forwardVelocity();
//    dynamic->setAcceleration(qdd);
//    dynamic->forwardAcceleration();
//
//    dynamic->calculateMassMatrix(); // modify model
//    dynamic->calculateCentrifugalCoriolis();
//    dynamic->calculateGravity();
//    M = dynamic->getMassMatrix(); // output value
//    C = dynamic->getCentrifugalCoriolis();
//    G = dynamic->getGravity();
//
//    integration_time+=dt;
//
////    ROS_INFO_STREAM("qdd " << qdd.transpose());
////    ROS_INFO_STREAM("qd " << qd.transpose());
////    ROS_INFO_STREAM("q " << q.transpose());
//    moveit_msgs::DisplayRobotState msg;
//    sensor_msgs::JointState msg2;
//    static int id = 0;
//    msg.state.joint_state.header.frame_id = "world";
//    msg.state.joint_state.header.seq = id++;
//    msg.state.joint_state.header.stamp = ros::Time::now();
//    int j = 0;
//
//    for (auto joint:joint_names) {
//        if (((joint_angle_mask >> j) & 0x1) == 0) {
//            moveit_msgs::ObjectColor color;
//            color.color.a = 1;
//            color.color.r = 1;
//            color.color.g = 1;
//            color.color.b = 1;
//            msg.highlight_links.push_back(color);
//            joint_pos[j] = q[j];
//            joint_vel[j] = qd[j];
//            joint_acc[j] = qdd[j];
//
//            msg.state.joint_state.name.push_back(joint);
//            msg.state.joint_state.position.push_back(joint_pos[j]);
//            msg.state.joint_state.velocity.push_back(joint_vel[j]);
//            msg.state.joint_state.effort.push_back(joint_acc[j]);
//        }
//        j++;
//    }
//    msg2 = msg.state.joint_state;
//    robot_state_pub.publish(msg);
//    joint_state_pub.publish(msg2);
//
//    int i = 1;
//    for (auto joint:joint_names) {
//        tf::Transform trans;
//        Affine3d aff;
//        aff.matrix() = kinematic->bodies[i]->t.matrix();
//        tf::transformEigenToTF(aff, trans);
//        tf_broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "world", joint.c_str()));
//        i++;
//    }
//
//    for (auto link:link_names) {
//        tf::Transform trans;
//        Affine3d aff;
//        aff.matrix() = world_to_link_transform[link].inverse();
//        tf::transformEigenToTF(aff, trans);
//        tf_broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "world", link.c_str()));
//    }
//
//    message_counter = 6666;
//    publishTendons();
//    publishForces();
//}
//
//void CASPR::init() {
//    qp_solver = SQProblem(number_of_cables, number_of_dofs);
//
//    H = new real_t[number_of_cables * number_of_cables];
//    g = new real_t[number_of_cables];
//    A = new real_t[number_of_cables * number_of_cables];
//    lb = new real_t[number_of_cables];
//    ub = new real_t[number_of_cables];
//    b = new real_t[number_of_dofs];
//    FOpt = new real_t[number_of_cables];
//    cable_forces.resize(number_of_cables);
//    l_dot.resize(number_of_cables);
//    cable_forces.setZero();
//    l_dot.setZero();
//
//    motor_pos.resize(number_of_cables, 0);
//    motor_pos_real.resize(number_of_cables, 0);
//    motor_pos_real_offset.resize(number_of_cables, 0);
//    motor_vel.resize(number_of_cables, 0);
//    motor_vel_real.resize(number_of_cables, 0);
//    displacement_real.resize(number_of_cables, 0);
//
//    torques.resize(number_of_dofs);
//    torques.setZero();
//
//    joint_pos.resize(number_of_dofs);
//    joint_vel.resize(number_of_dofs);
//    joint_acc.resize(number_of_dofs);
//    joint_vel_prev.resize(number_of_dofs);
//
//    q.resize(number_of_dofs);
//    C.resize(number_of_dofs);
//    G.resize(number_of_dofs);
//    qd.resize(number_of_dofs);
//    qdd.resize(number_of_dofs);
//
//    e.resize(number_of_dofs);
//    de.resize(number_of_dofs);
//    dde.resize(number_of_dofs);
//
//    q_target.resize(number_of_dofs);
//    qd_target.resize(number_of_dofs);
//    qdd_target.resize(number_of_dofs);
//
//    joint_state.resize(number_of_dofs);
//    motor_state.resize(number_of_cables);
//
//    q_target.setZero();
//    qd_target.setZero();
//    qdd_target.setZero();
//
//    for (int i = 0; i < number_of_dofs; i++) {
//        q(i) = 0;
//        C(i) = 0;
//        G(i) = 0;
//        qd(i) = 0;
//        qdd(i) = 0;
//        joint_pos[i] = 0;
//        joint_vel[i] = 0;
//        joint_vel_prev[i] = 0;
//        joint_acc[i] = 0;
//        e[i] = 0;
//        de[i] = 0;
//        dde[i] = 0;
//        joint_state[i][0] = 0;
//        joint_state[i][1] = 0;
//        motor_state[i][0] = 0;
//        motor_state[i][1] = 0;
//    }
//
//    x.resize(6 * number_of_links);
//    x_dot.resize(6 * number_of_links);
//    x_dot_relative.resize(6 * number_of_links);
//
//    M.resize(number_of_dofs, number_of_dofs);
//
//    V.resize(number_of_cables, 6 * number_of_links);
//    V.setZero(number_of_cables, 6 * number_of_links);
//    P.resize(6 * number_of_links, 6 * number_of_links);
//    P.setZero(6 * number_of_links, 6 * number_of_links);
//    S.resize(6 * number_of_links, number_of_dofs);
//    S.setZero(6 * number_of_links, number_of_dofs);
//
//    segments.resize(number_of_cables);
//
//    int muscle_index = 0;
//    for (auto muscle:muscles) {
//        for (int i = 1; i < muscle.viaPoints.size(); i++) {
//            pair<ViaPointPtr, ViaPointPtr> segment(muscle.viaPoints[i - 1], muscle.viaPoints[i]);
//            segments[muscle_index].push_back(segment);
//        }
//        muscle_index++;
//    }
//
//    kinematic->setPosition(q);
//    kinematic->forwardPosition();
//    kinematic->setVelocity(qd);
//    kinematic->forwardVelocity();
//    kinematic->setAcceleration(qdd);
//    kinematic->forwardAcceleration();
//
//    dynamic->setPosition(q);
//    dynamic->forwardPosition();
//    dynamic->setVelocity(qd);
//    dynamic->forwardVelocity();
//    dynamic->setAcceleration(qdd);
//    dynamic->forwardAcceleration();
//
//    dynamic->calculateMassMatrix(); // modify model
//    dynamic->calculateCentrifugalCoriolis();
//    dynamic->calculateGravity();
//    M = dynamic->getMassMatrix(); // output value
//    C = dynamic->getCentrifugalCoriolis();
//    G = dynamic->getGravity();
//
//    int i = 0;
//    for (auto &link_name:link_names) {
//        Matrix4d pose = kinematic->bodies[i]->t.matrix();
//        pose.block(0, 3, 3, 1) += pose.block(0, 0, 3, 3) * kinematic->bodies[i]->cm;
//        world_to_link_transform[link_name] = pose.inverse();
//        i++;
//    }
//
//    for (auto muscle:muscles) {
//        for (auto vp:muscle.viaPoints) {
//            Matrix4d transform = world_to_link_transform[vp->link_name].inverse();
//            vp->global_coordinates = transform.block(0, 3, 3, 1) + transform.block(0, 0, 3, 3) * vp->local_coordinates;
//        }
//    }
//
//    update_S();
//    update_V();
//    update_P();
//
//    W = P * S;
//    L = V * W;
//    L_t = -L.transpose();
//}
//
//void CASPR::updateRobotState(const moveit_msgs::DisplayRobotState::ConstPtr &msg) {
//    static int counter = 0;
//    int j = 0;
//    for (auto s:msg->state.joint_state.name) {
//        auto it = find(joint_names.begin(), joint_names.end(), msg->state.joint_state.name[j]);
//        if (it == joint_names.end()) {
//            j++;
//            continue;
//        }
//        int index = joint_index[msg->state.joint_state.name[j]];
//        joint_pos[index] = msg->state.joint_state.position[j];
//        joint_vel[index] = msg->state.joint_state.velocity[j];
//        joint_acc[index] = msg->state.joint_state.effort[j];
//
//        q(index) = joint_pos[index];
//        qd(index) = joint_vel[index];
//        qdd(index) = joint_acc[index];
//
//        if (log && (counter % 100 == 0)) {
//            log_file << msg->state.joint_state.name[j] << " " << q(index) << endl;
//        }
//
//        j++;
//    }
//    kinematic->setPosition(q);
//    kinematic->forwardPosition();
//    kinematic->setVelocity(qd);
//    kinematic->forwardVelocity();
//    kinematic->setAcceleration(qdd);
//    kinematic->forwardAcceleration();
//
//    dynamic->setPosition(q);
//    dynamic->forwardPosition();
//    dynamic->setVelocity(qd);
//    dynamic->forwardVelocity();
//    dynamic->setAcceleration(qdd);
//    dynamic->forwardAcceleration();
//
//    dynamic->calculateMassMatrix(); // modify model
//    dynamic->calculateCentrifugalCoriolis();
//    dynamic->calculateGravity();
//    M = dynamic->getMassMatrix(); // output value
//    C = dynamic->getCentrifugalCoriolis();
//    G = dynamic->getGravity();
//
//    counter++;
//    updated = true;
//}
//
//void CASPR::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
//    if (msg->id != id) // not for me
//        return;
//    ROS_INFO_THROTTLE(10, "receiving motor status");
//    for (uint motor = 0; motor < msg->position.size(); motor++) {
//        switch(motor_type[id][motor]){
//            case MYOMUSCLE500N:
//                motor_pos_real[motor] = myoMuscleMeterPerEncoderTick(msg->position[motor]);
//                motor_vel_real[motor] = myoMuscleMeterPerEncoderTick(msg->velocity[motor]);
//                displacement_real[motor] = msg->displacement[motor] * 0.00001; // 0.1mm per tick
//                break;
//            case MYOBRICK100N:
//                motor_pos_real[motor] = myoBrick100NEncoderTicksPerMeter(msg->position[motor]);
//                motor_vel_real[motor] = myoBrick100NEncoderTicksPerMeter(msg->velocity[motor]);
//                displacement_real[motor] = (msg->angle[motor]/4096.0)*2.0*M_PI*0.003; // rad to meter
//                break;
//            case MYOBRICK300N:
//                motor_pos_real[motor] = myoBrick300NEncoderTicksPerMeter(msg->position[motor]);
//                motor_vel_real[motor] = myoBrick300NEncoderTicksPerMeter(msg->velocity[motor]);
//                displacement_real[motor] = (msg->angle[motor]/4096.0)*2.0*M_PI*0.003; // rad to meter
//                break;
//
//        }
//    }
//}
//
//void CASPR::ElbowJointAngle(const std_msgs::Float32::ConstPtr &msg){
////    ROS_DEBUG_THROTTLE(5,"receiving joint angle data %f", msg->data);
////    elbow_joint_angle = msg->data;
//}
//
//void CASPR::update_V() {
//    static int counter = 0;
//    V.setZero(number_of_cables, 6 * number_of_links);
//    for (int muscle_index = 0; muscle_index < muscles.size(); muscle_index++) {
//        for (auto &segment:segments[muscle_index]) {
//            if (segment.first->link_name != segment.second->link_name) { // ignore redundant cables
//                Vector4d temp_vec;
//                // V term associated with segment translation
//                Vector4d v0(segment.first->global_coordinates[0], segment.first->global_coordinates[1],
//                            segment.first->global_coordinates[2], 1);
//                Vector4d v1(segment.second->global_coordinates[0], segment.second->global_coordinates[1],
//                            segment.second->global_coordinates[2], 1);
//                temp_vec = v1 - v0;
//                Vector3d segmentVector(temp_vec(0), temp_vec(1), temp_vec(2));
//                segmentVector.normalize();
//
//                int k = link_index[segment.first->link_name];
//                if (k > 0) {
//                    // convert to link coordinate frame
//                    Matrix4d transformMatrix = world_to_link_transform[link_names[k]];
//                    Matrix3d rotate_to_link_frame = transformMatrix.block(0, 0, 3, 3);
//
//                    Vector3d segmentVector_k = rotate_to_link_frame * segmentVector;
//
//                    // Total V term in translations
//                    Vector3d V_ijk_T = Vector3d(segmentVector_k(0), segmentVector_k(1), segmentVector_k(2));
//
//                    Vector3d temp_vec2 = segment.first->local_coordinates;
//
//                    Vector3d V_itk_T = temp_vec2.cross(V_ijk_T);
//
//                    V.block(muscle_index, 6 * k, 1, 3) = V_ijk_T.transpose();
//                    V.block(muscle_index, 6 * k + 3, 1, 3) = V_itk_T.transpose();
//                    if (log && (counter % 100 == 0)) {
//                        log_file << "-------------" << muscles[muscle_index].name << "\tk = " << k
//                                 << " startpoint" << endl;
//                        log_file << "v0 " << v0.transpose() << endl;
//                        log_file << "v1 " << v1.transpose() << endl;
//                        log_file << "segmentVector " << segmentVector.transpose() << endl;
//                        log_file << "R_0k " << rotate_to_link_frame << endl;
//                        log_file << "segmentVector_k " << segmentVector_k.transpose() << endl;
//                        log_file << "V_ijk_T " << V_ijk_T.transpose() << endl;
//                        log_file << "V_itk_T " << V_itk_T.transpose() << endl;
//                        log_file << "V " << V << endl;
//                    }
//                }
//
//                k = link_index[segment.second->link_name];
//                if (k > 0) {
//                    // convert to link coordinate frame
//                    Matrix4d transformMatrix = world_to_link_transform[link_names[k]];
//                    Matrix3d rotate_to_link_frame = transformMatrix.block(0, 0, 3, 3);
//
//                    Vector3d segmentVector_k = rotate_to_link_frame * segmentVector;
//
//                    // Total V term in translations
//                    Vector3d V_ijk_T = Vector3d(segmentVector_k(0), segmentVector_k(1), segmentVector_k(2));
//
//                    Vector3d temp_vec2 = segment.second->local_coordinates;
//
//                    Vector3d V_itk_T = temp_vec2.cross(V_ijk_T);
//
//                    V.block(muscle_index, 6 * k, 1, 3) = V.block(muscle_index, 6 * k, 1, 3) + V_ijk_T.transpose();
//                    V.block(muscle_index, 6 * k + 3, 1, 3) =
//                            V.block(muscle_index, 6 * k + 3, 1, 3) + V_itk_T.transpose();
//                    if (log && (counter % 100 == 0)) {
//                        log_file << "-------------" << muscles[muscle_index].name << "\tk = " << k
//                                 << " endpoint" << endl;
//                        log_file << "v0 " << v0.transpose() << endl;
//                        log_file << "v1 " << v1.transpose() << endl;
//                        log_file << "segmentVector " << segmentVector.transpose() << endl;
//                        log_file << "R_0k " << rotate_to_link_frame << endl;
//                        log_file << "segmentVector_k " << segmentVector_k.transpose() << endl;
//                        log_file << "V_ijk_T " << V_ijk_T.transpose() << endl;
//                        log_file << "V_itk_T " << V_itk_T.transpose() << endl;
//                        log_file << "V " << V << endl;
//                    }
//                }
//            }
//        }
//    }
//    counter++;
//}
//
//void CASPR::update_S() {
//    S.setZero(6 * number_of_links, number_of_dofs);
//    int k = 1, j = 0;
//    for (auto &joint:joints) {
//        if (joint_types[j] == PRISMATIC)
//            S.block(6 * k, k - 1, 3, 1) = joint_axis[j].block(3, 0, 3, 1);
//        else
//            S.block(6 * k + 3, k - 1, 3, 1) = joint_axis[j].block(0, 0, 3, 1);
//        k++;
//        j++;
//    }
//}
//
//void CASPR::update_P() {
//    P.setZero(6 * number_of_links, 6 * number_of_links);
//    P.block(0, 0, 6, 6).setIdentity(6, 6);
//
//    Matrix3d R_ka;
//    Eigen::Matrix<double, 6, 6> Pak;
//
//    static int counter = 0;
//    for (int k = 1; k < number_of_links; k++) {
//        Matrix4d transformMatrix_k = world_to_link_transform[link_names[k]];
//        Matrix3d R_k0 = transformMatrix_k.block(0, 0, 3, 3);
//
//        for (int a = 1; a <= k; a++) {
//            Matrix4d transformMatrix_a = world_to_link_transform[link_names[a]];
//            Matrix3d R_0a = transformMatrix_a.block(0, 0, 3, 3).transpose();
//            R_ka = R_k0 * R_0a;
//
//            Matrix3d R_pe;
//            Vector3d r_OP, r_OG;
//            r_OP.setZero();
//
//            if (joint_types[a - 1] == PRISMATIC)
//                R_pe.setIdentity();
//            else
//                R_pe = AngleAxisd(q[a - 1], joint_axis[a - 1].block(0, 0, 3, 1));
//
//            // Calculate forward position kinematics
//            rl::math::Transform pos = kinematic->getFrame(a);
//            r_OP = R_0a.transpose() * pos.translation();
//
//            r_OG = R_k0 * transformMatrix_k.inverse().block(0, 3, 3, 1);
//
//            Pak.block(0, 0, 3, 3) = R_ka * R_pe.transpose();
//            Pak.block(0, 3, 3, 3) = -R_ka * EigenExtension::SkewSymmetric2(-r_OP + R_ka.transpose() * r_OG);
//            Pak.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
//            Pak.block(3, 3, 3, 3) = R_ka;
//            P.block(6 * k, 6 * a, 6, 6) = Pak;
//
//            if (log && (counter % 100 == 0)) {
//                Eigen::IOFormat fmt(4, 0, " ", ";\n", "", "", "[", "]");
//                log_file << "---------------------" << links[k]->getName() << "---" << links[a]->getName() << endl;
//                log_file << "R_0k = " << R_k0.format(fmt) << endl;
//                log_file << "R_0a = " << R_0a.format(fmt) << endl;
//                log_file << "R_ka = " << R_ka.format(fmt) << endl;
//                log_file << "R_pe = " << R_pe.format(fmt) << endl;
//                log_file << "r_OP = " << r_OP.format(fmt) << endl;
//                log_file << "r_OG = " << r_OG.format(fmt) << endl;
//                log_file << "Pak = " << Pak.format(fmt) << endl;
//            }
//        }
//    }
//
//    counter++;
//}
//
//VectorXd CASPR::resolve_function(MatrixXd &A_eq, VectorXd &b_eq, VectorXd &f_min, VectorXd &f_max) {
//    // Initialisation
//    int qp_print_level = PL_NONE;
//    nh->getParam("qp_print_level", qp_print_level);
//    qp_solver.setPrintLevel(static_cast<PrintLevel>(qp_print_level));
//    // Set the optimisation parameters
//    int nWSR = 1000;
//    nh->getParam("qp_working_sets", nWSR);
//
//    // Convert the parameters to be in the correct form
//    for (int i = 0; i < number_of_cables * number_of_cables; i++)
//        A[i] = 0;
//    for (int i = 0; i < number_of_cables; i++) {
//        lb[i] = f_min(i);
//        ub[i] = f_max(i);
//        for (int j = 0; j < number_of_dofs; j++) {
//            if (abs(A_eq(j, i)) > 1e-6) {
//                A[j * number_of_cables + i] = A_eq(j, i);
//            } else {
//                A[j * number_of_cables + i] = 0;
//            }
//        }
//    }
//    for (int j = 0; j < number_of_dofs; j++) {
//        if (abs(b_eq[j]) > 1e-6) {
//            b[j] = b_eq[j];
//        } else {
//            b[j] = 0;
//        }
//    }
//    VectorXd f_opt;
//    f_opt.resize(number_of_cables);
//    f_opt.setZero();
//
//    // Run the optimisation
//    if (first_time_solving) {
//        for (int i = 0; i < number_of_cables * number_of_cables; i++)
//            H[i] = 0;
//        for (int i = 0; i < number_of_cables; i++) {
//            // H is the identity matrix
//            H[i * number_of_cables + i] = 1;
//            g[i] = 0;
//        }
//        returnValue status = qp_solver.init(H, g, A, lb, ub, b, b, nWSR, nullptr);
//        first_time_solving = false;
//    } else {
//        returnValue status = qp_solver.hotstart(H, g, A, lb, ub, b, b, nWSR, nullptr);
//        switch (status) {
//            case SUCCESSFUL_RETURN: { // all good
//                qp_solver.getPrimalSolution(FOpt);
//                for (int i = 0; i < number_of_cables; i++) {
//                    f_opt(i) = FOpt[i];
//                }
//                ROS_INFO_STREAM_THROTTLE(1, "target cable forces:\n" << f_opt.transpose());
//                break;
//            }
//            default: {
//                ROS_ERROR_STREAM_THROTTLE(1, MessageHandling::getErrorCodeMessage(status));
//                if (qp_solver.isInfeasible()) {
//                    qp_solver.getPrimalSolution(FOpt);
//                    for (int i = 0; i < number_of_cables; i++) {
//                        f_opt(i) = FOpt[i];
//                    }
//                    ROS_WARN_STREAM_THROTTLE(1, "infeasible, primal solution " << f_opt.transpose());
////                    f_opt.setZero();
//                } else {
//                    if (status = (returnValue) 54)
//                        first_time_solving = true;
//                }
//            }
//        }
//    }
//
//    bool log;
//    nh->getParam("log", log);
//    if (log) {
//        Eigen::IOFormat fmt(4, 0, " ", ";\n", "", "", "[", "]");
//        log_file << "---------------------" << endl;
//        log_file << "A_eq = " << A_eq.format(fmt) << endl;
//        log_file << "b_eq = " << b_eq.transpose().format(fmt) << endl;
//    }
//
//    return f_opt;
//}
//
//bool CASPR::getTransform(const char *to, const char *from, Matrix4d &transform) {
//    tf::StampedTransform trans;
//    try {
//        tf_listener.lookupTransform(to, from, ros::Time(0), trans);
//    }
//    catch (tf::TransformException ex) {
//        ROS_WARN_THROTTLE(1, "%s", ex.what());
//        return false;
//    }
//
//    Eigen::Affine3d trans_;
//    tf::transformTFToEigen(trans, trans_);
//    transform = trans_.matrix();
//    return true;
//}
//
//void CASPR::publishTendons() {
//    visualization_msgs::Marker line_strip;
//    line_strip.header.frame_id = "world";
//    line_strip.header.stamp = ros::Time::now();
//    line_strip.ns = "tendons_" + end_effektor_name;
//    line_strip.action = visualization_msgs::Marker::ADD;
//    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
//    line_strip.scale.x = 0.003;
//    line_strip.color.r = 0;
//    line_strip.color.g = 0;
//    line_strip.color.b = 1.0;
//    line_strip.color.a = 1.0;
//    line_strip.pose.orientation.w = 1.0;
//    line_strip.lifetime = ros::Duration(0);
//    for (uint muscle = 0; muscle < muscles.size(); muscle++) {
//        line_strip.points.clear();
//        line_strip.id = message_counter++;
//        for (uint i = 1; i < muscles[muscle].viaPoints.size(); i++) {
//            geometry_msgs::Point p;
//            p.x = muscles[muscle].viaPoints[i - 1]->global_coordinates[0];
//            p.y = muscles[muscle].viaPoints[i - 1]->global_coordinates[1];
//            p.z = muscles[muscle].viaPoints[i - 1]->global_coordinates[2];
//            line_strip.points.push_back(p);
//            p.x = muscles[muscle].viaPoints[i]->global_coordinates[0];
//            p.y = muscles[muscle].viaPoints[i]->global_coordinates[1];
//            p.z = muscles[muscle].viaPoints[i]->global_coordinates[2];
//            line_strip.points.push_back(p);
//            if (controller == 2) { // position control
//                Vector3d pos = (muscles[muscle].viaPoints[i]->global_coordinates +
//                                muscles[muscle].viaPoints[i - 1]->global_coordinates) / 2.0;
//                char str[100];
//                sprintf(str, "%.3lf", motor_pos[muscle]);
//                publishText(pos, str, "world", "tendon_length", message_counter++, COLOR(1, 1, 1, 1), 1, 0.01);
//            }
//        }
//        visualization_pub.publish(line_strip);
////        for (uint i = 0; i < muscles[muscle].viaPoints.size(); i++) {
////            publishSphere(muscles[muscle].viaPoints[i]->global_coordinates,"world","viaPoints",message_counter++,COLOR(0,1,1,1),0.01,1);
////        }
//    }
//    int i = 0;
//    for (auto muscle:muscles) {
//        Matrix4d transform = world_to_link_transform[muscle.viaPoints.front()->link_name].inverse();
//        muscle.viaPoints.front()->global_coordinates = transform.block(0, 3, 3, 1) + transform.block(0, 0, 3, 3) * muscle.viaPoints.front()->local_coordinates;
//        char str[20];
//        if(active_motors.end() == active_motors.find(id))
//            sprintf(str,"motor%d",i);
//        else
//            sprintf(str,"motor%d",active_motors[id][i]);
//        publishText(muscle.viaPoints.front()->global_coordinates, str, "world", (end_effektor_name+"_motor_ids").c_str(), message_counter++, COLOR(0, 1, 1, 1), 5,0.01);
//        i++;
//    }
//}
//
//void CASPR::publishForces() {
//    visualization_msgs::Marker arrow;
//    arrow.header.frame_id = "world";
//    arrow.ns = "force_" + end_effektor_name;
//    arrow.type = visualization_msgs::Marker::ARROW;
//    arrow.color.a = 1.0;
//    arrow.lifetime = ros::Duration(0);
//    arrow.scale.x = 0.005;
//    arrow.scale.y = 0.01;
//    arrow.scale.z = 0.01;
//    arrow.pose.orientation.w = 1;
//    arrow.pose.orientation.x = 0;
//    arrow.pose.orientation.y = 0;
//    arrow.pose.orientation.z = 0;
//    arrow.action = visualization_msgs::Marker::ADD;
//
//    for (uint muscle = 0; muscle < muscles.size(); muscle++) {
//        for (uint i = 1; i < muscles[muscle].viaPoints.size(); i++) {
//            // actio
//            arrow.id = message_counter++;
//            arrow.color.r = 0.0f;
//            arrow.color.g = 1.0f;
//            arrow.color.b = 0.0f;
//            arrow.header.stamp = ros::Time::now();
//            arrow.points.clear();
//            geometry_msgs::Point p;
//            p.x = muscles[muscle].viaPoints[i - 1]->global_coordinates[0];
//            p.y = muscles[muscle].viaPoints[i - 1]->global_coordinates[1];
//            p.z = muscles[muscle].viaPoints[i - 1]->global_coordinates[2];
//            Vector3d dir = muscles[muscle].viaPoints[i - 1]->global_coordinates-muscles[muscle].viaPoints[i]->global_coordinates;
//            dir.normalize();
//            arrow.points.push_back(p);
//            p.x -= dir[0]*cable_forces[muscle] * 0.001; // show fraction of force
//            p.y -= dir[1]*cable_forces[muscle] * 0.001;
//            p.z -= dir[2]* cable_forces[muscle] * 0.001;
//            arrow.points.push_back(p);
//            visualization_pub.publish(arrow);
//            // reactio
//            arrow.id = message_counter++;
//            arrow.color.r = 1.0f;
//            arrow.color.g = 1.0f;
//            arrow.color.b = 0.0f;
//            arrow.header.stamp = ros::Time::now();
//            arrow.points.clear();
//            p.x = muscles[muscle].viaPoints[i]->global_coordinates[0];
//            p.y = muscles[muscle].viaPoints[i]->global_coordinates[1];
//            p.z = muscles[muscle].viaPoints[i]->global_coordinates[2];
//            arrow.points.push_back(p);
//            p.x += dir[0]*cable_forces[muscle] * 0.001; // show fraction of force
//            p.y += dir[1]* cable_forces[muscle] * 0.001;
//            p.z += dir[2]*cable_forces[muscle] * 0.001;
//            arrow.points.push_back(p);
//            visualization_pub.publish(arrow);
//        }
//    }
//}
//
//bool CASPR::ForwardKinematicsService(roboy_communication_middleware::ForwardKinematics::Request &req,
//                                     roboy_communication_middleware::ForwardKinematics::Response &res) {
//
//    rl::math::Vector target_q = q;
//    int i = 0;
//    for (auto val:req.angles) {
//        target_q[i] = val;
//        i++;
//    }
//    ROS_INFO_STREAM("Serving ForwardKinematics Service robot config " << target_q.transpose());
//    // Calculate forward position kinematics
//    kinematic->setPosition(target_q); // input value
//    kinematic->forwardPosition(); // modify model
//    const rl::math::Transform &x = kinematic->getOperationalPosition(0);
//
//    ROS_INFO_STREAM("endeffector position: " << x.linear());
//    res.pos.x = x.linear()(0, 0);
//    res.pos.y = x.linear()(1, 0);
//    res.pos.z = x.linear()(2, 0);
//    return true;
//}
//
//bool CASPR::InverseKinematicsService(roboy_communication_middleware::InverseKinematics::Request &req,
//                                     roboy_communication_middleware::InverseKinematics::Response &res) {
//    kinematic_ik->setPosition(q);
//    rl::mdl::NloptInverseKinematics ik(kinematic_ik);
//    ik.duration = std::chrono::seconds(5);
//    ik.epsilonTranslation = 0.05;
//    ik.epsilonRotation = 0.05;
//    rl::math::Transform x_new;
//    tf::poseMsgToEigen(req.pose, x_new);
//
//    ik.goals.push_back(::std::make_pair(x_new, 0)); // goal frame in world coordinates for first TCP
//
//    bool result = ik.solve();
//    if (result) {
//        rl::math::Vector solution = kinematic_ik->getPosition();
//        ROS_INFO_STREAM("ik solution " << solution.transpose());
//        for (int i = 0; i < solution.rows(); i++) {
//            res.angles.push_back(solution[i]);
//        }
//        kinematic_ik->setPosition(q);
//        return true;
//    } else {
//        ROS_ERROR("unable to solve ik for target %.3lf %.3lf %.3lf", req.pose.position.x, req.pose.position.y, req.pose.position.z);
//        kinematic_ik->setPosition(q);
//        return false;
//    }
//}
//
//bool CASPR::parseSDFusion(const string &sdf, vector<roboy_simulation::MuscInfo> &myoMuscles) {
//    // initialize TiXmlDocument doc with a string
//    TiXmlDocument doc;
//    if (!doc.Parse(sdf.c_str()) && doc.Error()) {
//        ROS_FATAL("Can't parse Musc muscles. Invalid robot description.");
//        return false;
//    }
//
//    TiXmlElement *root = doc.RootElement();
//    TiXmlElement *model = root->FirstChildElement("model");
//    TiXmlElement *plugin = model->FirstChildElement("plugin");
//
//    // Constructs the myoMuscles by parsing custom xml.
//    TiXmlElement *myoMuscle_it = NULL;
//    for (myoMuscle_it = plugin->FirstChildElement("myoMuscle"); myoMuscle_it;
//         myoMuscle_it = myoMuscle_it->NextSiblingElement("myoMuscle")) {
//        roboy_simulation::MuscInfo myoMuscle;
//        if (myoMuscle_it->Attribute("name")) {
//            myoMuscle.name = myoMuscle_it->Attribute("name");
//            // myoMuscle joint acting on
//            TiXmlElement *link_child_it = NULL;
//            for (link_child_it = myoMuscle_it->FirstChildElement("link"); link_child_it;
//                 link_child_it = link_child_it->NextSiblingElement("link")) {
//                string linkname = link_child_it->Attribute("name");
//                if (!linkname.empty()) {
//                    TiXmlElement *viaPoint_child_it = NULL;
//                    for (viaPoint_child_it = link_child_it->FirstChildElement("viaPoint"); viaPoint_child_it;
//                         viaPoint_child_it = viaPoint_child_it->NextSiblingElement("viaPoint")) {
//                        roboy_simulation::ViaPointInfo vp;
//                        vp.link_name = linkname;
//                        float x, y, z;
//                        if (sscanf(viaPoint_child_it->GetText(), "%f %f %f", &x, &y, &z) != 3) {
//                            ROS_ERROR_STREAM_NAMED("parser", "error reading [via point] (x y z)");
//                            return false;
//                        }
//                        vp.local_coordinates = math::Vector3(x, y, z);
//                        if (viaPoint_child_it->Attribute("type")) {
//                            string type = viaPoint_child_it->Attribute("type");
//                            if (type == "FIXPOINT") {
//                                vp.type = roboy_simulation::IViaPoints::FIXPOINT;
//                            } else if (type == "SPHERICAL" || type == "CYLINDRICAL") {
//                                if (viaPoint_child_it->QueryDoubleAttribute("radius", &vp.radius) != TIXML_SUCCESS) {
//                                    ROS_ERROR_STREAM_NAMED("parser", "error reading radius");
//                                    return false;
//                                }
//                                if (viaPoint_child_it->QueryIntAttribute("state", &vp.state) != TIXML_SUCCESS) {
//                                    ROS_ERROR_STREAM_NAMED("parser", "error reading state");
//                                    return false;
//                                }
//                                if (viaPoint_child_it->QueryIntAttribute("revCounter", &vp.revCounter) !=
//                                    TIXML_SUCCESS) {
//                                    ROS_ERROR_STREAM_NAMED("parser", "error reading revCounter");
//                                    return false;
//                                }
//                                if (type == "SPHERICAL") {
//                                    vp.type = roboy_simulation::IViaPoints::SPHERICAL;
//                                } else {
//                                    vp.type = roboy_simulation::IViaPoints::CYLINDRICAL;
//                                }
//                            } else if (type == "MESH") {
//                                // TODO
//                            } else {
//                                ROS_ERROR_STREAM_NAMED("parser", "unknown type of via point: " + type);
//                                return false;
//                            }
//                        } else {
//                            ROS_ERROR_STREAM_NAMED("parser", "error reading type");
//                            return false;
//                        }
//                        myoMuscle.viaPoints.push_back(vp);
//                    }
//                    if (myoMuscle.viaPoints.empty()) {
//                        ROS_ERROR_STREAM_NAMED("parser", "No viaPoint element found in myoMuscle '"
//                                << myoMuscle.name << "' link element.");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No link name attribute specified for myoMuscle'"
//                            << myoMuscle.name << "'.");
//                    continue;
//                }
//            }
//            ROS_INFO("%ld viaPoints for myoMuscle %s", myoMuscle.viaPoints.size(), myoMuscle.name.c_str());
//
//            //check if wrapping surfaces are enclosed by fixpoints
//            for (int i = 0; i < myoMuscle.viaPoints.size(); i++) {
//                if (i == 0 && myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT) {
//                    ROS_ERROR_STREAM_NAMED("parser", "muscle insertion has to be a fix point");
//                    return false;
//                }
//                if (i == myoMuscle.viaPoints.size() - 1 &&
//                    myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT) {
//                    ROS_ERROR_STREAM_NAMED("parser", "muscle fixation has to be a fix point");
//                    return false;
//                }
//                if (myoMuscle.viaPoints[i].type != roboy_simulation::IViaPoints::FIXPOINT) {
//                    if (myoMuscle.viaPoints[i - 1].type != roboy_simulation::IViaPoints::FIXPOINT
//                        || myoMuscle.viaPoints[i + 1].type != roboy_simulation::IViaPoints::FIXPOINT) {
//                        ROS_ERROR_STREAM_NAMED("parser",
//                                               "non-FIXPOINT via-points have to be enclosed by two FIXPOINT via-points");
//                        return false;
//                    }
//                }
//            }
//
//            TiXmlElement *motor_child = myoMuscle_it->FirstChildElement("motor");
//            if (motor_child) {
//                // bemf_constant
//                TiXmlElement *bemf_constant_child = motor_child->FirstChildElement("bemf_constant");
//                if (bemf_constant_child) {
//                    if (sscanf(bemf_constant_child->GetText(), "%lf", &myoMuscle.motor.BEMFConst) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading bemf_constant constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No bemf_constant element found in myoMuscle '"
//                            << myoMuscle.name << "' motor element.");
//                    return false;
//                }
//                // torque_constant
//                TiXmlElement *torque_constant_child = motor_child->FirstChildElement("torque_constant");
//                if (torque_constant_child) {
//                    if (sscanf(torque_constant_child->GetText(), "%lf", &myoMuscle.motor.torqueConst) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading torque_constant constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No torque_constant element found in myoMuscle '"
//                            << myoMuscle.name << "' motor element.");
//                    return false;
//                }
//                // inductance
//                TiXmlElement *inductance_child = motor_child->FirstChildElement("inductance");
//                if (inductance_child) {
//                    if (sscanf(inductance_child->GetText(), "%lf", &myoMuscle.motor.inductance) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading inductance constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No inductance element found in myoMuscle '"
//                            << myoMuscle.name << "' motor element.");
//                    return false;
//                }
//                // resistance
//                TiXmlElement *resistance_child = motor_child->FirstChildElement("resistance");
//                if (resistance_child) {
//                    if (sscanf(resistance_child->GetText(), "%lf", &myoMuscle.motor.resistance) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading resistance constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No resistance element found in myoMuscle '"
//                            << myoMuscle.name << "' motor element.");
//                    return false;
//                }
//                // inertiaMoment
//                TiXmlElement *inertiaMoment_child = motor_child->FirstChildElement("inertiaMoment");
//                if (inertiaMoment_child) {
//                    if (sscanf(inertiaMoment_child->GetText(), "%lf", &myoMuscle.motor.inertiaMoment) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading inertiaMoment constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No inertiaMoment element found in myoMuscle '"
//                            << myoMuscle.name << "' motor element.");
//                    return false;
//                }
//            } else {
//                ROS_DEBUG_STREAM_NAMED("parser", "No motor element found in myoMuscle '" << myoMuscle.name <<
//                                                                                         "', using default parameters");
//            }
//
//            TiXmlElement *gear_child = myoMuscle_it->FirstChildElement("gear");
//            if (gear_child) {
//                // ratio
//                TiXmlElement *ratio_child = gear_child->FirstChildElement("ratio");
//                if (ratio_child) {
//                    if (sscanf(ratio_child->GetText(), "%lf", &myoMuscle.gear.ratio) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading ratio constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No ratio element found in myoMuscle '"
//                            << myoMuscle.name << "' gear element.");
//                    return false;
//                }
//                // ratio
//                TiXmlElement *efficiency_child = gear_child->FirstChildElement("efficiency");
//                if (efficiency_child) {
//                    if (sscanf(efficiency_child->GetText(), "%lf", &myoMuscle.gear.efficiency) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading efficiency constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No efficiency element found in myoMuscle '"
//                            << myoMuscle.name << "' gear element.");
//                    return false;
//                }
//                // inertiaMoment
//                TiXmlElement *inertiaMoment_child = gear_child->FirstChildElement("inertiaMoment");
//                if (inertiaMoment_child) {
//                    if (sscanf(inertiaMoment_child->GetText(), "%lf", &myoMuscle.gear.inertiaMoment) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading inertiaMoment constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No inertiaMoment element found in myoMuscle '"
//                            << myoMuscle.name << "' gear element.");
//                    return false;
//                }
//            } else {
//                ROS_DEBUG_STREAM_NAMED("parser", "No gear element found in myoMuscle '" << myoMuscle.name <<
//                                                                                        "', using default parameters");
//            }
//
//            TiXmlElement *spindle_child = myoMuscle_it->FirstChildElement("spindle");
//            if (spindle_child) {
//                // radius
//                TiXmlElement *radius_child = spindle_child->FirstChildElement("radius");
//                if (radius_child) {
//                    if (sscanf(radius_child->GetText(), "%lf", &myoMuscle.spindle.radius) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading radius constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No radius element found in myoMuscle '"
//                            << myoMuscle.name << "' spindle element.");
//                    return false;
//                }
//            } else {
//                ROS_DEBUG_STREAM_NAMED("parser",
//                                       "No spindle element found in myoMuscle '" << myoMuscle.name <<
//                                                                                 "', using default parameters");
//            }
//
//            TiXmlElement *SEE_child = myoMuscle_it->FirstChildElement("SEE");
//            if (SEE_child) {
//                // stiffness
//                TiXmlElement *stiffness_child = SEE_child->FirstChildElement("stiffness");
//                if (stiffness_child) {
//                    if (sscanf(stiffness_child->GetText(), "%lf", &myoMuscle.see.stiffness) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading radius constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No stiffness element found in myoMuscle '"
//                            << myoMuscle.name << "' SEE element.");
//                    return false;
//                }
//                // length
//                TiXmlElement *length_child = SEE_child->FirstChildElement("length");
//                if (length_child) {
//                    if (sscanf(length_child->GetText(), "%lf", &myoMuscle.see.length) != 1) {
//                        ROS_ERROR_STREAM_NAMED("parser", "error reading length constant");
//                        return false;
//                    }
//                } else {
//                    ROS_ERROR_STREAM_NAMED("parser", "No length element found in myoMuscle '"
//                            << myoMuscle.name << "' SEE element.");
//                    return false;
//                }
//            } else {
//                ROS_DEBUG_STREAM_NAMED("parser", "No SEE element found in myoMuscle '" << myoMuscle.name <<
//                                                                                       "', using default parameters");
//            }
//
//        } else {
//            ROS_ERROR_STREAM_NAMED("parser",
//                                   "No name attribute specified for myoMuscle, please name the muscle in sdf file");
//            return false;
//        }
//        myoMuscles.push_back(myoMuscle);
//    }
//    return true;
//}