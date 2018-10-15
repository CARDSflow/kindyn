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

    if (!ok) {
        ROS_FATAL_STREAM("KinDynComputationsWithEigen: impossible to load model from " << urdf_file_path);
        return;
    }

    // Create a KinDynComputations class from the model
    ok = kinDynComp.loadRobotModel(mdlLoader.model());

    if (!ok) {
        ROS_FATAL_STREAM(
                "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:"
                        << std::endl
                        << mdlLoader.model().toString());
        return;
    }

    ROS_INFO_STREAM(kinDynComp.getDescriptionOfDegreesOfFreedom());

//    kinDynComp.setFloatingBase("base");
    const iDynTree::Model &model = kinDynComp.model();
    number_of_dofs = model.getNrOfDOFs();
    number_of_joints = model.getNrOfJoints();
    number_of_links = model.getNrOfLinks();

    if (!parseViapoints(viapoints_file_path, cables)) {
        ROS_FATAL("something went wrong parsing the viapoints");
        return;
    }
    number_of_cables = cables.size();

    ROS_INFO_STREAM(
            "robot:\ndofs: " << number_of_dofs << "\njoints: " << number_of_joints << "\nlinks: " << number_of_links
                             << "\nnumber_of_cables: " << number_of_cables);

    baseVel.setZero();
    world_H_base.setIdentity();
    gravity << 0, 0, -9.81;

    // get the link names of the kinematic chain
    int j = 0;
    for (int link = 0; link < number_of_links; link++) {
        string link_name = model.getLinkName(link);
        link_names.push_back(link_name);
        ROS_INFO_STREAM(link_name);
        link_index[link_name] = link;
    }
    for (int joint = 0; joint < number_of_dofs; joint++) {
        iDynTree::Vector6 s = model.getJoint(joint)->getMotionSubspaceVector(0, model.getJoint(
                joint)->getSecondAttachedLink(), model.getJoint(joint)->getFirstAttachedLink()).asVector();
        string joint_name = model.getJointName(joint);
        joint_names.push_back(joint_name);
        VectorXd axis = iDynTree::toEigen(s);
        joint_axis.push_back(axis);
        ROS_INFO_STREAM(joint_name << " " << axis.format(fmt));
        joint_index[joint_name] = joint;
    }

    init();

    // ros control, NOTE: most of our joints cannot be directly position controlled, for those this class is registered
    // as a specialized hardware interface, giving the controllers the informations about the tendon space
    for (int joint = 0; joint < number_of_dofs; joint++) {
        // connect and register the joint state interface
        hardware_interface::CardsflowStateHandle state_handle(joint_names[joint], joint, &q[joint], &qd[joint], &qdd[joint], &L, &M, &CG

        );
        cardsflow_state_interface.registerHandle(state_handle);

        // connect and register the joint position interface
        hardware_interface::CardsflowHandle pos_handle(cardsflow_state_interface.getHandle(joint_names[joint]),
                                                       &q_target[joint], &qd_target[joint], &ld[joint]);
        cardsflow_command_interface.registerHandle(pos_handle);
    }
    registerInterface(&cardsflow_command_interface);
}

Robot::~Robot() {
    delete[] H;
    delete[] g;
    delete[] A;
    delete[] lb;
    delete[] ub;
    delete[] b;
    delete[] FOpt;
}

void Robot::init() {
    q.resize(number_of_dofs);
    qd.resize(number_of_dofs);
    qdd.resize(number_of_dofs);

    q_target.resize(number_of_dofs);
    qd_target.resize(number_of_dofs);
    qdd_target.resize(number_of_dofs);

    q.setZero();
    qd.setZero();
    qdd.setZero();
    q_target.setZero();
    qd_target.setZero();
    qdd_target.setZero();

    e.resize(number_of_dofs);
    e.setZero();
    de.resize(number_of_dofs);
    de.setZero();
    dde.resize(number_of_dofs);
    dde.setZero();

    M.resize(number_of_dofs + 6, number_of_dofs + 6);
    CG.resize(number_of_dofs);
    CG.setZero();

    const iDynTree::Model &model = kinDynComp.model();
    bias = iDynTree::FreeFloatingGeneralizedTorques(model);
    Mass = iDynTree::MatrixDynSize(number_of_dofs + 6, number_of_dofs + 6);

    iDynTree::Transform world_H_base_;
    iDynTree::VectorDynSize jointPos, jointVel;
    jointPos.resize(number_of_dofs);
    jointVel.resize(number_of_dofs);
    iDynTree::Twist baseVel_;
    iDynTree::Vector3 gravity_;
    iDynTree::fromEigen(world_H_base_, world_H_base);
    toEigen(jointPos) = q;
    iDynTree::fromEigen(baseVel_, baseVel);
    toEigen(jointVel) = qd;
    toEigen(gravity_) = gravity;

    kinDynComp.setRobotState(world_H_base_, jointPos, baseVel_, jointVel, gravity_);
    kinDynComp.generalizedBiasForces(bias);
    kinDynComp.getFreeFloatingMassMatrix(Mass);
    CG = toEigen(bias.jointTorques());
    M = toEigen(Mass);

    world_to_link_transform.resize(number_of_links);

    V.resize(number_of_cables, 6 * number_of_links);
    V.setZero(number_of_cables, 6 * number_of_links);
    P.resize(6 * number_of_links, 6 * number_of_links);
    P.setZero(6 * number_of_links, 6 * number_of_links);
    S.resize(6 * number_of_links, number_of_dofs);
    S.setZero(6 * number_of_links, number_of_dofs);

    segments.resize(number_of_cables);

    int muscle_index = 0;
    for (auto muscle:cables) {
        muscle.viaPoints[0]->link_index = model.getLinkIndex(muscle.viaPoints[0]->link_name);
        for (int i = 1; i < muscle.viaPoints.size(); i++) {
            muscle.viaPoints[i]->link_index = model.getLinkIndex(muscle.viaPoints[i]->link_name);
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

    qp_solver = SQProblem(number_of_cables, number_of_dofs);

    H = new real_t[number_of_cables * number_of_cables];
    g = new real_t[number_of_cables];
    A = new real_t[number_of_cables * number_of_cables];
    lb = new real_t[number_of_cables];
    ub = new real_t[number_of_cables];
    b = new real_t[number_of_dofs];
    FOpt = new real_t[number_of_cables];
    l.resize(number_of_cables);
    ld.resize(number_of_dofs);
    l.setZero();
    for(int i=0;i<number_of_dofs;i++){
        ld[i].resize(number_of_cables);
        ld[i].setZero();
    }

}

VectorXd Robot::resolve_function(MatrixXd &A_eq, VectorXd &b_eq, VectorXd &f_min, VectorXd &f_max) {
    // Initialisation
    int qp_print_level = PL_NONE;
    nh->getParam("qp_print_level", qp_print_level);
    qp_solver.setPrintLevel(static_cast<PrintLevel>(qp_print_level));
    // Set the optimisation parameters
    int nWSR = 1000;
    nh->getParam("qp_working_sets", nWSR);

    // Convert the parameters to be in the correct form
    for (int i = 0; i < number_of_cables * number_of_cables; i++)
        A[i] = 0;
    for (int i = 0; i < number_of_cables; i++) {
        lb[i] = f_min(i);
        ub[i] = f_max(i);
        for (int j = 0; j < number_of_dofs; j++) {
            if (abs(A_eq(j, i)) > 1e-6) {
                A[j * number_of_cables + i] = A_eq(j, i);
            } else {
                A[j * number_of_cables + i] = 0;
            }
        }
    }
    for (int j = 0; j < number_of_dofs; j++) {
        if (abs(b_eq[j]) > 1e-6) {
            b[j] = b_eq[j];
        } else {
            b[j] = 0;
        }
    }
    VectorXd f_opt;
    f_opt.resize(number_of_cables);
    f_opt.setZero();

    // Run the optimisation
    if (first_time_solving) {
        for (int i = 0; i < number_of_cables * number_of_cables; i++)
            H[i] = 0;
        for (int i = 0; i < number_of_cables; i++) {
            // H is the identity matrix
            H[i * number_of_cables + i] = 1;
            g[i] = 0;
        }
        returnValue status = qp_solver.init(H, g, A, lb, ub, b, b, nWSR, nullptr);
        first_time_solving = false;
    } else {
        returnValue status = qp_solver.hotstart(H, g, A, lb, ub, b, b, nWSR, nullptr);
        switch (status) {
            case SUCCESSFUL_RETURN: { // all good
                qp_solver.getPrimalSolution(FOpt);
                for (int i = 0; i < number_of_cables; i++) {
                    f_opt(i) = FOpt[i];
                }
                ROS_INFO_STREAM_THROTTLE(1, "target cable forces:\n" << f_opt.transpose());
                break;
            }
            default: {
                ROS_ERROR_STREAM_THROTTLE(1, MessageHandling::getErrorCodeMessage(status));
                if (qp_solver.isInfeasible()) {
                    qp_solver.getPrimalSolution(FOpt);
                    for (int i = 0; i < number_of_cables; i++) {
                        f_opt(i) = FOpt[i];
                    }
                    ROS_WARN_STREAM_THROTTLE(1, "infeasible, primal solution " << f_opt.transpose());
//                    f_opt.setZero();
                } else {
                    if (status = (returnValue) 54)
                        first_time_solving = true;
                }
            }
        }
    }

    bool log;
    nh->getParam("log", log);
    if (log) {
        Eigen::IOFormat fmt(4, 0, " ", ";\n", "", "", "[", "]");
        log_file << "---------------------" << endl;
        log_file << "A_eq = " << A_eq.format(fmt) << endl;
        log_file << "b_eq = " << b_eq.transpose().format(fmt) << endl;
    }

    return f_opt;
}

void Robot::update(double period) {
    forwardKinematics(period);

    iDynTree::Transform world_H_base_;
    iDynTree::VectorDynSize jointPos, jointVel;
    jointPos.resize(number_of_dofs);
    jointVel.resize(number_of_dofs);
    iDynTree::Twist baseVel_;
    iDynTree::Vector3 gravity_;
    iDynTree::fromEigen(world_H_base_, world_H_base);
    iDynTree::toEigen(jointPos) = q;
    iDynTree::fromEigen(baseVel_, baseVel);
    toEigen(jointVel) = qd;
    toEigen(gravity_) = gravity;

    kinDynComp.setRobotState(world_H_base_, jointPos, baseVel_, jointVel, gravity_);
    kinDynComp.generalizedBiasForces(bias);
    kinDynComp.getFreeFloatingMassMatrix(Mass);
    CG = toEigen(bias.jointTorques());
    M = toEigen(Mass);

    const iDynTree::Model &model = kinDynComp.model();
    for (int i = 0; i < number_of_links; i++) {
        Matrix4d pose = iDynTree::toEigen(kinDynComp.getRelativeTransform(0, i).asHomogeneousTransform());
        Vector3d com = iDynTree::toEigen(model.getLink(i)->getInertia().getCenterOfMass());
        pose.block(0, 3, 3, 1) += pose.block(0, 0, 3, 3) * com;
        world_to_link_transform[i] = pose.inverse();
    }

    for (auto muscle:cables) {
        for (auto vp:muscle.viaPoints) {
            Matrix4d transform = world_to_link_transform[vp->link_index].inverse();
            vp->global_coordinates = transform.block(0, 3, 3, 1) + transform.block(0, 0, 3, 3) * vp->local_coordinates;
        }
    }

    update_V();
    update_P();

    W = P * S;
    L = V * W;
    L_t = -L.transpose();
}

void Robot::updateController() {
//    static int counter = 0;
//
//    vector<double> q_target_;
//    nh->getParam("q_target", q_target_);
//    for (int i = 0; i < number_of_dofs; i++)
//        q_target[i] = q_target_[i];
//
//    nh->getParam("Kp", Kp);
//    nh->getParam("Kd", Kd);
//
//    e = q_target - q;
//    de = qd_target - qd;
//
//    VectorXd q_dd_cmd = qdd_target + Kp * e + Kd * de;
//
//    torques = M.block(6, 6, number_of_dofs, number_of_dofs) * q_dd_cmd + CG;// + w_ext
//
//    nh->getParam("controller", controller);
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
////            if (simulate) {
////                int i = 0;
////                roboy_communication_middleware::TorqueControl msg;
////                msg.request.joint_names = joint_names;
////                for (auto &joint_name:joint_names) {
////                    if (abs(torques[i]) < 10000000 && isfinite(torques[i]))
////                        msg.request.torque.push_back(torques[i]);
////                    else
////                        msg.request.torque.push_back(0);
////                    i++;
////                }
////                if (!torque_control_srv.call(msg))
////                    ROS_WARN_ONCE("could not set torque, service unavailable");
////            }
//            break;
//        }
//        case 2: { // position control
//            ROS_WARN_THROTTLE(5, "position controller active");
//            ld = L * (Kd * (qd_target - qd) + Kp * (q_target - q));
////            torques.setZero();
//            break;
//        }
//        default: { // deactivate
//            torques.setZero();
//            ROS_WARN_THROTTLE(5, "controller not active");
//            break;
//        }
//    }
//    ROS_INFO_STREAM_THROTTLE(1, "torques " << torques.transpose());
    ROS_INFO_STREAM_THROTTLE(1, "M " << M.format(fmt));
    ROS_INFO_STREAM_THROTTLE(1, "C+G " << CG.transpose());
    ROS_INFO_STREAM_THROTTLE(1, "qdd " << qdd.transpose());
    ROS_INFO_STREAM_THROTTLE(1, "qd " << qd.transpose());
    ROS_INFO_STREAM_THROTTLE(1, "q " << q.transpose());
}

void Robot::forwardKinematics(double dt) {
    const iDynTree::Model &model = kinDynComp.model();
    vector<double> q_target_;
    nh->getParam("q_target", q_target_);
    for (int i = 0; i < number_of_dofs; i++)
        q_target[i] = q_target_[i];
    nh->getParam("controller", controller);
    switch (controller) {
        case 0:
//            qdd = M.block(6, 6, number_of_dofs, number_of_dofs).inverse() * (-L.transpose() * cable_forces + CG);
            qd += qdd * dt;
            q += qd * dt;
//            ROS_INFO_STREAM_THROTTLE(1, "torques from tendons:" << endl << (L.transpose() * cable_forces).transpose());
            break;
        case 1:
//            qdd = M.block(6, 6, number_of_dofs, number_of_dofs).inverse() * (torques + CG);
            qd += qdd * dt;
            q += qd * dt;
            break;
        case 2:
            VectorXd Ld;
            Ld.resize(number_of_cables);
            Ld.setZero();
            for(int i=0; i<number_of_dofs; i++) {
//                ROS_INFO_STREAM_THROTTLE(1,i << " " << ld[i].transpose().format(fmt));
                Ld += ld[i];
            }
            qd = EigenExtension::Pinv(L) * Ld;
            q += qd * dt;
            l += Ld * dt;
            break;
    }

    moveit_msgs::DisplayRobotState msg;
    sensor_msgs::JointState msg2;
    static int id = 0;
    msg.state.joint_state.header.frame_id = "world";
    msg.state.joint_state.header.seq = id++;
    msg.state.joint_state.header.stamp = ros::Time::now();
    int j = 0;

    for (auto joint:joint_names) {
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
        j++;
    }
    msg2 = msg.state.joint_state;
    robot_state_pub.publish(msg);
    joint_state_pub.publish(msg2);

    for (int i = 0; i < number_of_links; i++) {
        tf::Transform trans;
        Affine3d aff;
        aff.matrix() = world_to_link_transform[i].inverse();
        tf::transformEigenToTF(aff, trans);
        tf_broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "world", link_names[i].c_str()));
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
                // V term associated with segment translation
                Vector3d segmentVector;
                segmentVector = segment.second->global_coordinates - segment.first->global_coordinates;
                segmentVector.normalize();

                int k = segment.first->link_index;
                if (k > 0) {
                    // convert to link coordinate frame
                    Matrix4d transformMatrix = world_to_link_transform[k];
                    Matrix3d rotate_to_link_frame = transformMatrix.block(0, 0, 3, 3);

                    Vector3d segmentVector_k = rotate_to_link_frame * segmentVector;

                    // Total V term in translations
                    Vector3d V_ijk_T = Vector3d(segmentVector_k(0), segmentVector_k(1), segmentVector_k(2));

                    Vector3d temp_vec2 = segment.first->local_coordinates;

                    Vector3d V_itk_T = temp_vec2.cross(V_ijk_T);

                    V.block(muscle_index, 6 * k, 1, 3) = V_ijk_T.transpose();
                    V.block(muscle_index, 6 * k + 3, 1, 3) = V_itk_T.transpose();
                }

                k = segment.second->link_index;
                if (k > 0) {
                    // convert to link coordinate frame
                    Matrix4d transformMatrix = world_to_link_transform[k];
                    Matrix3d rotate_to_link_frame = transformMatrix.block(0, 0, 3, 3);

                    Vector3d segmentVector_k = rotate_to_link_frame * segmentVector;

                    // Total V term in translations
                    Vector3d V_ijk_T = Vector3d(segmentVector_k(0), segmentVector_k(1), segmentVector_k(2));

                    Vector3d temp_vec2 = segment.second->local_coordinates;

                    Vector3d V_itk_T = temp_vec2.cross(V_ijk_T);

                    V.block(muscle_index, 6 * k, 1, 3) = V.block(muscle_index, 6 * k, 1, 3) + V_ijk_T.transpose();
                    V.block(muscle_index, 6 * k + 3, 1, 3) =
                            V.block(muscle_index, 6 * k + 3, 1, 3) + V_itk_T.transpose();
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

    const iDynTree::Model &model = kinDynComp.model();

    static int counter = 0;
    for (int k = 1; k < number_of_links; k++) {
        Matrix4d transformMatrix_k = world_to_link_transform[k];
        Matrix3d R_k0 = transformMatrix_k.block(0, 0, 3, 3);

        for (int a = 1; a <= k; a++) {
            Matrix4d transformMatrix_a = world_to_link_transform[a];
            Matrix3d R_0a = transformMatrix_a.block(0, 0, 3, 3).transpose();
            R_ka = R_k0 * R_0a;

            Matrix3d R_pe;
            Vector3d r_OP, r_OG;
            r_OP.setZero();

            R_pe = AngleAxisd(q[a - 1], joint_axis[a - 1].block(0, 0, 3, 1));

            // Calculate forward position kinematics
            Eigen::MatrixXd pose = iDynTree::toEigen(model.getFrameTransform(a).asHomogeneousTransform());
            r_OP = R_0a.transpose() * pose.block(0, 3, 3, 1);

            r_OG = R_k0 * transformMatrix_k.inverse().block(0, 3, 3, 1);

            Pak.block(0, 0, 3, 3) = R_ka * R_pe.transpose();
            Pak.block(0, 3, 3, 3) = -R_ka * EigenExtension::SkewSymmetric2(-r_OP + R_ka.transpose() * r_OG);
            Pak.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
            Pak.block(3, 3, 3, 3) = R_ka;
            P.block(6 * k, 6 * a, 6, 6) = Pak;
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
    line_strip.ns = "tendons_";
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
        line_strip.id = muscle + 9999999;
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
            if (controller == 2) { // position control
                Vector3d pos = (cables[muscle].viaPoints[i]->global_coordinates +
                                cables[muscle].viaPoints[i - 1]->global_coordinates) / 2.0;
                char str[100];
                sprintf(str, "%.3lf", l[muscle]);
                publishText(pos, str, "world", "tendon_length", message_counter++, COLOR(1, 1, 1, 1), 1, 0.01);
            }
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
//    visualization_msgs::Marker arrow;
//    arrow.header.frame_id = "world";
//    arrow.ns = "force_";
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
//    for (uint muscle = 0; muscle < cables.size(); muscle++) {
//        for (uint i = 1; i < cables[muscle].viaPoints.size(); i++) {
//            // actio
//            arrow.id = message_counter++;
//            arrow.color.r = 0.0f;
//            arrow.color.g = 1.0f;
//            arrow.color.b = 0.0f;
//            arrow.header.stamp = ros::Time::now();
//            arrow.points.clear();
//            geometry_msgs::Point p;
//            p.x = cables[muscle].viaPoints[i - 1]->global_coordinates[0];
//            p.y = cables[muscle].viaPoints[i - 1]->global_coordinates[1];
//            p.z = cables[muscle].viaPoints[i - 1]->global_coordinates[2];
//            Vector3d dir = cables[muscle].viaPoints[i - 1]->global_coordinates -
//                           cables[muscle].viaPoints[i]->global_coordinates;
//            dir.normalize();
//            arrow.points.push_back(p);
//            p.x -= dir[0] * cable_forces[muscle] * 0.001; // show fraction of force
//            p.y -= dir[1] * cable_forces[muscle] * 0.001;
//            p.z -= dir[2] * cable_forces[muscle] * 0.001;
//            arrow.points.push_back(p);
//            visualization_pub.publish(arrow);
//            // reactio
//            arrow.id = message_counter++;
//            arrow.color.r = 1.0f;
//            arrow.color.g = 1.0f;
//            arrow.color.b = 0.0f;
//            arrow.header.stamp = ros::Time::now();
//            arrow.points.clear();
//            p.x = cables[muscle].viaPoints[i]->global_coordinates[0];
//            p.y = cables[muscle].viaPoints[i]->global_coordinates[1];
//            p.z = cables[muscle].viaPoints[i]->global_coordinates[2];
//            arrow.points.push_back(p);
//            p.x += dir[0] * cable_forces[muscle] * 0.001; // show fraction of force
//            p.y += dir[1] * cable_forces[muscle] * 0.001;
//            p.z += dir[2] * cable_forces[muscle] * 0.001;
//            arrow.points.push_back(p);
//            visualization_pub.publish(arrow);
//        }
//    }
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
                        cable.viaPoints.push_back(ViaPointPtr(new ViaPoint(link_name, local_coordinates)));
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