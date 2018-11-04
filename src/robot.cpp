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
    robot_state_pub = nh->advertise<geometry_msgs::PoseStamped>("/robot_state", 1);
    tendon_state_pub = nh->advertise<roboy_communication_simulation::Tendon>("/tendon_state", 1);
    joint_state_pub = nh->advertise<roboy_communication_simulation::JointState>("/joint_state", 1);
    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");
}

Robot::~Robot() {
//    delete[] H;
//    delete[] g;
//    delete[] A;
//    delete[] lb;
//    delete[] ub;
//    delete[] b;
//    delete[] FOpt;
    delete[] link_to_link_transform;
}

void Robot::init(string urdf_file_path, string viapoints_file_path, vector<string> joint_names_ordered) {
    // Helper class to load the model from an external format
    iDynTree::ModelLoader mdlLoader;
    bool ok ;
    if(joint_names_ordered.empty())
        ok = mdlLoader.loadModelFromFile(urdf_file_path);
    else
        ok = mdlLoader.loadReducedModelFromFile(urdf_file_path, joint_names_ordered);

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
        ROS_INFO_STREAM(joint_name << " " << axis.transpose().format(fmt));
        joint_index[joint_name] = joint;
    }

    q.resize(number_of_dofs);
    qd.resize(number_of_dofs);
    qdd.resize(number_of_dofs);
    qdd_torque_control.resize(number_of_dofs);
    qdd_torque_control.setZero();
    qdd_force_control.resize(number_of_dofs);
    qdd_force_control.setZero();

    q_target.resize(number_of_dofs);
    qd_target.resize(number_of_dofs);
    qdd_target.resize(number_of_dofs);

    q.setZero();
    qd.setZero();
    qdd.setZero();
    q_target.setZero();
    qd_target.setZero();
    qdd_target.setZero();

    l.resize(number_of_cables);
    Ld.resize(number_of_cables);
    Ld.setZero();
    ld.resize(number_of_dofs);
    l.setZero();
    for (int i = 0; i < number_of_dofs; i++) {
        ld[i].resize(number_of_cables);
        ld[i].setZero();
    }

    robotstate.resize(number_of_dofs);
    baseVel.setZero();
    world_H_base.setIdentity();
    gravity << 0, 0, -9.81;
    iDynTree::fromEigen(robotstate.world_H_base, world_H_base);
    iDynTree::toEigen(robotstate.jointPos) = q;
    iDynTree::fromEigen(robotstate.baseVel, baseVel);
    toEigen(robotstate.jointVel) = qd;
    toEigen(robotstate.gravity) = gravity;
    kinDynComp.setRobotState(robotstate.world_H_base, robotstate.jointPos, robotstate.baseVel, robotstate.jointVel,
                             robotstate.gravity);

    cable_forces.resize(number_of_cables);
    cable_forces.setZero();
    torques.resize(number_of_dofs);
    torques.setZero();

    controller_type.resize(number_of_cables, CARDSflow::ControllerType::cable_length_controller);
    joint_state.resize(number_of_dofs);
    motor_state.resize(number_of_cables);
    // ros control
    for (int joint = 0; joint < number_of_dofs; joint++) {
        ROS_INFO("initializing controllers for joint %d %s", joint, joint_names[joint].c_str());
        // connect and register the cardsflow state interface
        hardware_interface::CardsflowStateHandle state_handle(joint_names[joint], joint, &q[joint], &qd[joint],
                                                              &qdd[joint], &L, &M, &CG

        );
        cardsflow_state_interface.registerHandle(state_handle);

        // connect and register the cardsflow command interface
        hardware_interface::CardsflowHandle pos_handle(cardsflow_state_interface.getHandle(joint_names[joint]),
                                                       &q_target[joint], &qd_target[joint], &torques[joint], &ld[joint]);
        cardsflow_command_interface.registerHandle(pos_handle);
        // connect and register the cardsflow state interface
        hardware_interface::JointStateHandle state_handle2(joint_names[joint], &q[joint], &qd[joint], &q_target[joint]);
        joint_state_interface.registerHandle(state_handle2);

        // connect and register the cardsflow command interface
        hardware_interface::JointHandle torque_handle(joint_state_interface.getHandle(joint_names[joint]),
                                                      &torques[joint]);
        joint_command_interface.registerHandle(torque_handle);
        joint_state[joint][0] = 0;
        joint_state[joint][1] = 0;
    }
    registerInterface(&cardsflow_command_interface);
    registerInterface(&joint_command_interface);

    M.resize(number_of_dofs + 6, number_of_dofs + 6);
    CG.resize(number_of_dofs);
    CG.setZero();

    bias = iDynTree::FreeFloatingGeneralizedTorques(model);
    Mass = iDynTree::MatrixDynSize(number_of_dofs + 6, number_of_dofs + 6);

    kinDynComp.generalizedBiasForces(bias);
    kinDynComp.getFreeFloatingMassMatrix(Mass);
    CG = toEigen(bias.jointTorques());
    M = toEigen(Mass);

    world_to_link_transform.resize(number_of_links);
    link_to_world_transform.resize(number_of_links);
    frame_transform.resize(number_of_links);
    link_to_link_transform = new Matrix3d[number_of_links * number_of_links];

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
            muscle.viaPoints[i]->link_index = link_index[muscle.viaPoints[i]->link_name];
            pair<ViaPointPtr, ViaPointPtr> segment(muscle.viaPoints[i - 1], muscle.viaPoints[i]);
            segments[muscle_index].push_back(segment);
        }
        motor_state[muscle_index][0] = 0;
        motor_state[muscle_index][1] = 0;
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
    qp_solver.setPrintLevel(static_cast<PrintLevel>(qp_print_level));

    double min_force, max_force;
    nh->getParam("min_force", min_force);
    nh->getParam("max_force", max_force);

    f_min = VectorXd::Ones(number_of_cables);
    f_max = VectorXd::Ones(number_of_cables);

    f_min = min_force * f_min;
    f_max = max_force * f_max;

    last_visualization = ros::Time::now();

    int k=0;
    nh->getParam("endeffectors", endeffectors);
    endeffector_dof_offset.push_back(0);
    for (string ef:endeffectors) {
        ROS_INFO_STREAM("configuring endeffector " << ef);
        vector<string> ik_joints;
        nh->getParam((ef + "/joints"), ik_joints);
        if (ik_joints.empty()) {
            ROS_WARN(
                    "endeffector %s has no joints defined, check your endeffector.yaml or parameter server.  skipping...",
                    ef.c_str());
            continue;
        }
        endeffector_index[ef] = k;
        endeffector_number_of_dofs.push_back(ik_joints.size());
        if(k>0)
            endeffector_dof_offset.push_back(endeffector_dof_offset[k-1]+endeffector_number_of_dofs[k-1]);
        string base_link;
        nh->getParam((ef + "/base_link"), base_link);
        if (base_link.empty()) {
            ROS_WARN(
                    "endeffector %s has no base_link defined, check your endeffector.yaml or parameter server. skipping...",
                    ef.c_str());
            continue;
        }
        ik_base_link[ef] = base_link;

        iDynTree::ModelLoader mdlLoaderIK;
        bool ok = mdlLoaderIK.loadReducedModelFromFile(urdf_file_path, ik_joints);
        if (!ok) {
            ROS_FATAL_STREAM("Oh no!: impossible to load model from " << urdf_file_path);
            return;
        }

        // Create a KinDynComputations class from the model
        ok = ik_models[ef].loadRobotModel(mdlLoaderIK.model());
        if (!ok) {
            ROS_FATAL_STREAM(
                    "Oh no!: impossible to load the following model in a KinDynComputations class:"
                            << std::endl
                            << mdlLoader.model().toString());
            return;
        }
        ik[ef].setModel(ik_models[ef].getRobotModel());
        ik[ef].setVerbosity(0);
        tf::Vector3 pos(0,0.3*k,0);
        make6DofMarker(false,visualization_msgs::InteractiveMarkerControl::MOVE_3D,pos,false,0.15,"world",ef.c_str());
        k++;
    }

    controller_type_sub = nh->subscribe("/controller_type", 100, &Robot::controllerType, this);
    joint_state_sub = nh->subscribe("/joint_states", 100, &Robot::JointState, this);
    floating_base_sub = nh->subscribe("/floating_base", 100, &Robot::FloatingBase, this);
    ik_srv = nh->advertiseService("/ik", &Robot::InverseKinematicsService, this);
    fk_srv = nh->advertiseService("/fk", &Robot::ForwardKinematicsService, this);
    interactive_marker_sub = nh->subscribe("/interactive_markers/feedback",1,&Robot::InteractiveMarkerFeedback, this);
}

VectorXd Robot::resolve_function(MatrixXd &A_eq, VectorXd &b_eq, VectorXd &f_min, VectorXd &f_max) {
    nWSR = 1000;
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
    return f_opt;
}

void Robot::update() {
    ROS_INFO_STREAM_THROTTLE(5, "q_target " << q_target.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "qdd " << qdd.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "qd " << qd.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "q " << q.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "l " << l.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "ld " << Ld.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "torques " << torques.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "cable_forces " << cable_forces.transpose().format(fmt));


    ros::Time t0 = ros::Time::now();
    iDynTree::fromEigen(robotstate.world_H_base, world_H_base);
    iDynTree::toEigen(robotstate.jointPos) = q;
    iDynTree::fromEigen(robotstate.baseVel, baseVel);
    toEigen(robotstate.jointVel) = qd;
    toEigen(robotstate.gravity) = gravity;

    kinDynComp.setRobotState(robotstate.world_H_base, robotstate.jointPos, robotstate.baseVel, robotstate.jointVel,
                             robotstate.gravity);
    kinDynComp.generalizedBiasForces(bias);
    kinDynComp.getFreeFloatingMassMatrix(Mass);
    CG = toEigen(bias.jointTorques());
    M = toEigen(Mass);

    const iDynTree::Model &model = kinDynComp.model();
    for (int i = 0; i < number_of_links; i++) {
        Matrix4d pose = iDynTree::toEigen(kinDynComp.getWorldTransform(i).asHomogeneousTransform());
        Vector3d com = iDynTree::toEigen(model.getLink(i)->getInertia().getCenterOfMass());
        pose.block(0, 3, 3, 1) += pose.block(0, 0, 3, 3) * com;
        world_to_link_transform[i] = pose.inverse();
        link_to_world_transform[i] = pose;
        frame_transform[i] = iDynTree::toEigen(model.getFrameTransform(i).asHomogeneousTransform());
    }
    P.setZero(6 * number_of_links, 6 * number_of_links);
    P.block(0, 0, 6, 6).setIdentity(6, 6);
    for (int k = 1; k < number_of_links; k++) {
        for (int a = 1; a <= k; a++) {
            link_to_link_transform[k * number_of_links + a] =
                    world_to_link_transform[k].block(0, 0, 3, 3) * world_to_link_transform[a].block(0, 0, 3, 3);
            P.block(6 * k + 3, 6 * a + 3, 3, 3) = link_to_link_transform[k * number_of_links + a];
        }
    }

    for (auto muscle:cables) {
        for (auto vp:muscle.viaPoints) {
            if (!vp->fixed_to_world) { // move viapoint with link
                vp->global_coordinates = link_to_world_transform[vp->link_index].block(0, 3, 3, 1) +
                                         link_to_world_transform[vp->link_index].block(0, 0, 3, 3) *
                                         vp->local_coordinates;
            }
        }
    }
//    ROS_INFO_THROTTLE(1,"model update takes %f seconds", (ros::Time::now()-t0).toSec());
//    t0 = ros::Time::now();
    update_V();
//    ROS_INFO_THROTTLE(1,"update V takes %f seconds", (ros::Time::now()-t0).toSec());
//    t0 = ros::Time::now();
    update_P();
//    ROS_INFO_THROTTLE(1,"update P takes %f seconds", (ros::Time::now()-t0).toSec());

    W = P * S;
    L = V * W;
    L_t = -L.transpose();

//    ROS_INFO_STREAM_THROTTLE(5, "Lt " << L_t.transpose().format(fmt));
//    ROS_INFO_STREAM_THROTTLE(5, "P " << P.transpose().format(fmt));
//    ROS_INFO_STREAM_THROTTLE(5, "V " << V.transpose().format(fmt));
//    ROS_INFO_STREAM_THROTTLE(5, "S " << S.transpose().format(fmt));
    torque_position_controller_active = force_position_controller_active = cable_length_controller_active = false;
    for(auto type:controller_type){
        switch(type){
            case CARDSflow::ControllerType::cable_length_controller:
                cable_length_controller_active = true;
                break;
            case CARDSflow::ControllerType::torque_position_controller:
                torque_position_controller_active = true;
                break;
            case CARDSflow::ControllerType::force_position_controller:
                force_position_controller_active = true;
                break;
        }
    }

    // for the cable force controller with do a centralized update
    if(force_position_controller_active){
        cable_forces = resolve_function(L_t, torques, f_min, f_max);
    }

    if ((1.0 / (ros::Time::now() - last_visualization).toSec()) < 30) {
        { // tendon state publisher
            roboy_communication_simulation::Tendon msg;
            for (int i = 0; i < number_of_cables; i++) {
                msg.name.push_back(cables[i].name);
                msg.force.push_back(cable_forces[i]);
                msg.l.push_back(l[i]);
                msg.ld.push_back(Ld[i]);
                msg.number_of_viapoints.push_back(cables[i].viaPoints.size());
                for (auto vp:cables[i].viaPoints) {
                    geometry_msgs::Vector3 VP;
                    tf::vectorEigenToMsg(vp->global_coordinates, VP);
                    msg.viaPoints.push_back(VP);
                }
            }
            tendon_state_pub.publish(msg);
        }
        { // robot state publisher
            static int seq = 0;
            for (int i = 0; i < number_of_links; i++) {
                geometry_msgs::PoseStamped msg;
                msg.header.seq = seq++;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = link_names[i];
                Isometry3d iso(link_to_world_transform[i]);
                tf::poseEigenToMsg(iso, msg.pose);
                robot_state_pub.publish(msg);
            }
        }
        { // joint state publisher
            roboy_communication_simulation::JointState msg;
            msg.names = joint_names;
            for (int i = 1; i < number_of_links; i++) {
                Matrix4d pose = iDynTree::toEigen(kinDynComp.getWorldTransform(i).asHomogeneousTransform());
                Vector3d axis;
                axis << joint_axis[i - 1][3], joint_axis[i - 1][4], joint_axis[i - 1][5];
                axis = pose.block(0, 0, 3, 3) * axis;
                msg.origin.push_back(convertEigenToGeometry(pose.topRightCorner(3, 1)));
                msg.axis.push_back(convertEigenToGeometry(axis));
                msg.torque.push_back(torques[i - 1]);
            }
            joint_state_pub.publish(msg);
        }
        last_visualization = ros::Time::now();
    }
}

void Robot::forwardKinematics(double dt) {
    const iDynTree::Model &model = kinDynComp.model();

    if(torque_position_controller_active) // we do the calculations only if there is a controller active
        qdd_torque_control = M.block(6, 6, number_of_dofs, number_of_dofs).inverse() * (-torques - CG);
    if(force_position_controller_active) // we do the calculations only if there is a controller active
        qdd_force_control = M.block(6, 6, number_of_dofs, number_of_dofs).inverse() * (L_t * cable_forces - CG);

    for(int i = 0; i<endeffectors.size();i++) {
        int dof_offset = endeffector_dof_offset[i];
        Ld.setZero();
        for (int j = dof_offset; j < endeffector_number_of_dofs[i]+dof_offset; j++) {
            Ld -= ld[j];
        }

        MatrixXd L_endeffector = L.block(0,dof_offset,number_of_cables,endeffector_number_of_dofs[i]);
        MatrixXd L_endeffector_inv = EigenExtension::Pinv(L_endeffector);
        VectorXd qd_temp =  L_endeffector_inv * Ld;

        for (int j = dof_offset; j < endeffector_number_of_dofs[i]+dof_offset; j++) {
            switch(controller_type[j]){
                case CARDSflow::ControllerType::torque_position_controller:
                    boost::numeric::odeint::integrate(
                            [this, j](const state_type &x, state_type &dxdt, double t) {
                                dxdt[1] = qdd_torque_control[j];
                                dxdt[0] = x[1];
                            }, joint_state[j], integration_time, integration_time + dt, dt);
                    break;
                case CARDSflow::ControllerType::cable_length_controller:
                    boost::numeric::odeint::integrate(
                            [this, j, qd_temp, dof_offset](const state_type &x, state_type &dxdt, double t) {
                                dxdt[1] = 0;
                                dxdt[0] = qd_temp[j-dof_offset];
                            }, joint_state[j], integration_time, integration_time + dt, dt);
                    break;
                case CARDSflow::ControllerType::force_position_controller:
                    boost::numeric::odeint::integrate(
                            [this, j](const state_type &x, state_type &dxdt, double t) {
                                dxdt[1] = qdd_force_control[j];
                                dxdt[0] = x[1];
                            }, joint_state[j], integration_time, integration_time + dt, dt);
                    break;
            }
            qd[j] = joint_state[j][1];
            q[j] = joint_state[j][0];
//        ROS_INFO("%s control type %d", joint_names[j].c_str(), controller_type[j]);
        }
    }
    for (int i = 0; i < number_of_cables; i++) {
        boost::numeric::odeint::integrate(
                [this, i](const state_type &x, state_type &dxdt, double t) {
                    dxdt[1] = 0;
                    dxdt[0] = Ld[i];
                }, motor_state[i], integration_time, integration_time + dt, dt);
        l[i] = motor_state[i][0];
    }
    integration_time += dt;
    ROS_INFO_THROTTLE(5, "forward kinematics calculated for %lf s", integration_time);
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
                    // Total V term in translations
                    Vector3d V_ijk_T = world_to_link_transform[k].block(0, 0, 3, 3) * segmentVector;

                    Vector3d temp_vec2 = segment.first->local_coordinates;

                    Vector3d V_itk_T = temp_vec2.cross(V_ijk_T);

                    V.block(muscle_index, 6 * k, 1, 3) = V_ijk_T.transpose();
                    V.block(muscle_index, 6 * k + 3, 1, 3) = V_itk_T.transpose();
                }

                k = segment.second->link_index;
                if (k > 0) {
                    // Total V term in translations
                    Vector3d V_ijk_T = world_to_link_transform[k].block(0, 0, 3, 3) * segmentVector;

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
    int k = 1;
    for (auto &axis:joint_axis) {
        S.block(6 * k, k - 1, 6, 1) = axis;
        k++;
    }
//    ROS_INFO_STREAM("S_t = " << S.transpose().format(fmt));
}

void Robot::update_P() {
    static int counter = 0;
    Matrix3d R_pe;
    Vector3d r_OP, r_OG;
    Matrix3d skew;
    for (int k = 1; k < number_of_links; k++) {
        for (int a = 1; a <= k; a++) {
            R_pe = AngleAxisd(q[a - 1], joint_axis[a - 1].topLeftCorner(3, 1));
            r_OP = link_to_world_transform[a].topLeftCorner(3, 3) * frame_transform[a].topRightCorner(3, 1);
            r_OG = world_to_link_transform[k].topLeftCorner(3, 3) * link_to_world_transform[k].topRightCorner(3, 1);
            P.block(6 * k, 6 * a, 3, 3) = link_to_link_transform[k * number_of_links + a] * R_pe.transpose();
            Vector3d v = -r_OP + link_to_link_transform[k * number_of_links + a].transpose() * r_OG;
            skew << 0, -v(2), v(1),
                    v(2), 0, -v(0),
                    -v(1), v(0), 0;
            P.block(6 * k, 6 * a + 3, 3, 3) = -link_to_link_transform[k * number_of_links + a] * skew;
        }
    }

    counter++;
}

void Robot::controllerType(const roboy_communication_simulation::ControllerTypeConstPtr &msg) {
    auto it = find(joint_names.begin(), joint_names.end(),msg->joint_name);
    if(it!=joint_names.end()) {
        ROS_INFO("%s changed controller to %s", msg->joint_name.c_str(),
                 (msg->type==CARDSflow::ControllerType::cable_length_controller?"cable_length_controller":
                  msg->type==CARDSflow::ControllerType::torque_position_controller?"torque_position_controller":
                  msg->type==CARDSflow::ControllerType::force_position_controller?"force_position_controller":"UNKNOWN"));
        controller_type[distance(joint_names.begin(), it)] = msg->type;
    }
}

bool Robot::ForwardKinematicsService(roboy_communication_middleware::ForwardKinematics::Request &req,
                                     roboy_communication_middleware::ForwardKinematics::Response &res) {
    if (ik_models.find(req.endeffector) == ik_models.end()) {
        ROS_ERROR_STREAM("endeffector " << req.endeffector << " not initialized");
        return false;
    }
    int index = endeffector_index[req.endeffector];
    iDynTree::VectorDynSize jointPos, jointVel;
    jointPos.resize(endeffector_number_of_dofs[index]);
    jointVel.resize(endeffector_number_of_dofs[index]);
    iDynTree::toEigen(jointPos) = q.segment(endeffector_dof_offset[index], endeffector_number_of_dofs[index]);
    iDynTree::toEigen(jointVel) = qd.segment(endeffector_dof_offset[index], endeffector_number_of_dofs[index]);

    ik_models[req.endeffector].setRobotState(robotstate.world_H_base, jointPos, robotstate.baseVel,
                                             jointVel, robotstate.gravity);

    int i = 0;
    for (string joint:req.joint_names) {
        int joint_index = ik[req.endeffector].fullModel().getJointIndex(joint);
        if (joint_index != iDynTree::JOINT_INVALID_INDEX) {
            jointPos(joint_index) = req.angles[i];
        } else {
            ROS_ERROR("joint %s not found in model", joint.c_str());
        }
        i++;
    }
    ik_models[req.endeffector].setRobotState(robotstate.world_H_base, jointPos, robotstate.baseVel,
                                             jointVel, robotstate.gravity);
    iDynTree::Transform trans = ik_models[req.endeffector].getRelativeTransform(
            ik_models[req.endeffector].model().getFrameIndex(ik_base_link[req.endeffector]),
            ik_models[req.endeffector].model().getFrameIndex(req.frame));
    Eigen::Matrix4d frame = iDynTree::toEigen(trans.asHomogeneousTransform());
    Eigen::Isometry3d iso(frame);
    tf::poseEigenToMsg(iso, res.pose);
    return true;
}

bool Robot::InverseKinematicsService(roboy_communication_middleware::InverseKinematics::Request &req,
                                     roboy_communication_middleware::InverseKinematics::Response &res) {
    if (ik_models.find(req.endeffector) == ik_models.end()) {
        ROS_ERROR_STREAM("endeffector " << req.endeffector << " not initialized");
        return false;
    }
    int index = endeffector_index[req.endeffector];
    iDynTree::VectorDynSize jointPos, jointVel;
    jointPos.resize(endeffector_number_of_dofs[index]);
    jointVel.resize(endeffector_number_of_dofs[index]);
    iDynTree::toEigen(jointPos) = q.segment(endeffector_dof_offset[index], endeffector_number_of_dofs[index]);
    iDynTree::toEigen(jointVel) = qd.segment(endeffector_dof_offset[index], endeffector_number_of_dofs[index]);

    ik_models[req.endeffector].setRobotState(robotstate.world_H_base, jointPos, robotstate.baseVel,
                                             jointVel, robotstate.gravity);
    ik[req.endeffector].clearProblem();
    // we constrain the base link to stay where it is
    ik[req.endeffector].addTarget(ik_base_link[req.endeffector], ik_models[req.endeffector].model().getFrameTransform(
            ik_models[req.endeffector].getFrameIndex(ik_base_link[req.endeffector])));
    switch (req.type) {
        case 0: {
            Eigen::Isometry3d iso;
            tf::poseMsgToEigen(req.pose, iso);
            iDynTree::Transform trans;
            iDynTree::fromEigen(trans, iso.matrix());
            ik[req.endeffector].addTarget(req.frame, trans);
            break;
        }
        case 1: {
            iDynTree::Position pos(req.pose.position.x, req.pose.position.y, req.pose.position.z);
            ik[req.endeffector].addPositionTarget(req.frame, pos);
            break;
        }
        case 2: {
            Eigen::Quaterniond q(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y,
                                 req.pose.orientation.z);
            Eigen::Matrix3d rot = q.matrix();
            iDynTree::Rotation r(rot(0, 0), rot(0, 1), rot(0, 2), rot(1, 0), rot(1, 1), rot(1, 2), rot(2, 0), rot(2, 1),
                                 rot(2, 2));
            ik[req.endeffector].addRotationTarget(req.frame, r);
            break;
        }
    }
    if (ik[req.endeffector].solve()) {
        iDynTree::Transform base_solution;
        iDynTree::VectorDynSize q_star;
        ik[req.endeffector].getFullJointsSolution(base_solution, q_star);
        ROS_INFO_STREAM("ik solution:\n" << "base solution:" << base_solution.toString() << "\njoint solution: "
                                         << q_star.toString());
        for (int i = 0; i < q_star.size(); i++) {
            res.joint_names.push_back(ik[req.endeffector].reducedModel().getJointName(i));
            res.angles.push_back(q_star(i));
        }
        return true;
    } else {
        switch (req.type) {
            case 0:
                ROS_ERROR("unable to solve full pose ik for endeffector %s and target frame %s:\n"
                          "target_position    %.3lf %.3lf %.3lf"
                          "target_orientation %.3lf %.3lf %.3lf %.3lf", req.endeffector.c_str(), req.frame.c_str(),
                          req.pose.position.x, req.pose.position.y, req.pose.position.z,
                          req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y,
                          req.pose.orientation.z);
                break;
            case 1:
                ROS_ERROR("unable to solve position ik for endeffector %s and target frame %s:\n"
                          "target_position    %.3lf %.3lf %.3lf", req.endeffector.c_str(), req.frame.c_str(),
                          req.pose.position.x, req.pose.position.y, req.pose.position.z);
                break;

            case 2:
                ROS_ERROR("unable to solve orientation ik for endeffector %s and target frame %s:\n"
                          "target_orientation %.3lf %.3lf %.3lf %.3lf", req.endeffector.c_str(), req.frame.c_str(),
                          req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y,
                          req.pose.orientation.z);
                break;
        }

        return false;
    }
}

void Robot::InteractiveMarkerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg){
    if(msg->event_type!=visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
        return;
    auto it = find(endeffectors.begin(),endeffectors.end(),msg->marker_name);
    if(it!=endeffectors.end()){
        roboy_communication_middleware::InverseKinematics msg2;
        msg2.request.pose = msg->pose;
        msg2.request.endeffector = msg->marker_name;
        msg2.request.frame = msg->marker_name;
        msg2.request.type = 1;
        if(InverseKinematicsService(msg2.request,msg2.response)){
            int index = endeffector_index[msg->marker_name];
            for(int i=0;i<msg2.response.joint_names.size();i++){
                q_target[endeffector_dof_offset[index]+i] = msg2.response.angles[i];
            }

        }
    }
}

void Robot::JointState(const sensor_msgs::JointStateConstPtr &msg) {
    const iDynTree::Model &model = kinDynComp.getRobotModel();
    int i = 0;
    for (string joint:msg->name) {
        int joint_index = model.getJointIndex(joint);
        if (joint_index != iDynTree::JOINT_INVALID_INDEX) {
            q(joint_index) = msg->position[i];
            qd(joint_index) = msg->velocity[i];
        } else {
            ROS_ERROR("joint %s not found in model", joint.c_str());
        }
        i++;
    }
}

void Robot::FloatingBase(const geometry_msgs::PoseConstPtr &msg) {
    Isometry3d iso;
    tf::poseMsgToEigen(*msg, iso);
    world_H_base = iso.matrix();
}

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
                        if (link_name == "world")
                            cable.viaPoints.push_back(ViaPointPtr(new ViaPoint(link_name, local_coordinates, true)));
                        else
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