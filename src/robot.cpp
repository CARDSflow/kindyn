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
    robot_state = nh->advertise<geometry_msgs::PoseStamped>("/robot_state",1);
    tendon_state = nh->advertise<roboy_communication_simulation::Tendon>("/tendon_state",1);
    controller_type_sub = nh->subscribe("/controller_type",100, &Robot::controllerType, this);
    fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");
}

Robot::~Robot() {
    delete[] H;
    delete[] g;
    delete[] A;
    delete[] lb;
    delete[] ub;
    delete[] b;
    delete[] FOpt;
    delete[] link_to_link_transform;
}

void Robot::init(string urdf_file_path, string viapoints_file_path) {
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

    l.resize(number_of_cables);
    Ld.resize(number_of_cables);
    Ld.setZero();
    ld.resize(number_of_dofs);
    l.setZero();
    for(int i=0;i<number_of_dofs;i++){
        ld[i].resize(number_of_cables);
        ld[i].setZero();
    }

    cable_forces.resize(number_of_cables);
    cable_forces.setZero();
    torques.resize(number_of_dofs);
    torques.setZero();

    controller_type.resize(number_of_cables,0);
    joint_state.resize(number_of_dofs);
    motor_state.resize(number_of_cables);
    // ros control
    for (int joint = 0; joint < number_of_dofs; joint++) {
        // connect and register the cardsflow state interface
        hardware_interface::CardsflowStateHandle state_handle(joint_names[joint], joint, &q[joint], &qd[joint], &qdd[joint], &L, &M, &CG

        );
        cardsflow_state_interface.registerHandle(state_handle);

        // connect and register the cardsflow command interface
        hardware_interface::CardsflowHandle pos_handle(cardsflow_state_interface.getHandle(joint_names[joint]),
                                                       &q_target[joint], &qd_target[joint], &ld[joint]);
        cardsflow_command_interface.registerHandle(pos_handle);
        // connect and register the cardsflow state interface
        hardware_interface::JointStateHandle state_handle2(joint_names[joint], &q[joint], &qd[joint], &q_target[joint]);
        joint_state_interface.registerHandle(state_handle2);

        // connect and register the cardsflow command interface
        hardware_interface::JointHandle torque_handle(joint_state_interface.getHandle(joint_names[joint]), &torques[joint]);
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
    link_to_world_transform.resize(number_of_links);
    frame_transform.resize(number_of_links);
    link_to_link_transform = new Matrix3d[number_of_links*number_of_links];

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

    last_visualization = ros::Time::now();
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
    return f_opt;
}

void Robot::update() {
    ros::Time t0 = ros::Time::now();
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
        link_to_world_transform[i] = pose;
        frame_transform[i] = iDynTree::toEigen(model.getFrameTransform(i).asHomogeneousTransform());
    }
    P.setZero(6 * number_of_links, 6 * number_of_links);
    P.block(0, 0, 6, 6).setIdentity(6, 6);
    for(int k = 1; k<number_of_links; k++){
        for(int a= 1; a<=k; a++) {
            link_to_link_transform[k*number_of_links+a] = world_to_link_transform[k].block(0, 0, 3, 3) * world_to_link_transform[a].block(0, 0, 3, 3);
            P.block(6 * k + 3, 6 * a + 3, 3, 3) = link_to_link_transform[k*number_of_links+a];
        }
    }

    for (auto muscle:cables) {
        for (auto vp:muscle.viaPoints) {
            vp->global_coordinates = link_to_world_transform[vp->link_index].block(0, 3, 3, 1) +
                    link_to_world_transform[vp->link_index].block(0, 0, 3, 3) * vp->local_coordinates;
        }
    }
    ROS_INFO_THROTTLE(1,"model update takes %f seconds", (ros::Time::now()-t0).toSec());
    t0 = ros::Time::now();
    update_V();
    ROS_INFO_THROTTLE(1,"update V takes %f seconds", (ros::Time::now()-t0).toSec());
    t0 = ros::Time::now();
    update_P();
    ROS_INFO_THROTTLE(1,"update P takes %f seconds", (ros::Time::now()-t0).toSec());

    W = P * S;
    L = V * W;
    L_t = -L.transpose();

    if(1.0/(ros::Time::now()-last_visualization).toSec()<30) {
        for (int i = 0; i < number_of_cables; i++) {
            roboy_communication_simulation::Tendon msg;
            msg.name = cables[i].name;
            msg.force = 0;
            msg.l = l[i];
            msg.ld = Ld[i];
            for (auto vp:cables[i].viaPoints) {
                geometry_msgs::Vector3 VP;
                tf::vectorEigenToMsg(vp->global_coordinates, VP);
                msg.viaPoints.push_back(VP);
            }
            tendon_state.publish(msg);
        }
        static int seq = 0;
        for (int i = 0; i < number_of_links; i++) {
            geometry_msgs::PoseStamped msg;
            msg.header.seq = seq;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = link_names[i];
            Isometry3d iso(world_to_link_transform[i].inverse());
            tf::poseEigenToMsg(iso, msg.pose);
            robot_state.publish(msg);
        }
    }
}

void Robot::forwardKinematics(double dt) {
    const iDynTree::Model &model = kinDynComp.model();

//    qdd = M.block(6, 6, number_of_dofs, number_of_dofs).inverse() * (-L.transpose() * cable_forces + CG);
//    qd += qdd * dt;
//    q += qd * dt;
//    auto torque_control_claims = joint_state_interface.getClaims();
//    auto length_control_claims = cardsflow_command_interface.getClaims();

    qdd = M.block(6, 6, number_of_dofs, number_of_dofs).inverse() * (-torques - CG);

    Ld.setZero();
    for(int i=0; i<number_of_dofs; i++) {
        Ld -= ld[i];
    }
    VectorXd qd_temp = EigenExtension::Pinv(L) * Ld;

    for(int j=0;j<joint_names.size();j++){
        if(controller_type[j]==0){
            boost::numeric::odeint::integrate(
                    [this,j](const state_type &x, state_type &dxdt, double t) {
                        dxdt[1] = qdd[j];
                        dxdt[0] = x[1];
                    }, joint_state[j], integration_time, integration_time + dt, dt);
//            qd[j] += qdd[j] * dt;
//            q[j] += qd[j] * dt;
        }
        if(controller_type[j]==2){
            boost::numeric::odeint::integrate(
                    [this,j,qd_temp](const state_type &x, state_type &dxdt, double t) {
                        dxdt[1] = 0;
                        dxdt[0] = qd_temp[j];
                    }, joint_state[j], integration_time, integration_time + dt, dt);
//            qd[j] = qd_temp[j];
//            q[j] += qd_temp[j] * dt;
        }
        qd[j] = joint_state[j][1];
        q[j] = joint_state[j][0];
//        ROS_INFO("%s control type %d", joint_names[j].c_str(), controller_type[j]);
    }
    for (int i = 0; i < number_of_cables; i++) {
        boost::numeric::odeint::integrate(
                [this,i](const state_type &x, state_type &dxdt, double t) {
                    dxdt[1] = 0;
                    dxdt[0] = Ld[i];
                }, motor_state[i], integration_time, integration_time + dt, dt);
        l[i] = motor_state[i][0];
    }
//    l += Ld * dt;
    integration_time += dt;

//    ROS_INFO_STREAM_THROTTLE(1, "M " << M.format(fmt));
//    ROS_INFO_STREAM_THROTTLE(1, "C+G " << CG.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "q_target " << q_target.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "qdd " << qdd.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "qd " << qd.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "q " << q.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "l " << l.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "ld " << Ld.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "torques " << torques.transpose().format(fmt));
    ROS_INFO_STREAM_THROTTLE(5, "cable_forces " << cable_forces.transpose().format(fmt));
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
                    Vector3d V_ijk_T =  world_to_link_transform[k].block(0, 0, 3, 3) * segmentVector;

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
    int k = 1, j = 0;
    for (auto &joint:joint_names) {
        S.block(6 * k, k - 1, 6, 1) = joint_axis[j];
        k++;
        j++;
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
            P.block(6 * k, 6 * a, 3, 3) = link_to_link_transform[k*number_of_links+a] * R_pe.transpose();
            Vector3d v = -r_OP + link_to_link_transform[k*number_of_links+a].transpose() * r_OG;
            skew <<  0,   -v(2),  v(1),
                    v(2),    0, -v(0),
                    -v(1), v(0), 0;
            P.block(6 * k, 6 * a + 3, 3, 3) = -link_to_link_transform[k*number_of_links+a] * skew;
        }
    }

    counter++;
}

void Robot::controllerType(const roboy_communication_simulation::ControllerStateConstPtr &msg){
    for(int i=0;i<joint_names.size();i++) {
        if (joint_names[i] == msg->joint_name)
            controller_type[i] = msg->type;
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