//
// Created by roboy on 01.07.21.
//

#include "kindyn/kinematics.hpp"

using namespace cardsflow::kindyn;

Kinematics::Kinematics(){

}

Kinematics::~Kinematics(){

};

void Kinematics::init(string urdf_file_path, string viapoints_file_path, vector <string> joint_names_ordered) {

    /**
     * Init model by iDynTree
     */

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

    kinDynCompTarget.loadRobotModel(mdlLoader.model());

    /**
     * Extract robot info from model
     */
    const iDynTree::Model &model = kinDynComp.model();
    number_of_dofs = model.getNrOfDOFs();
    number_of_joints = model.getNrOfJoints();
    number_of_links = model.getNrOfLinks();

    /**
     * Init cables
     */

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

    /**
     * Resize variables
     */

    q.resize(number_of_dofs);
    qd.resize(number_of_dofs);
    qdd.resize(number_of_dofs);
    q_next.resize(number_of_dofs);
    qd_next.resize(number_of_dofs);
    qdd_next.resize(number_of_dofs);

    joint_state.resize(number_of_dofs);
    q_min.resize(number_of_dofs);
    q_max.resize(number_of_dofs);

    /**
     * Set robot state to initial pose
     */
    robotstate.resize(number_of_dofs);
    baseVel.setZero();
    world_H_base.setIdentity();
    gravity << 0, 0, -9.81;

    M.resize(number_of_dofs + 6, number_of_dofs + 6);
    CG.resize(number_of_dofs);
    CG.setZero();
    torques.resize(number_of_dofs);
    torques.setZero();
    cable_forces.resize(number_of_cables);
    cable_forces.setZero();

    q.setZero();
    qd.setZero();
    qdd.setZero();
    q_next.setZero();
    qd_next.setZero();
    qdd_next.setZero();

    /**
     * Init joint info
     */
    for (int joint = 0; joint < number_of_dofs; joint++) {
        joint_state[joint][0] = 0;
        joint_state[joint][1] = 0;
        q_min[joint] = model.getJoint(joint)->getMinPosLimit(0);
        q_max[joint] = model.getJoint(joint)->getMaxPosLimit(0);
    }

    /**
     * Init pose info
     */
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

    /**
     * Init cable info
     */
    segments.resize(number_of_cables);

    int muscle_index = 0;
    for (auto muscle:cables) {
        muscle.viaPoints[0]->link_index = model.getLinkIndex(muscle.viaPoints[0]->link_name);
        for (int i = 1; i < muscle.viaPoints.size(); i++) {
            muscle.viaPoints[i]->link_index = link_index[muscle.viaPoints[i]->link_name];
            pair<ViaPointPtr, ViaPointPtr> segment(muscle.viaPoints[i - 1], muscle.viaPoints[i]);
            segments[muscle_index].push_back(segment);
        }
        muscle_index++;
    }

    /**
     * Init Jacobians
     */
    update_S();
}

void Kinematics::setRobotState(VectorXd q_in, VectorXd qd_in){
    // TODO: Set RobotState in critical section

    // Update variables
    q = q_in;
    qd = qd_in;

    // Update Idyntree and convert from Joint State to Pose
    iDynTree::fromEigen(robotstate.world_H_base, world_H_base);
    iDynTree::toEigen(robotstate.jointPos) = q;
    iDynTree::fromEigen(robotstate.baseVel, baseVel);
    toEigen(robotstate.jointVel) = qd;
    toEigen(robotstate.gravity) = gravity;
    kinDynComp.setRobotState(robotstate.world_H_base, robotstate.jointPos, robotstate.baseVel, robotstate.jointVel,
                             robotstate.gravity);

    const iDynTree::Model &model = kinDynComp.model();
    for (int i = 0; i < number_of_links; i++) {
        Matrix4d pose = iDynTree::toEigen(kinDynComp.getWorldTransform(i).asHomogeneousTransform());
        Vector3d com = iDynTree::toEigen(model.getLink(i)->getInertia().getCenterOfMass());
        pose.block(0, 3, 3, 1) += pose.block(0, 0, 3, 3) * com;
        world_to_link_transform[i] = pose.inverse();
        link_to_world_transform[i] = pose;
        frame_transform[i] = iDynTree::toEigen(model.getFrameTransform(i).asHomogeneousTransform());
    }
}

void Kinematics::updateJacobians(){

    // Update V - Cable-link jacobian
    update_V();

    // Update P - Link-Position jacobian
    update_P();

    // Compute L matrix - Cable-Joint jacobian
    W = P * S;
    L = V * W;
    L_t = -L.transpose();
}

void Kinematics::update_V() {
    static int counter = 0;
    V.setZero(number_of_cables, 6 * number_of_links);
//    #pragma omp parallel for
    for (int muscle_index = 0; muscle_index < cables.size(); muscle_index++) {
        for (auto &segment:segments[muscle_index]) {
            if (segment.first->link_name != segment.second->link_name) { // ignore redundant cables
                // V term associated with segment translation
                Vector3d segmentVector;
                segmentVector = segment.second->global_coordinates - segment.first->global_coordinates;
                segmentVector.normalize();

                int k = segment.second->link_index;
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

void Kinematics::update_S() {
    S.setZero(6 * number_of_links, number_of_dofs);

    int k = 0;
    for (int i=0; i < link_relation_name.size(); i++){

        int link_idx = -1;
        for (int l=0; l < link_names.size(); l++){
            if(link_names[l] == link_relation_name[i]){
                link_idx = l;
                break;
            }
        }

        for (int j=0; j < joint_relation_name[i].size(); j++){
            S.block(6 * link_idx, k, 6, 1) = joint_axis[k];
            k++;
        }
    }
}

void Kinematics::update_P() {
    P.setZero(6 * number_of_links, 6 * number_of_links);
    P.block(0, 0, 6, 6).setIdentity(6, 6);

    Matrix3d R_ka;
    Eigen::Matrix<double, 6, 6> Pak;

    const iDynTree::Model &model = kinDynComp.model();

    static int counter = 0;
//    #pragma omp parallel for
    for (int k = 1; k < number_of_links; k++) {
        if(std::find(link_relation_name.begin(), link_relation_name.end(), link_names[k]) != link_relation_name.end()) {

            Matrix4d transformMatrix_k = world_to_link_transform[k];
            Matrix3d R_k0 = transformMatrix_k.block(0, 0, 3, 3);

            int a = k;
            Matrix4d transformMatrix_a = world_to_link_transform[a];
            Matrix3d R_0a = transformMatrix_a.block(0, 0, 3, 3).transpose();
            R_ka = R_k0 * R_0a;

            Matrix3d R_pe;
            Vector3d r_OP, r_OG;
            r_OP.setZero();

            R_pe = AngleAxisd(q[a - 1], -joint_axis[a - 1].block(3, 0, 3, 1));

            // absolute joint location
            Matrix4d pose = iDynTree::toEigen(kinDynComp.getWorldTransform(a).asHomogeneousTransform());
            r_OP = pose.topRightCorner(3, 1);

            // absolute com location
            r_OG = link_to_world_transform[k].topRightCorner(3, 1);

            Matrix3d PaK_2_1 = EigenExtension::SkewSymmetric2(-r_OP + R_ka.transpose() * r_OG);
            Matrix3d PaK_2 = -R_ka * PaK_2_1;
            Matrix3d PaK_1 = R_ka * R_pe;
            Pak.block(0, 0, 3, 3) = PaK_1;
            Pak.block(0, 3, 3, 3) = PaK_2;
            Pak.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
            Pak.block(3, 3, 3, 3) = R_ka;
            P.block(6 * k, 6 * a, 6, 6) = Pak;
        }
    }
    counter++;
}


vector<VectorXd> Kinematics::oneStepForward(VectorXd& q_in, VectorXd& qd_in, vector<VectorXd> Ld) {

    integration_time = ros::Time::now().toSec();

    for(int i=0;i<number_of_joints;i++){
        joint_state[i][0] = q_in[i];
        joint_state[i][1] = qd_in[i];
    }

    for(int i = 0; i<endeffectors.size();i++) {
        int dof_offset = endeffector_dof_offset[i];
        MatrixXd L_endeffector = L.block(0,dof_offset,number_of_cables,endeffector_number_of_dofs[i]);
//        L_endeffector = L_endeffector + 1e-2 * MatrixXd::Identity(L_endeffector.rows(), L_endeffector.cols());
        MatrixXd L_endeffector_inv = EigenExtension::Pinv(L_endeffector);
        VectorXd qd_temp =  L_endeffector_inv * Ld[i];

        for (int j = dof_offset; j < endeffector_number_of_dofs[i]+dof_offset; j++) {
            boost::numeric::odeint::integrate(
                    [this, j, qd_temp, dof_offset](const state_type &x, state_type &dxdt, double t) {
                        dxdt[1] = 0;
                        dxdt[0] = qd_temp[j-dof_offset];
                    }, joint_state[j], integration_time, integration_time + joint_dt[j], joint_dt[j]);
            qd_next[j] = qd_temp[j-dof_offset];
            q_next[j] = joint_state[j][0];
        }
    }

    // respect joint limits
    for(int i=0;i<number_of_joints;i++){
        if(q_next[i]<q_min[i]){
            q_next[i] = q_min[i];
            qd_next[i] = 0;
        }
        if(q_next[i]>q_max[i]){
            q_next[i] = q_max[i];
            qd_next[i] = 0;
        }
    }

    ROS_INFO_THROTTLE(10, "forward kinematics calculated for %lf s", integration_time);

    vector<VectorXd> result = {q_next, qd_next};
    return result;
}

vector<Matrix4d> Kinematics::getRobotPosesFromJoints(){
    return link_to_world_transform;
}

Matrix4d Kinematics::getPoseFromJoint(int index){
    Matrix4d pose = iDynTree::toEigen(kinDynComp.getWorldTransform(index).asHomogeneousTransform());
    return pose;
}

void Kinematics::getRobotCableFromJoints(VectorXd &l_out){
    int i=0;
    for (auto muscle:cables) {
        l_out[i] = 0;
        int j=0;
        for (auto vp:muscle.viaPoints) {
            if (!vp->fixed_to_world) { // move viapoint with link
                vp->global_coordinates = link_to_world_transform[vp->link_index].block(0, 3, 3, 1) +
                                         link_to_world_transform[vp->link_index].block(0, 0, 3, 3) *
                                         vp->local_coordinates;
            }
            if(j>0){
                l_out[i] += (muscle.viaPoints[j]->global_coordinates-muscle.viaPoints[j-1]->global_coordinates).norm();
            }
            j++;
        }
        i++;
    }
}

int Kinematics::GetJointIdByName(string joint) {
    const iDynTree::Model &model = kinDynComp.getRobotModel();
    int joint_index = model.getJointIndex(joint);
    if (joint_index == iDynTree::JOINT_INVALID_INDEX) {
        ROS_WARN_THROTTLE(5.0, "joint %s not found in model", joint.c_str());
        return -1;
    }
    return joint_index;
}

bool Kinematics::parseViapoints(const string &viapoints_file_path, vector<Cable> &cables) {
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
