#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>

using namespace std;

class RoboyIcecream: public cardsflow::kindyn::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    RoboyIcecream(string urdf, string cardsflow_xml){
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "roboy_icecream");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        for(auto ef:endeffectors) {
            motor_control_mode[ef] = nh->serviceClient<roboy_middleware_msgs::ControlMode>(
                    "/roboy/" + ef + "/middleware/ControlMode");
            nh->getParam((ef+"/joints").c_str(),endeffector_jointnames[ef]);
        }
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        init(urdf,cardsflow_xml,joint_names);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);
        roboy_middleware_msgs::ControlMode msg;
        msg.request.control_mode = POSITION;
        msg.request.set_point = 0;
        for(auto control:motor_control_mode){
            if(!control.second.call(msg))
                ROS_WARN("failed to change control mode to position");
        }
        update();

//        for(auto ef:endeffectors) {
//            for(int i=0;i<sim_motors[ef].size();i++)
//                l_offset[ef][i] = l[sim_motors[ef][i]];
//        }
    };
    /**
     * Updates the robot model and integrates the robot model using the forwardKinematics function
     * with a small step length
     */
    void read(){
        update();
        if(!external_robot_state) {
            forwardKinematics(0.0001);
        }
        else {
            for(int i = 0; i<endeffectors.size();i++) {
                int dof_offset = endeffector_dof_offset[i];
                Ld[i].setZero();
                for (int j = dof_offset; j < endeffector_number_of_dofs[i] + dof_offset; j++) {
                    Ld[i] -= ld[j];
                }
            }
        }
    };
    /**
     * Sends motor commands to the real robot
     */
    void write(){
        stringstream str;
        for(int e=0; e<endeffectors.size(); e++) {
            auto _q = q.segment(endeffector_dof_offset[e], endeffector_number_of_dofs[e]);
            auto _q_target = q_target.segment(endeffector_dof_offset[e], endeffector_number_of_dofs[e]);

            double coef = sqrt((_q-_q_target).norm())/10.0;
//            str << "coef: " << coef;

            string ef = endeffectors[e];
//            if (ef != "head") {
//                continue;
//            }
            str << ef << ": ";
//            str << "ld: " << Ld[0].segment(sim_motors[ef][0], sim_motors[ef].size()).transpose().format(fmt) << endl;
            roboy_middleware_msgs::MotorCommand msg;
            msg.id = bodyPartIDs[ef];
            for (int i = 0; i < sim_motors[ef].size(); i++) {
                msg.motors.push_back(sim_motors[ef][i]);
                msg.set_points.push_back(coef*myoMuscleEncoderTicksPerMeter(Ld[0][sim_motors[ef][i]]));
//                str << Ld[e].transpose().format(fmt);
            }
            str << endl;
            motor_command.publish(msg);
        }
//        ROS_INFO_STREAM_THROTTLE(1, str.str());
    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::ServiceClient motor_config, sphere_left_axis0_params, sphere_left_axis1_params, sphere_left_axis2_params;
    map<string,ros::ServiceClient> motor_control_mode;
    vector<string> endeffectors = {"head", "shoulder_left"};//, "shoulder_right", "shoulder_left"}; //"head", "shoulder_left", "shoulder_right",
    map<string, vector<string>> endeffector_jointnames;
    bool external_robot_state; /// indicates if we get the robot state externally
    map<string,vector<short unsigned int>> motors = {
            {"head",{9,10,11,12,13,14}},
            {"shoulder_left",{0,1,2,3,4,5,6,7,8,9,10}},
            {"shoulder_right",{0,1,2,3,4,5,6,7,8,9,11}},
            {"spine_right",{9,10,11,12,13,14}}
    };
    map<string,vector<short unsigned int>> sim_motors = {
            {"head",{18,19,20,21,22,23}},
            {"shoulder_left",{9,10,11,12,13,14,15,16,17,26,27}},
            {"shoulder_right",{0,1,2,3,4,5,6,7,8,24,25}},
            {"spine_right",{11,10,13,14,12,9}}
    };
    map<string,vector<double>> l_offset = {
            {"head",{0,0,0,0,0,0}},
            {"shoulder_left",{0,0,0,0,0,0,0,0,0,0,0,0}},
            {"shoulder_right",{0,0,0,0,0,0,0,0,0,0,0,0}},
            {"spine_right",{0,0,0,0,0,0}}
    };
};

/**
 * controller manager update thread. Here you can define how fast your controllers should run
 * @param cm pointer to the controller manager
 */
void update(controller_manager::ControllerManager *cm) {
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(300); // changing this value affects the control speed of your running controllers
    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        cm->update(time, period);
        prev_time = time;
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "cardsflow_example_robot");
    }
    ros::NodeHandle nh;
    string urdf, cardsflow_xml;
    if(nh.hasParam("urdf_file_path") && nh.hasParam("cardsflow_xml")) {
        nh.getParam("urdf_file_path", urdf);
        nh.getParam("cardsflow_xml", cardsflow_xml);
    }else {
        ROS_FATAL("USAGE: rosrun kindyn test_robot path_to_urdf path_to_viapoints_xml");
        return 1;
    }
    ROS_INFO("\nurdf file path: %s\ncardsflow_xml %s", urdf.c_str(), cardsflow_xml.c_str());

    RoboyIcecream robot(urdf, cardsflow_xml);

    controller_manager::ControllerManager cm(&robot);

    // we need an additional update thread, otherwise the controllers won't switch
    thread update_thread(update, &cm);
    update_thread.detach();

    ROS_INFO("STARTING ROBOT MAIN LOOP...");

    while(ros::ok()){
        robot.read();
        robot.write();
        ros::spinOnce();
    }

    ROS_INFO("TERMINATING...");

    update_thread.join();

    return 0;
}
