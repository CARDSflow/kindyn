#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorConfig.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>

using namespace std;

class RoboyHead: public cardsflow::kindyn::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    RoboyHead(string urdf, string cardsflow_xml){
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "roboy_head");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        motor_status = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyHead::MotorStatus, this);
        for(auto ef:endeffectors) {
            motor_control_mode[ef] = nh->serviceClient<roboy_middleware_msgs::ControlMode>(
                    "/roboy/" + ef + "/middleware/ControlMode");
            motor_config[ef] = nh->serviceClient<roboy_middleware_msgs::ControlMode>(
                    "/roboy/" + ef + "/middleware/MotorConfig");
            nh->getParam((ef+"/joints").c_str(),endeffector_jointnames[ef]);
        }
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        init(urdf,cardsflow_xml,joint_names);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);
        update();
        ros::Rate rate(1);
        while(!status_received && ros::ok()){
            ROS_INFO("waiting for head MotorStatus to be received");
            rate.sleep();
        }
        for(auto ef:endeffectors) {
            roboy_middleware_msgs::MotorConfig msg;
            for(int i=0;i<sim_motors[ef].size();i++) {
                msg.motors.push_back(motors[ef][i]);
                msg.control_mode.push_back(POSITION);
                msg.output_pos_max.push_back(300);
                msg.output_neg_max.push_back(-300);
                msg.sp_pos_max.push_back(100000);
                msg.sp_neg_max.push_back(-100000);
                msg.kp.push_back(1);
                msg.kd.push_back(0);
                msg.ki.push_back(0);
                msg.forward_gain.push_back(0);
                msg.dead_band.push_back(0);
                msg.integral_pos_max.push_back(0);
                msg.integral_neg_max.push_back(0);
                msg.output_divider.push_back(5);
                msg.setpoint.push_back(encoder_offset[ef][i]);
                l_offset[ef][i] += l[sim_motors[ef][i]];
            }
        }
//        roboy_middleware_msgs::ControlMode msg;
//        msg.request.control_mode = POSITION;
//        msg.request.set_point = 0;
//        for(auto control:motor_control_mode){
//            if(!control.second.call(msg))
//                ROS_WARN("failed to change control mode to position");
//        }
    };

    void MotorStatus( const roboy_middleware_msgs::MotorStatus::ConstPtr &msg){
        if(msg->id != SHOULDER_RIGHT)
            return;
        int j = 0;
        for(int i=9;i<15;i++){
            encoder_offset["shoulder_right"][j]=((msg->position[i]));
            j++;
        }
        motor_status.shutdown();
        status_received = true;
    }
    /**
     * Updates the robot model and integrates the robot model using the forwardKinematics function
     * with a small step length
     */
    void read(){
        update();
        if(!external_robot_state)
            forwardKinematics(0.0001);
    };
    /**
     * Sends motor commands to the real robot
     */
    void write(){
        stringstream str;
        for(auto ef:endeffectors) {
            str << ef << ": ";
            roboy_middleware_msgs::MotorCommand msg;
            msg.id = bodyPartIDs[ef];
            msg.motors = motors[ef];
            for (int i = 0; i < sim_motors[ef].size(); i++) {
                double l_meter = l_offset[ef][i] - l[sim_motors[ef][i]];
                str  <<  l_meter << "\t";
                switch(motor_type[msg.id][i]){
                    case MYOBRICK100N:{
                        msg.set_points.push_back(myoBrick100NEncoderTicksPerMeter(l_meter)+encoder_offset[ef][i]);
                        break;
                    }
                    case MYOBRICK300N:{
                        msg.set_points.push_back(myoBrick300NEncoderTicksPerMeter(l_meter)+encoder_offset[ef][i]);
                        break;
                    }
                    case MYOMUSCLE500N:{
                        msg.set_points.push_back(myoMuscleEncoderTicksPerMeter(l_meter)+encoder_offset[ef][i]);
                        break;
                    }
                }
            }
            str << endl;
            motor_command.publish(msg);
        }
        ROS_INFO_STREAM_THROTTLE(1, str.str());
    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::Subscriber motor_status;
    ros::ServiceClient sphere_left_axis0_params, sphere_left_axis1_params, sphere_left_axis2_params;
    map<string,ros::ServiceClient> motor_control_mode, motor_config;
    vector<string> endeffectors = {"shoulder_right"}; //"head", "shoulder_left", "shoulder_right",
    map<string, vector<string>> endeffector_jointnames;
    bool external_robot_state; /// indicates if we get the robot state externally
    bool status_received = false;
//    map<string,vector<short unsigned int>> motors = {
//            {"head",{9,10,11,12,13,14}},
//            {"shoulder_left",{0,1,2,3,4,5,6,7,8,9,10}},
//            {"shoulder_right",{0,1,2,3,4,5,6,7,8,9,11}},
//            {"spine_right",{9,10,11,12,13,14}}
//    };
    map<string,vector<short unsigned int>> motors = {
            {"shoulder_right",{9,10,11,12,13,14}}
    };
//    map<string,vector<short unsigned int>> sim_motors = {
//            {"head",{36,37,35,34,32,33}},
//            {"shoulder_left",{0,1,2,3,4,5,6,7,8,9,10}},
//            {"shoulder_right",{0,1,2,3,4,5,6,7,8,9,11}},
//            {"spine_right",{11,10,13,14,12,9}}
//    };
    map<string,vector<short unsigned int>> sim_motors = {
            {"shoulder_right",{0,1,2,5,4,3}}
    };

    map<string,vector<double>> l_offset = {
            {"head",{0,0,0,0,0,0}},
            {"shoulder_left",{0,0,0,0,0,0,0,0,0,0,0,0}},
            {"shoulder_right",{0,0,0,0,0,0,0,0,0,0,0,0}},
            {"spine_right",{0,0,0,0,0,0}}
    };
    map<string,vector<double>> encoder_offset = {
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
    ros::Rate rate(100); // changing this value affects the control speed of your running controllers
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

    RoboyHead robot(urdf, cardsflow_xml);

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
