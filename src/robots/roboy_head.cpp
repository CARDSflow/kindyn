#include "kindyn/robot.hpp"
#include <thread>
#include <std_msgs/Float32.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorConfig.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
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
        sphere_head_axis0 = nh->advertise<std_msgs::Float32>("/sphere_head_axis0/sphere_head_axis0/target",1);
        sphere_head_axis1 = nh->advertise<std_msgs::Float32>("/sphere_head_axis1/sphere_head_axis1/target",1);
        sphere_head_axis2 = nh->advertise<std_msgs::Float32>("/sphere_head_axis2/sphere_head_axis2/target",1);
        // face_coordinates = nh->subscribe("/roboy/cognition/vision/face_coordinates",1, &RoboyHead::FaceCoordinates, this);
        motor_status = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyHead::MotorStatus, this);
        for(auto ef:endeffectors) {
            motor_control_mode[ef] = nh->serviceClient<roboy_middleware_msgs::ControlMode>(
                    "/roboy/" + ef + "/middleware/ControlMode");
            motor_config[ef] = nh->serviceClient<roboy_middleware_msgs::MotorConfigService>(
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
        if(!external_robot_state) {
            for (auto ef:endeffectors) {
                roboy_middleware_msgs::MotorConfigService msg;
                for (int i = 0; i < sim_motors[ef].size(); i++) {
                    msg.request.config.motors.push_back(motors[ef][i]);
                    msg.request.config.control_mode.push_back(POSITION);
                    msg.request.config.output_pos_max.push_back(300);
                    msg.request.config.output_neg_max.push_back(-300);
                    msg.request.config.sp_pos_max.push_back(100000);
                    msg.request.config.sp_neg_max.push_back(-100000);
                    msg.request.config.kp.push_back(1);
                    msg.request.config.kd.push_back(0);
                    msg.request.config.ki.push_back(0);
                    msg.request.config.forward_gain.push_back(0);
                    msg.request.config.dead_band.push_back(0);
                    msg.request.config.integral_pos_max.push_back(0);
                    msg.request.config.integral_neg_max.push_back(0);
                    msg.request.config.output_divider.push_back(5);
                    msg.request.config.setpoint.push_back(encoder_offset[ef][i]);
                    l_offset[ef][i] += l[sim_motors[ef][i]];
                    motor_config[ef].call(msg);
                }
            }
        }else{
            for (auto ef:endeffectors) {
                roboy_middleware_msgs::MotorConfigService msg;
                for (int i = 0; i < sim_motors[ef].size(); i++) {
                    msg.request.config.motors.push_back(motors[ef][i]);
                    msg.request.config.control_mode.push_back(DIRECT_PWM);
                    msg.request.config.output_pos_max.push_back(1000);
                    msg.request.config.output_neg_max.push_back(-100);
                    msg.request.config.sp_pos_max.push_back(100000);
                    msg.request.config.sp_neg_max.push_back(-100000);
                    msg.request.config.kp.push_back(1);
                    msg.request.config.kd.push_back(0);
                    msg.request.config.ki.push_back(0);
                    msg.request.config.forward_gain.push_back(0);
                    msg.request.config.dead_band.push_back(0);
                    msg.request.config.integral_pos_max.push_back(0);
                    msg.request.config.integral_neg_max.push_back(0);
                    msg.request.config.output_divider.push_back(5);
                    msg.request.config.setpoint.push_back(0);
                    l_offset[ef][i] += l[sim_motors[ef][i]];
                    motor_config[ef].call(msg);
                }
            }
        }
        roll.data = 0;
        pitch.data = 0;
        yaw.data = 0;
    };

    void FaceCoordinates(const geometry_msgs::Point::ConstPtr &msg){
        float gain;
        nh->getParam("/face_coordinate/gain", gain);
        pitch.data -= gain*msg->y;
        yaw.data -= gain*msg->x;
        if(pitch.data>=pitch_max)
            pitch.data = pitch_max;
        if(pitch.data<=pitch_min)
            pitch.data = pitch_min;
        if(yaw.data>=yaw_max)
            yaw.data = yaw_max;
        if(yaw.data<=yaw_min)
            yaw.data = yaw_min;
        sphere_head_axis1.publish(pitch);
        sphere_head_axis2.publish(yaw);
    }

    void MotorStatus( const roboy_middleware_msgs::MotorStatus::ConstPtr &msg){
        if(msg->id != SHOULDER_RIGHT)
            return;
        int j = 0;
        for(int i=9;i<15;i++){
            encoder_offset["shoulder_right"][j]=((msg->position[i]+3000));
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
        if(!external_robot_state) {
            stringstream str;
            for (auto ef:endeffectors) {
                str << ef << ": ";
                roboy_middleware_msgs::MotorCommand msg;
                msg.id = bodyPartIDs[ef];
                msg.motors = motors[ef];
                for (int i = 0; i < sim_motors[ef].size(); i++) {
                    double l_meter = l_offset[ef][i] - l[sim_motors[ef][i]];
                    str << l_meter << "\t";
                    switch (motor_type[msg.id][i]) {
                        case MYOBRICK100N: {
                            msg.set_points.push_back(myoBrick100NEncoderTicksPerMeter(l_meter) + encoder_offset[ef][i]);
                            break;
                        }
                        case MYOBRICK300N: {
                            msg.set_points.push_back(myoBrick300NEncoderTicksPerMeter(l_meter) + encoder_offset[ef][i]);
                            break;
                        }
                        case MYOMUSCLE500N: {
                            msg.set_points.push_back(myoMuscleEncoderTicksPerMeter(l_meter) + encoder_offset[ef][i]);
                            break;
                        }
                    }
                }
                str << endl;
                motor_command.publish(msg);
            }
            ROS_INFO_STREAM_THROTTLE(1, str.str());
        }else{
            stringstream str;
            float Kp, pretension;
            nh->getParam("Kp_pwm", Kp);
            nh->getParam("pretension", pretension);
            for(auto ef:endeffectors) {
                roboy_middleware_msgs::MotorCommand msg;
                msg.id = bodyPartIDs[ef];
                msg.motors = motors[ef];
                for (int i = 0; i < sim_motors[ef].size(); i++) {
                    switch (motor_type[msg.id][motors[ef][i]]) {
                        case MYOBRICK100N: {
                            l_change[ef][i] += Kp*(l_target[i]-l[i]);
                            if(l_change[ef][i]>1000)
                                l_change[ef][i] = 1000;
                            if(l_change[ef][i]<pretension)
                                l_change[ef][i] = pretension;
                            break;
                        }
                        case MYOBRICK300N: {
                            l_change[ef][i] += 0.3*Kp*(l_target[i]-l[i]);
                            if(l_change[ef][i]>500)
                                l_change[ef][i] = 500;
                            if(l_change[ef][i]<0.3*pretension)
                                l_change[ef][i] = 0.3*pretension;
                            break;
                        }
                    }

                    msg.set_points.push_back(l_change[ef][i]);
                    str << l_change[ef][i] << "\t";
                }
                ROS_INFO_STREAM_THROTTLE(1, str.str());
                motor_command.publish(msg);
            }
        }
    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command, sphere_head_axis0, sphere_head_axis1, sphere_head_axis2; /// motor command publisher
    ros::Subscriber motor_status, face_coordinates;
    ros::ServiceClient sphere_left_axis0_params, sphere_left_axis1_params, sphere_left_axis2_params;
    map<string,ros::ServiceClient> motor_control_mode, motor_config;
    vector<string> endeffectors = {"shoulder_right"}; //"head", "shoulder_left", "shoulder_right",
    map<string, vector<string>> endeffector_jointnames;
    bool external_robot_state; /// indicates if we get the robot state externally
    bool status_received = false;
    std_msgs::Float32 roll, pitch, yaw;
    float err_x = 0, error_y = 0, pitch_max = 0.33, pitch_min = -0.50, yaw_min = -0.50, yaw_max = 0;
//    map<string,vector<int>> motors = {
//            {"head",{9,10,11,12,13,14}},
//            {"shoulder_left",{0,1,2,3,4,5,6,7,8,9,10}},
//            {"shoulder_right",{0,1,2,3,4,5,6,7,8,9,11}},
//            {"spine_right",{9,10,11,12,13,14}}
//    };
    map<string,vector<int>> motors = {
            {"shoulder_right",{9,10,11,12,13,14}}
    };
//    map<string,vector<int>> sim_motors = {
//            {"head",{36,37,35,34,32,33}},
//            {"shoulder_left",{0,1,2,3,4,5,6,7,8,9,10}},
//            {"shoulder_right",{0,1,2,3,4,5,6,7,8,9,11}},
//            {"spine_right",{11,10,13,14,12,9}}
//    };
    map<string,vector<int>> sim_motors = {
            {"shoulder_right",{0,1,2,3,4,5}}
    };

    map<string,vector<float>> l_change = {
            {"shoulder_right",{0,0,0,0,0,0}}
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
