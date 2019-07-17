#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>
#include <std_srvs/Empty.h>

#define myoMuscleMeterPerEncoderTick(encoderTicks) ((encoderTicks)/(2096.0*53.0)*(2.0*M_PI*0.0055))
#define myoMuscleEncoderTicksPerMeter(meter) ((meter)*(2096.0*53.0)/(2.0*M_PI*0.0055))

using namespace std;

class RoboyIcecream: public vrpuppet::kindyn::Robot{
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
        for(auto part:body_parts) {
            motor_control_mode[part] = nh->serviceClient<roboy_middleware_msgs::ControlMode>(
                    "/roboy/" + part + "/middleware/ControlMode");
            motor_config[part] = nh->serviceClient<roboy_middleware_msgs::MotorConfigService>(
                    "/roboy/" + part + "/middleware/MotorConfig");
            nh->getParam((part+"/joints"),endeffector_jointnames[part]);
            string mode;
            nh->getParam((part+"/init_mode"),mode);
            if(mode=="POSITION") {
                init_mode[part] = POSITION;
                ROS_INFO("init mode for %s POSITION", part.c_str());
            }else if(mode=="VELOCITY") {
                init_mode[part] = VELOCITY;
                ROS_INFO("init mode for %s VELOCITY", part.c_str());
            }else if(mode=="DISPLACEMENT") {
                init_mode[part] = DISPLACEMENT;
                ROS_INFO("init mode for %s DISPLACEMENT", part.c_str());
            }else if(mode=="DIRECT_PWM") {
                init_mode[part] = DIRECT_PWM;
                ROS_INFO("init mode for %s DIRECT_PWM", part.c_str());
            }
            nh->getParam((part+"/real_motor_ids"),real_motor_ids[part]);
            nh->getParam((part+"/sim_motor_ids"),sim_motor_ids[part]);
            nh->getParam((part+"/init_setpoint"),init_setpoint[part]);
            nh->getParam((part+"/init_mode"),init_mode[part]);
            nh->getParam((part+"/motor_type"),motor_type[part]);
            l_offset[part].resize(real_motor_ids[part].size(),0);
            position[part].resize(real_motor_ids[part].size(),0);
            velocity[part].resize(real_motor_ids[part].size(),0);
            displacement[part].resize(real_motor_ids[part].size(),0);
            nh->getParam((part+"/bodyPartID"),bodyPartIDs[part]);
        }
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        init(urdf,cardsflow_xml,joint_names);
        nh->getParam("external_robot_state", external_robot_state);
        update();
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        motor_status_sub = nh->subscribe("roboy/middleware/MotorStatus",1,&RoboyIcecream::MotorStatus, this);
        init_pose = nh->advertiseService("init_pose",&RoboyIcecream::initPose,this);
    };

    bool initPose(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res){
        initialized = false;
         ROS_INFO("changing control mode to init mode for each part");
         for(auto part:body_parts){
             roboy_middleware_msgs::MotorConfigService msg;
             for (int i = 0; i < sim_motor_ids[part].size(); i++) {
                 msg.request.config.motors.push_back(real_motor_ids[part][i]);
                 msg.request.config.control_mode.push_back(init_mode[part]);
                 msg.request.config.output_pos_max.push_back(500);
                 msg.request.config.output_neg_max.push_back(-500);
                 msg.request.config.sp_pos_max.push_back(100000);
                 msg.request.config.sp_neg_max.push_back(-100000);
                 msg.request.config.kp.push_back(100);
                 msg.request.config.kd.push_back(0);
                 msg.request.config.ki.push_back(0);
                 msg.request.config.forward_gain.push_back(0);
                 msg.request.config.dead_band.push_back(0);
                 msg.request.config.integral_pos_max.push_back(0);
                 msg.request.config.integral_neg_max.push_back(0);
                 msg.request.config.output_divider.push_back(0);
                 msg.request.config.setpoint.push_back(init_setpoint[part]);
                 motor_config[part].call(msg);
             }
         }
         ros::Duration d(5);
         ROS_INFO("sleeping for 5 seconds");
         d.sleep();
        while(!motor_status_received[0] || !motor_status_received[1])
            ROS_INFO_THROTTLE(1,"waiting to receive motor status from both fpgas");
        stringstream str;
        str << "saving position offsets:" << endl << "sim motor id   |  real motor id  |   position offset (ticks)  | length offset(m)" << endl;
        for(auto part:body_parts) {
            for(int i=0;i<sim_motor_ids[part].size();i++) {
                l_offset[part][i] = l[sim_motor_ids[part][i]] + myoMuscleMeterPerEncoderTick(position[part][i]);
                str << sim_motor_ids[part][i] << "\t|\t" << real_motor_ids[part][i] << "\t|\t" << position[part][i] << "\t|\t" << l_offset[part][i] << endl;
            }
        }
        ROS_INFO_STREAM(str.str());
         ROS_INFO("changing control mode to POSITION");
         for(auto part:body_parts){
             roboy_middleware_msgs::MotorConfigService msg;
             for (int i = 0; i < sim_motor_ids[part].size(); i++) {
                 msg.request.config.motors.push_back(real_motor_ids[part][i]);
                 msg.request.config.control_mode.push_back(POSITION);
                 msg.request.config.output_pos_max.push_back(500);
                 msg.request.config.output_neg_max.push_back(-500);
                 msg.request.config.sp_pos_max.push_back(1000000);
                 msg.request.config.sp_neg_max.push_back(-1000000);
                 msg.request.config.kp.push_back(1);
                 msg.request.config.kd.push_back(0);
                 msg.request.config.ki.push_back(0);
                 msg.request.config.forward_gain.push_back(0);
                 msg.request.config.dead_band.push_back(0);
                 msg.request.config.integral_pos_max.push_back(0);
                 msg.request.config.integral_neg_max.push_back(0);
                 msg.request.config.output_divider.push_back(5);
                 msg.request.config.setpoint.push_back(position[part][i]);
             }
             motor_config[part].call(msg);
         }
         ROS_INFO("pose init done");
         initialized = true;
        return true;
    }

    void MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg){
        if(msg->id==3){
            int j = 0;
            if(std::find(body_parts.begin(), body_parts.end(), "shoulder_left") != body_parts.end()) {
                for(int i=0;i<9;i++) {
                    position["shoulder_left"][j] = msg->position[i];
                    velocity["shoulder_left"][j] = msg->velocity[i];
                    displacement["shoulder_left"][j] = msg->displacement[i];
                    j++;
                }
            }
            j = 0;
            if(std::find(body_parts.begin(), body_parts.end(), "arms") != body_parts.end()) {
                for (int i = 9; i < 15; i++) {
                    position["arms"][j] = msg->position[i];
                    velocity["arms"][j] = msg->velocity[i];
                    displacement["arms"][j] = msg->displacement[i];
                    j++;
                }
            }
            j = 0;
            if(std::find(body_parts.begin(), body_parts.end(), "leg_left") != body_parts.end()) {
                for (int i = 15; i < 21; i++) {
                    position["leg_left"][j] = msg->position[i];
                    velocity["leg_left"][j] = msg->velocity[i];
                    displacement["leg_left"][j] = msg->displacement[i];
                    j++;
                }
            }
            motor_status_received[0] = true;
        }else if(msg->id == 4){
            int j = 0;
            if(std::find(body_parts.begin(), body_parts.end(), "shoulder_right") != body_parts.end()) {
                for (int i = 0; i < 9; i++) {
                    position["shoulder_right"][j] = msg->position[i];
                    velocity["shoulder_right"][j] = msg->velocity[i];
                    displacement["shoulder_right"][j] = msg->displacement[i];
                    j++;
                }
            }
            j = 0;
            if(std::find(body_parts.begin(), body_parts.end(), "head") != body_parts.end()) {
                for (int i = 9; i < 15; i++) {
                    position["head"][j] = msg->position[i];
                    velocity["head"][j] = msg->velocity[i];
                    displacement["head"][j] = msg->displacement[i];
                    j++;
                }
            }
            j = 0;
            if(std::find(body_parts.begin(), body_parts.end(), "leg_right") != body_parts.end()) {
                for (int i = 15; i < 21; i++) {
                    position["leg_right"][j] = msg->position[i];
                    velocity["leg_right"][j] = msg->velocity[i];
                    displacement["leg_right"][j] = msg->displacement[i];
                    j++;
                }
            }
            motor_status_received[1] = true;
        }
    }

    /**
     * Updates the robot model
     */
    void read(){
        update();
    };
    /**
     * Sends motor commands to the real robot
     */
    void write(){
         if(initialized) {
             stringstream str;
             for (auto part:body_parts) {
                 str << part << ": ";
                 roboy_middleware_msgs::MotorCommand msg;
                 msg.id = bodyPartIDs[part];
                 msg.motors = real_motor_ids[part];
                 for (int i = 0; i < sim_motor_ids[part].size(); i++) {
                     double l_meter = (-l[sim_motor_ids[part][i]] + l_offset[part][i]);
                     str << sim_motor_ids[part][i] << "\t" << l_meter << "\t";
                     switch (motor_type[part][i]) {
                         case MYOBRICK100N: {
                             msg.set_points.push_back(myoBrick100NEncoderTicksPerMeter(l_meter));
                             break;
                         }
                         case MYOBRICK300N: {
                             msg.set_points.push_back(myoBrick300NEncoderTicksPerMeter(l_meter));
                             break;
                         }
                         case MYOMUSCLE500N: {
                             str << myoMuscleEncoderTicksPerMeter(l_meter) << "\t";
                             msg.set_points.push_back(myoMuscleEncoderTicksPerMeter(l_meter));
                             break;
                         }
                     }
                 }
                 str << endl;
                 motor_command.publish(msg);
             }
             ROS_INFO_STREAM_THROTTLE(1, str.str());
         }else{
             ROS_INFO_THROTTLE(5,"waiting for initialisation, call /init_pose service!!!");
         }
    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::Subscriber motor_status_sub;
    ros::ServiceServer init_pose;
    map<string,ros::ServiceClient> motor_control_mode, motor_config;
//    vector<string> body_parts = {"head","shoulder_left", "shoulder_right", "arms"};
    vector<string> body_parts = {"shoulder_right", "shoulder_left"};
    map<string, vector<string>> endeffector_jointnames;
    bool external_robot_state; /// indicates if we get the robot state externally
    bool initialized = false, motor_status_received[2] = {false,false};
    map<string, int> init_mode, init_setpoint;
    map<string,vector<int>> real_motor_ids, sim_motor_ids, motor_type;
    map<string,vector<double>> l_offset, position, velocity, displacement;
    map<string,int> bodyPartIDs;
};

/**
 * controller manager update thread. Here you can define how fast your controllers should run
 * @param cm pointer to the controller manager
 */
void update(controller_manager::ControllerManager *cm) {
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10); // changing this value affects the control speed of your running controllers
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
        ros::init(argc, argv, "VRpuppet");
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
