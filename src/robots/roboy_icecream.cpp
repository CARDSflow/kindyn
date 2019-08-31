#include "kindyn/vrpuppet.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>
#include <std_srvs/Empty.h>

using namespace std;

class RoboyIcecream: public cardsflow::vrpuppet::Robot{
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
            std::string board_name;
            nh->getParam(part + "/board_name", board_name);
            motor_control_mode[part] = nh->serviceClient<roboy_middleware_msgs::ControlMode>(
                    "/roboy/" + board_name + "/middleware/ControlMode");
            motor_config[part] = nh->serviceClient<roboy_middleware_msgs::MotorConfigService>(
                    "/roboy/" + board_name + "/middleware/MotorConfig");
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
            nh->getParam((part+"/use_motor_config"),use_motor_config[part]);
            l_offset[part].resize(real_motor_ids[part].size(),0);
            position[part].resize(real_motor_ids[part].size(),0);
            velocity[part].resize(real_motor_ids[part].size(),0);
            displacement[part].resize(real_motor_ids[part].size(),0);
            nh->getParam((part+"/bodyPartID"),bodyPartIDs[part]);

            motor_status_received[part] = false;
            ROS_INFO_STREAM("Added body part: " << part);
        }
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        init(urdf,cardsflow_xml,joint_names);
        nh->getParam("external_robot_state", external_robot_state);
        ROS_INFO_STREAM("External robot state: " << external_robot_state);
        update();
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        motor_status_sub = nh->subscribe("roboy/middleware/MotorStatus",1,&RoboyIcecream::MotorStatus, this);
        init_pose = nh->advertiseService("init_pose",&RoboyIcecream::initPose,this);
        ROS_INFO_STREAM("Finished setup");
    };

    bool initPose(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res){
        initialized = false;
        ROS_INFO("changing control mode to init mode for each part");
        for(auto part:body_parts){
          if (use_motor_config[part]) {
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
            }
            motor_config[part].call(msg);
          } else {
            roboy_middleware_msgs::ControlMode msg;
            for (int i = 0; i < sim_motor_ids[part].size(); i++) {
              msg.request.motor_id.push_back(i);
              msg.request.set_point = init_setpoint[part];
              msg.request.control_mode = init_mode[part];
            }
            motor_control_mode[part].call(msg);
          }
        }
        ros::Duration d(5);
        ROS_INFO("sleeping for 5 seconds");
        d.sleep();
        while(std::any_of(motor_status_received.begin(), motor_status_received.end(), [](auto &e){return !e.second;})) {
            stringstream ss; ss << "Waiting to receive motor status from these body parts: ";
            for (const auto &part : body_parts) {
                if (!motor_status_received[part]) ss << part << " ";
            }
            ROS_INFO_STREAM_THROTTLE(0.5, ss.str());
        }
        stringstream str;
        str << "saving position offsets:" << endl << "sim motor id   |  real motor id  |   position offset (ticks)  | length offset(m)" << endl;
        for (const auto &part:body_parts) {
            str << "Positions for body part " << part;
            for (int i = 0; i<sim_motor_ids[part].size();i++) str << i << ": " << position[part][i] << ", ";
            str << endl;
            for(int i=0;i<sim_motor_ids[part].size();i++) {
                double meters = 0.0;
                switch (motor_type[part][i]) {
                    case MYOMUSCLE500N:
                        meters = myoMuscleMeterPerEncoderTick(position[part][i]);
                        break;
                    case MYOBRICK300N:
                        meters = myoBrick300NMeterPerEncoderTick(position[part][i]);
                        break;
                    case MYOBRICK100N:
                        meters = myoBrick100NMeterPerEncoderTick(position[part][i]);
                        break;
                }
                l_offset[part][i] = l[sim_motor_ids[part][i]] + meters;
                str << sim_motor_ids[part][i] << "\t|\t" << real_motor_ids[part][i] << "\t|\t" << position[part][i] << "\t|\t" << l_offset[part][i] << endl;
            }
        }
        ROS_INFO_STREAM(str.str());
        ROS_INFO("changing control mode to POSITION");
        for (const auto &part:body_parts){
          if (use_motor_config[part]) {
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
          } else {
            roboy_middleware_msgs::ControlMode msg;
            for (int i = 0; i < sim_motor_ids[part].size(); i++) {
              msg.request.motor_id.push_back(i);
              msg.request.set_point = position[part][i];
              msg.request.control_mode = POSITION;
            }
          motor_control_mode[part].call(msg);
          }
        }
        ROS_INFO("pose init done");
        initialized = true;
        return true;
    }

    void MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg){
        for (const auto &part : body_parts) {
            if (msg->id==bodyPartIDs[part]) {
                int j = 0;
                for (const int &real_motor_id : real_motor_ids[part]) {
                    position[part][j] = msg->position[real_motor_id];
                    velocity[part][j] = msg->velocity[real_motor_id];
                    displacement[part][j] = msg->displacement[real_motor_id];
                    j++;
                }
                motor_status_received[part] = true;
            }
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
      // initialized = true;
        if(initialized) {
            double Kp_dl = 0;
            nh->getParam("Kp_dl",Kp_dl);
            stringstream str;
            for (auto part:body_parts) {
                str << part << ": ";
                roboy_middleware_msgs::MotorCommand msg;
                msg.id = bodyPartIDs[part];
                msg.motors = real_motor_ids[part];
                for (int i = 0; i < sim_motor_ids[part].size(); i++) {
                    double l_meter = (-l_target[sim_motor_ids[part][i]] + l_offset[part][i]) - Kp_dl*(l_target[sim_motor_ids[part][i]]-l[sim_motor_ids[part][i]]);
                    str << sim_motor_ids[part][i] << "\t" << l_meter << "\t";
                    switch (motor_type[part][i]) {
                        case MYOBRICK100N: {
                            str << myoBrick100NEncoderTicksPerMeter(l_meter) << "\t";
                            msg.set_points.push_back(myoBrick100NEncoderTicksPerMeter(l_meter));
                            break;
                        }
                        case MYOBRICK300N: {
                            str << myoBrick300NEncoderTicksPerMeter(l_meter) << "\t";
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
    vector<string> body_parts = {"arm_right", "shoulder_right"};
    map<string, vector<string>> endeffector_jointnames;
    bool initialized = false;
    map<string, bool> motor_status_received;
    map<string, int> init_mode, init_setpoint;
    map<string,vector<int>> real_motor_ids, sim_motor_ids, motor_type;
    map<string,vector<double>> l_offset, position, velocity, displacement;
    map<string,int> bodyPartIDs;
    map<string,bool> use_motor_config;
};

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_icecream");
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

    if (nh.hasParam("simulated")) {
      nh.getParam("simulated", robot.simulated);
    }

    ros::Rate rate(5.0);
    while(ros::ok()){
        robot.read();
        if (!robot.simulated)
          robot.write();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("TERMINATING...");

    return 0;
}
