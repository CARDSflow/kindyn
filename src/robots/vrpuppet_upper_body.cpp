#include "kindyn/vrpuppet.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorState.h>
#include <roboy_middleware_msgs/RoboyState.h>
#include <roboy_middleware_msgs/MotorInfo.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <roboy_middleware_msgs/SetStrings.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace std;

class PuppetUpperBody: public cardsflow::vrpuppet::Robot{
private:
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::Subscriber motor_state_sub, motor_info_sub, roboy_state_sub;
    // vector<ros::ServiceServer> init_poses;
    ros::ServiceServer init_pose;
    ros::AsyncSpinner *spinner;
    // ros::ServiceClient motor_control_mode, motor_config, control_mode;

    map<int,int> pos, initial_pos;
    map<string, ros::ServiceClient> motor_config, motor_control_mode, control_mode;
    map<string, bool> motor_status_received;
    map<int,float> l_offset, position;
    map<string, vector<float>> integral;
    boost::shared_ptr<tf::TransformListener> listener;
    std::vector<string> body_parts = {"shoulder_left", "shoulder_right", "head", "wrist_left", "wrist_right"};//, "shoulder_left"};//}, "elbow_left"};
    map<string, bool> init_called;
    ros::Time prev_roboy_state_time;

public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    PuppetUpperBody(string urdf, string cardsflow_xml){

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "vrpuppet_upper_body");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        spinner = new ros::AsyncSpinner(0);
        spinner->start();

        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        nh->getParam("external_robot_state", external_robot_state);
        ROS_INFO_STREAM("External robot state: " << external_robot_state);
        init(urdf,cardsflow_xml,joint_names);
        listener.reset(new tf::TransformListener);
        update();

        motor_state_sub = nh->subscribe("/roboy/pinky/middleware/MotorState",1,&PuppetUpperBody::MotorState,this);
        roboy_state_sub = nh->subscribe("/roboy/pinky/middleware/RoboyState",1,&PuppetUpperBody::RoboyState,this);
        // motor_info_sub = nh->subscribe("/roboy/pinky/middleware/MotorInfo",1,&PuppetUpperBody::MotorInfo,this);

        for (auto body_part: body_parts) {
            init_called[body_part] = false;
            motor_config[body_part] = nh->serviceClient<roboy_middleware_msgs::MotorConfigService>("/roboy/pinky/middleware/MotorConfig");
            control_mode[body_part] = nh->serviceClient<roboy_middleware_msgs::ControlMode>( "/roboy/pinky/middleware/ControlMode");//+body_part+"ControlMode");
        }

        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/pinky/middleware/MotorCommand",1);
        init_pose = nh->advertiseService("/roboy/pinky/init_pose", &PuppetUpperBody::initPose,this);

        ROS_INFO_STREAM("Finished setup");
    };

    bool initPose(roboy_middleware_msgs::SetStrings::Request &req,
                  roboy_middleware_msgs::SetStrings::Response &res){
        res.result = true;

        if (find(req.strings.begin(), req.strings.end(), "all") != req.strings.end()) {
            req.strings = body_parts;
        }

        for (string body_part: req.strings) {
            ROS_WARN_STREAM("init called for: " << body_part);
            init_called[body_part] = false;
            auto r = initBodyPart(body_part);
            res.result = r*res.result;
        }

        return res.result;
    }


    bool initBodyPart(string name) {
        ROS_WARN_STREAM("initBodyPart: " << name);

        std::vector<int> motor_ids;
        try {
            nh->getParam(name+"/motor_ids", motor_ids);
        }
        catch (const std::exception&) {
            ROS_ERROR("motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", name);
            return false;
        }
        int pwm;
        try {
            if (name == "wrist_left" || name == "wrist_right") {
                nh->getParam("m3_pwm", pwm);
            } else {
                nh->getParam("pwm",pwm);
            }
        }
        catch (const std::exception&) {
            ROS_ERROR_STREAM("rosparam pwm or init_m3_displacement is not set. will not init.");
            return false;
        }


        ROS_INFO("changing control mode of motors to PWM with %d",pwm);
        roboy_middleware_msgs::ControlMode msg;


        // if (name == "wrist_left" || name == "wrist_right") {
        //     msg.request.control_mode = DISPLACEMENT;
        // } else {
        msg.request.control_mode = DIRECT_PWM;
        // }
        // TODO: fix in plexus PWM direction for the new motorboard
        std::vector<float> set_points(motor_ids.size(), pwm);
        for (auto m: motor_ids) msg.request.global_id.push_back(m);
        msg.request.set_points = set_points;

        stringstream str1;
        for(int i=0;i<msg.request.set_points.size();i++) {
            int motor_id = motor_ids[i];

            str1 << msg.request.global_id[i] << "\t|\t" << msg.request.set_points[i] << endl;
        }


        ROS_INFO_STREAM(str1.str());


        if (!control_mode[name].call(msg)) {
            ROS_ERROR("Changing control mode for %s didnt work", name);
            return false;
        }


        ros::Time t0;
        t0= ros::Time::now();
        double timeout = 0;
        nh->getParam("timeout",timeout);
        if(timeout==0) {
            int seconds = 5;
            while ((ros::Time::now() - t0).toSec() < 5) {
                ROS_INFO_THROTTLE(1, "waiting %d", seconds--);
            }
        }else{
            int seconds = timeout;
            while ((ros::Time::now() - t0).toSec() < timeout) {
                ROS_INFO_THROTTLE(1, "waiting %d", seconds--);
            }
        }
        motor_status_received[name] = true;
        if(!motor_status_received[name]) {
            ROS_ERROR("did not receive motor status for %s, try again", name);
            return false;
        }

        stringstream str;
        str << "saving position offsets:" << endl << "motor id  |   position offset [ticks]  | length_sim[m] | length offset[m]" << endl;

        // for (int i = 0; i<motor_ids.size();i++) str << motor_ids[i] << ": " << position[motor_ids[i]] << ", ";
        // str << endl;

        for(int i=0;i<motor_ids.size();i++) {
            int motor_id = motor_ids[i];
            ROS_WARN_STREAM(name << " info print");
            l_offset[motor_id] = l[motor_id] + position[motor_id];
            str << motor_id << "\t|\t" << position[motor_id] << "\t|\t" << l[motor_id] << "\t|\t" << l_offset[motor_id] << endl;
        }


        ROS_INFO_STREAM(str.str());

        ROS_INFO_STREAM("changing control mode of %s to POSITION" << name);

        roboy_middleware_msgs::ControlMode msg1;
        msg1.request.control_mode = ENCODER0_POSITION;
        for (int id: motor_ids) {
            msg1.request.global_id.push_back(id);
            msg1.request.set_points.push_back(position[id]);
        }

        if (!control_mode[name].call(msg1)) {
            ROS_ERROR_STREAM("Changing control mode for %s didnt work" << name);
            return false;
        }

        vector<float> _integral(motor_ids.size(), 0);
        integral[name] = _integral;


//        }

        update();
        ROS_INFO_STREAM("%s pose init done" << name);
        init_called[name] = true;
        nh->setParam("initialized", init_called);

        return true;


    }


    string findBodyPartByMotorId(int id) {
        string ret = "unknown";
        for (auto body_part: body_parts) {
            std::vector<int> motor_ids;
            try {
//                mux.lock();
                nh->getParam(body_part+"/motor_ids", motor_ids);
//                mux.unlock();
            }
            catch (const std::exception&) {
                ROS_ERROR("motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", body_part);
                return ret;
            }
            if (find(motor_ids.begin(), motor_ids.end(), id) != motor_ids.end()) {
                return body_part;
            }
        }
        ROS_WARN_ONCE("Seems like motor with id %d does not belong to any body part", id);
        return ret;
    }

    void MotorState(const roboy_middleware_msgs::MotorState::ConstPtr &msg){
        int i=0;
        for (auto id:msg->global_id) {
            // ROS_INFO_STREAM("ID: " << id);
            position[id] = msg->encoder0_pos[i];
            i++;
        }
    }
    void RoboyState(const roboy_middleware_msgs::RoboyState::ConstPtr &msg) {
        prev_roboy_state_time = msg->header.stamp;
    }

    void MotorInfo(const roboy_middleware_msgs::MotorInfo::ConstPtr &msg){
        for (int i=0;i<msg->global_id.size();i++) {
            auto id = int(msg->global_id[i]);
            auto body_part = findBodyPartByMotorId(id);
//            ROS_INFO_STREAM(body_part);
            if (body_part != "unknown") {
                if (msg->communication_quality[i] > 0 ) {
                    motor_status_received[body_part] = true;
                    //            communication_established[id] = true;
                }
                else {
                    //            communication_established[id] = false;
                    ROS_WARN_THROTTLE(10,"Did not receive motor status for motor with id: %d. %s Body part is disabled.", (id, body_part));

                    // TODO fix triceps
                    //if (id != 18 && body_part != "shoulder_right") {
                    if(init_called[body_part]) {
                        init_called[body_part] = false;
                        nh->setParam("initialized", init_called);
                    }

                    //}

                }
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


        // check if plexus is alive
        auto diff = ros::Time::now() - prev_roboy_state_time;
        if (diff.toSec() > 1) {
            for (auto body_part: body_parts) {
                init_called[body_part] = false;
                nh->setParam("initialized", init_called);
                // reset the joint targets
                q_target.setZero();
            }
            ROS_ERROR_STREAM_THROTTLE(5, "No messages from roboy_plexus. Will not be sending MotorCommand...");
            return;
        }

        float Kp_dl = 0, Ki_dl = 0, Kp_disp = 0, integral_limit = 0;
        nh->getParam("Kp_dl",Kp_dl);
        nh->getParam("Ki_dl",Ki_dl);
        nh->getParam("integral_limit",integral_limit);

        for (auto body_part: body_parts) {
            if (!init_called[body_part]) {
                ROS_WARN_STREAM_THROTTLE(5, body_part << " was not initialized. skipping");
            } else {
                std::vector<int> motor_ids;
                try {
                    nh->getParam(body_part+"/motor_ids", motor_ids); }
                catch (const std::exception&) {
                    ROS_ERROR("motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", body_part);
                }

                stringstream str;
                map<int,float> l_meter;

                // l_meter.resize(sim_motor_ids.size());


//                str << endl << "motor_id | l_meter | ticks | error | integral" << endl;
//                char s[200];

                for (int i = 0; i < motor_ids.size(); i++) {
                    int motor_id = motor_ids[i];
                    float error = l[motor_id] - l_target[motor_id];
//                    if(Ki_dl==0){
//                        integral[body_part][motor_id] = 0;
//                    }else {
//                        integral[body_part][motor_id] += Ki_dl * error;
//                        if (integral[body_part][motor_id] > integral_limit)
//                            integral[body_part][motor_id] = integral_limit;
//                        if (integral[body_part][motor_id] < -integral_limit)
//                            integral[body_part][motor_id] = -integral_limit;
//                        if (Ki_dl == 0) {
//                            integral[body_part][motor_id] = 0;
//                        }
//                    }
//                    if (motor_id == 16 || motor_id == 17) {
//                        Kp_dl = 0; Ki_dl = 0;
//
//                    }
                    l_meter[motor_id] = (l_offset[motor_id] - l_target[motor_id]) ;//+
//                                        Kp_dl*error + integral[body_part][motor_id];

//                    sprintf(s,     "%d            | %.3f   | %.1f    |  %.3f   | %.3f\n",
//                            motor_id,l_meter[motor_id],l_meter[motor_id]),error,integral[body_part][motor_id];
//                    str <<  s;
                }

//                str << endl;
//                ROS_INFO_STREAM_THROTTLE(2,str.str());

                roboy_middleware_msgs::MotorCommand msg;
                msg.global_id = {};
                msg.setpoint = {};
                for (int i = 0; i < motor_ids.size(); i++) {
                    msg.global_id.push_back(motor_ids[i]);
                    msg.setpoint.push_back(l_meter[motor_ids[i]]);
                }
                motor_command.publish(msg);

            }
        }
    };

};

int main(int argc, char *argv[]) {
    ROS_INFO("Puppet Upper body starting...");
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "puppet_upper_body");
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


    PuppetUpperBody robot(urdf, cardsflow_xml);

    if (nh.hasParam("simulated")) {
        nh.getParam("simulated", robot.simulated);
    }

    ros::Rate rate(30);
    while(ros::ok()){
        robot.read();
        if (!robot.simulated)
            robot.write();
        ros::spinOnce();
//        rate.sleep();
    }

    ROS_INFO("TERMINATING...");

    return 0;
}