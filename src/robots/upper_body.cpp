// #include "kindyn/vrpuppet.hpp"
#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorState.h>
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

class UpperBody: public cardsflow::kindyn::Robot{
private:
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::Subscriber motor_state_sub, motor_info_sub;
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

public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    UpperBody(string urdf, string cardsflow_xml){

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "upper_body");
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

        motor_state_sub = nh->subscribe("/roboy/middleware/MotorState",1,&UpperBody::MotorState,this);
        // motor_info_sub = nh->subscribe("/roboy/middleware/MotorInfo",1,&UpperBody::MotorInfo,this);

        for (auto body_part: body_parts) {
            init_called[body_part] = false;
            motor_config[body_part] = nh->serviceClient<roboy_middleware_msgs::MotorConfigService>("/roboy/middleware/"+body_part+"/MotorConfig");
            control_mode[body_part] = nh->serviceClient<roboy_middleware_msgs::ControlMode>( "/roboy/middleware/ControlMode");//+body_part+"ControlMode");
        }

        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        init_pose = nh->advertiseService("init_pose", &UpperBody::initPose,this);

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

        if (name == "wrist_left" || name == "writst_right") {
            int init_m3_displacement;
            try {
                nh->getParam("init_m3_displacement",init_m3_displacement); }
            catch (const std::exception&) {
                ROS_ERROR_STREAM("rosparam init_m3_displacement is not set. will not init.");
                return false;
            }

            ROS_INFO_STREAM("changing control mode for " << name << " with displacement " << init_m3_displacement);
            roboy_middleware_msgs::MotorConfigService msg;

            roboy_middleware_msgs::MotorConfig config_msg;

            for (int id: motor_ids) {
                config_msg.global_id.push_back(id);
                config_msg.setpoint.push_back(init_m3_displacement);
                config_msg.update_frequency.push_back(100);
                config_msg.control_mode.push_back(DISPLACEMENT);
                config_msg.deadband.push_back(0);
                config_msg.IntegralLimit.push_back(50);
                config_msg.PWMLimit.push_back(500);
                config_msg.Kp.push_back(1);
                config_msg.Ki.push_back(0);
                config_msg.Kd.push_back(0);
            }

            msg.request.config = config_msg;

            auto success = motor_config[name].call(msg);
            auto res = msg.response.mode;
            for (int i=0;i<res.size();i++) {
                success *= res[i] == DISPLACEMENT;
            }
            if (!success) {
                ROS_ERROR_STREAM("Failed to change M3 control mode on " << name << " to DISPLACEMENT");
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
                l_offset[motor_id] = l[motor_id] + position[motor_id];
                str << motor_id << "\t|\t" << position[motor_id] << "\t|\t" << l[motor_id] << "\t|\t" << l_offset[motor_id] << endl;
            }


            ROS_INFO_STREAM(str.str());

            ROS_INFO_STREAM("changing control mode of %s to POSITION" << name);

            // config_msg.update_frequency = std::vector<int> v(motor_ids.size(), 100);
            // config_msg.control_mode = std::vector<int> v(motor_ids.size(), ENCODER0_POSITION);
            // config_msg.deadband = std::vector<int> v(motor_ids.size(), 0);
            // config_msg.IntegralLimit = std::vector<int> v(motor_ids.size(), 50);
            // config_msg.PWMLimit = std::vector<int> v(motor_ids.size(), 500);
            // config_msg.current_limit = std::vector<float> v(motor_ids.size(), 1);
            // config_msg.Kp = std::vector<float> v(motor_ids.size(), 1);
            // config_msg.Ki = std::vector<float> v(motor_ids.size(), 0);
            // config_msg.Kd = std::vector<float> v(motor_ids.size(), 0);
            // config_msg.motor = std::vector<int> v;
            // config_msg.setpoint = std::vector<int> v;

            for (int id: motor_ids) {
                config_msg.global_id.push_back(id);
                config_msg.setpoint.push_back(position[id]);
            }

            msg.request.config = config_msg;

            success = motor_config[name].call(msg);
            res = msg.response.mode;
            for (int i=0;i<res.size();i++) {
                success *= res[i] == ENCODER0_POSITION;
            }

            if (!success) {
                ROS_ERROR_STREAM("Failed to change M3 control mode on " << name << " to ENCODER0_POSITION");
                return false;
            }

            vector<float> _integral(motor_ids.size(), 0);
            integral[name] = _integral;

        }
        else {
            int pwm;
            try {
                nh->getParam("pwm",pwm); }
            catch (const std::exception&) {
                ROS_ERROR_STREAM("rosparam pwm is not set. will not init.");
                return false;
            }


            ROS_INFO("changing control mode of motors to PWM with %d",pwm);
            roboy_middleware_msgs::ControlMode msg;
            msg.request.control_mode = DIRECT_PWM;
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


        }

        update();
        ROS_INFO_STREAM("%s pose init done" << name);
        init_called[name] = true;
        return true;

    }

//     float myoBrickMeterPerEncoderTicks(int motor_pos) {
// // #ifdef LEGACY
//             // TODO!!
//          return ((motor_pos/4096.0f)*(M_PI*0.0085f));
// // #else
// //        return ((motor_pos/2048.0f)*(M_PI*0.0085f));
// // #endif
//     }

//     float myoBrickEncoderTicksPerMeter(float meter){
//         return  (meter/(M_PI*0.0085f))*(4096.0f);
//         // TODO  legacy motoboards
// //        return (meter/(M_PI*0.0085f))*2048.0f;

//     }

    string findBodyPartByMotorId(int id) {
        for (auto body_part: body_parts) {
            std::vector<int> motor_ids;
            try {
//                mux.lock();
                nh->getParam(body_part+"/motor_ids", motor_ids);
//                mux.unlock();
            }
            catch (const std::exception&) {
                ROS_ERROR("motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", body_part);
                return "unknown";
            }
            if (find(motor_ids.begin(), motor_ids.end(), id) != motor_ids.end()) {
                return body_part;
            }
        }
        ROS_WARN_THROTTLE(10,"Seems like motor with id %d does not belong to any body part", id);
        return "unknown";
    }

    void MotorState(const roboy_middleware_msgs::MotorState::ConstPtr &msg){
        int i=0;
        for (auto id:msg->global_id) {
            // ROS_INFO_STREAM("ID: " << id);
            position[id] = msg->encoder0_pos[i];
            i++;
        }
    }

    void MotorInfo(const roboy_middleware_msgs::MotorInfo::ConstPtr &msg){
      int i=0;
      for (auto id:msg->global_id) {
        auto body_part = findBodyPartByMotorId(id);
        if (msg->communication_quality[i] > 0 && body_part != "unknown") {
            motor_status_received[body_part] = true;
        }
        else {
            ROS_WARN_THROTTLE(1, "Did not receive %s's motor status for motor with id: %d", (body_part.c_str(), id));
        }
        i++;
      }
    }


    /**
     * Updates the robot model
     */
    void read(){
        update();
        if (!external_robot_state)
            forwardKinematics(0.00001);
    };
    /**
     * Sends motor commands to the real robot
     */
    void write(){

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


                str << endl << "motor_id | l_meter | ticks | error | integral" << endl;
                char s[200];

                for (int i = 0; i < motor_ids.size(); i++) {
                    int motor_id = motor_ids[i];
                    float error = l[motor_id] - l_target[motor_id];
                    if(Ki_dl==0){
                        integral[body_part][motor_id] = 0;
                    }else {
                        integral[body_part][motor_id] += Ki_dl * error;
                        if (integral[body_part][motor_id] > integral_limit)
                            integral[body_part][motor_id] = integral_limit;
                        if (integral[body_part][motor_id] < -integral_limit)
                            integral[body_part][motor_id] = -integral_limit;
                        if (Ki_dl == 0) {
                            integral[body_part][motor_id] = 0;
                        }
                    }
                    if (motor_id == 16 || motor_id == 17) {
                        Kp_dl = 0; Ki_dl = 0;

                    }
                    l_meter[motor_id] = (l_offset[motor_id] - l_target[motor_id]) +
                         Kp_dl*error + integral[body_part][motor_id];

                    sprintf(s,     "%d            | %.6f   | %.6f    |  %.6f   | %.6f\n",
                            motor_id,l_meter[motor_id],l_meter[motor_id]),error,integral[body_part][motor_id];
                    str <<  s;
                }

                str << endl;
                ROS_INFO_STREAM_THROTTLE(2,str.str());

                roboy_middleware_msgs::MotorCommand msg;
                msg.global_id = {};
                msg.setpoint = {};
                for (int i = 0; i < motor_ids.size(); i++) {
                    msg.global_id.push_back(motor_ids[i]);
                    msg.setpoint.push_back(l_meter[motor_ids[i]]);
                }
                // ROS_WARN_STREAM_THROTTLE(1, msg);
                motor_command.publish(msg);

        }
     }
    };

};

/**
 * controller manager update thread. Here you can define how fast your controllers should run
 * @param cm pointer to the controller manager
 */
void update(controller_manager::ControllerManager *cm) {
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(200); // changing this value affects the control speed of your running controllers
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
        ros::init(argc, argv, "upper_body");
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


    UpperBody robot(urdf, cardsflow_xml);
    controller_manager::ControllerManager cm(&robot);

    if (nh.hasParam("simulated")) {
      nh.getParam("simulated", robot.simulated);
    }

    thread update_thread(update, &cm);
    update_thread.detach();

    ros::Rate rate(30);
    while(ros::ok()){
        robot.read();
        if (!robot.simulated)
          robot.write();
        ros::spinOnce();
//        rate.sleep();
    }

    ROS_INFO("TERMINATING...");
    update_thread.join();

    return 0;
}
