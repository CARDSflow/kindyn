// #include "kindyn/vrpuppet.hpp"
#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorState.h>
#include <roboy_middleware_msgs/RoboyState.h>
#include <roboy_middleware_msgs/MotorInfo.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <roboy_middleware_msgs/SetStrings.h>
#include <roboy_middleware_msgs/SystemStatus.h>
#include <roboy_middleware_msgs/BodyPart.h>
#include <roboy_simulation_msgs/Tendon.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace std;

class UpperBody: public cardsflow::kindyn::Robot{
private:
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command, system_status_pub, tendon_motor_pub; /// motor command publisher
    ros::Subscriber motor_state_sub, motor_info_sub, roboy_state_sub;
    // vector<ros::ServiceServer> init_poses;
    ros::ServiceServer init_pose;
    ros::AsyncSpinner *spinner;
    // ros::ServiceClient motor_control_mode, motor_config, control_mode;

    map<int,int> pos, initial_pos;
    map<string, ros::ServiceClient> motor_config, motor_control_mode, control_mode;
    map<string, bool> motor_status_received;
    map<int, bool> communication_established; // keeps track of communication quality for each motor
    map<int,float> l_offset, position, tendon_length;
    map<string, vector<float>> integral, error_last;
    boost::shared_ptr<tf::TransformListener> listener;
    std::vector<string> body_parts = {"shoulder_right", "shoulder_left","head", "wrist_right","wrist_left"};//, "shoulder_left"};//}, "elbow_left"};
    map<string, bool> init_called;
    boost::shared_ptr<std::thread> system_status_thread;
    ros::Time prev_roboy_state_time;
    VectorXd set_point, prev_set_point;
    map<int, bool> is_reached;
    map<int, int> is_reached_times;
    float Kp_dl = 0, Kd_dl = 0, Ki_dl = 0, Kp_disp = 0, integral_limit = 0.01;

public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    UpperBody(string urdf, string cardsflow_xml, string robot_model){

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, robot_model + "upper_body");
        }

        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        spinner = new ros::AsyncSpinner(0);
        spinner->start();

        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        nh->getParam("external_robot_state", external_robot_state);
        ROS_INFO_STREAM("External robot state: " << external_robot_state);
        topic_root = "/roboy/" + robot_model + "/";


        init(urdf,cardsflow_xml,joint_names);
//        listener.reset(new tf::TransformListener);
        update();

        set_point.resize(number_of_cables);
        prev_set_point.resize(number_of_cables);


        motor_state_sub = nh->subscribe(topic_root + "middleware/MotorState",1,&UpperBody::MotorState,this);
        roboy_state_sub = nh->subscribe(topic_root + "middleware/RoboyState",1,&UpperBody::RoboyState,this);
        motor_info_sub = nh->subscribe(topic_root + "middleware/MotorInfo",1,&UpperBody::MotorInfo,this);

        for (auto body_part: body_parts) {
            init_called[body_part] = false;
            motor_config[body_part] = nh->serviceClient<roboy_middleware_msgs::MotorConfigService>(topic_root + "middleware/"+body_part+"/MotorConfig");
            control_mode[body_part] = nh->serviceClient<roboy_middleware_msgs::ControlMode>( topic_root + "middleware/ControlMode");//+body_part+"ControlMode");
        }

        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>(topic_root + "middleware/MotorCommand",1);
        init_pose = nh->advertiseService(topic_root + "init_pose", &UpperBody::initPose,this);

        system_status_pub = nh->advertise<roboy_middleware_msgs::SystemStatus>(topic_root + "control/SystemStatus",1);
        system_status_thread = boost::shared_ptr<std::thread>(new std::thread(&UpperBody::SystemStatusPublisher, this));
        system_status_thread->detach();

        tendon_motor_pub = nh->advertise<roboy_simulation_msgs::Tendon>(topic_root + "control/tendon_state_motor", 1);

        nh->setParam("initialized", init_called);

        ROS_INFO_STREAM("Finished setup");
    };

    ~UpperBody() {
        if (system_status_thread->joinable())
            system_status_thread->join();
    }

    void SystemStatusPublisher() {
        ros::Rate rate(100);
        while (ros::ok()) {
            auto msg = roboy_middleware_msgs::SystemStatus();
            msg.header.stamp = ros::Time::now();
            auto body_part = roboy_middleware_msgs::BodyPart();
            for (auto part: body_parts) {
                body_part.name = part;
                body_part.status = !init_called[part];
                msg.body_parts.push_back(body_part);
            }
            system_status_pub.publish(msg);
            rate.sleep();
        }

    }


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

        write_initialization(name);

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

//        ROS_INFO_STREAM("changing control mode of %s to POSITION" << name);
//
//        roboy_middleware_msgs::ControlMode msg1;
//        msg1.request.control_mode = ENCODER0_POSITION;
//        for (int id: motor_ids) {
//            msg1.request.global_id.push_back(id);
//            msg1.request.set_points.push_back(position[id]);
//        }
//
//        if (!control_mode[name].call(msg1)) {
//            ROS_ERROR_STREAM("Changing control mode for %s didnt work" << name);
//            return false;
//        }

        vector<float> _integral(motor_ids.size(), 0);
        vector<float> _error(motor_ids.size(), 0);
        integral[name] = _integral;
        error_last[name] = _error;


//        }

        // TODO: Interpolate q_target from q_ext here
        // Update q_target

//        move_to_zero_position(name);
//        VectorXd delta = (q - q_ext) / 500;
//        ROS_INFO_STREAM_THROTTLE(0.5, "******* " << delta);

        update();

//        set_point.setZero();
//        prev_set_point.setZero();
//        for (int id: motor_ids) {
//            is_reached[id] = true;
//            is_reached_times[id] = 0;
//        }

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
            position[id] = msg->encoder0_pos[i];
            tendon_length[id] = l_offset[id] - position[id];
            i++;
        }

//        roboy_simulation_msgs::Tendon tendon_msg;
//        for (int j=0; j < tendon_length.size(); j++){
//            tendon_msg.l.push_back(tendon_length[j]);
//        }
//        tendon_motor_pub.publish(tendon_msg);
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

                            // set respecitve body part joint targets to 0
                            string endeffector;
                            nh->getParam(body_part+"/endeffector", endeffector);
                            if ( !endeffector.empty() ) {
                                vector<string> ik_joints;
                                nh->getParam((endeffector + "/joints"), ik_joints);
                                if (ik_joints.empty()) {
                                    ROS_ERROR(
                                            "endeffector %s has no joints defined, check your endeffector.yaml or parameter server.  skipping...",
                                            body_part.c_str());
                                }
                                else {

                                    for (auto joint: ik_joints) {
                                        int joint_index = GetJointIdByName(joint);
                                        if (joint_index != iDynTree::JOINT_INVALID_INDEX) {
                                            q_target(joint_index) = 0;
                                            ROS_WARN_STREAM("Set target 0 for " << joint);
                                        }
                                    }

                                }
                            }

                        }
                            
                    //}

                }
            }

        }
//      int i=0;
//      for (auto id:msg->global_id) {
//           ROS_INFO_STREAM("ID: " << id);
//        auto body_part = findBodyPartByMotorId(id);
//        if (msg->communication_quality[i] > 0 && body_part != "unknown") {
//            motor_status_received[body_part] = true;
////            communication_established[id] = true;
//        }
//        else {
////            communication_established[id] = false;
//            ROS_WARN_THROTTLE(1, "Did not receive %s's motor status for motor with id: %d. Body part is disabled.", (body_part.c_str(), id));
//            init_called[body_part] = false;
//        }
//        i++;
//      }
    }


    /**
     * Updates the robot model
     */
    void read(){
//        ROS_INFO_STREAM("***** 0 " << q[11]);
        update();
//        ROS_INFO_STREAM("***** 1 " << q[11]);
        forwardKinematics(0.005);
//        ROS_INFO_STREAM("***** 2 " << q[11]);
//        if (!external_robot_state)
//            forwardKinematics(0.005);
    };

    void write_initialization(string body_part){

        ros::Rate init_rate(40);

        std::vector<int> motor_ids;
        try {
            nh->getParam(body_part+"/motor_ids", motor_ids); }
        catch (const std::exception&) {
            ROS_ERROR("motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", body_part);
        }

        roboy_middleware_msgs::MotorCommand msg;
        msg.global_id = {};
        msg.setpoint = {};

//        if (abs(error) < 0.005) {
//            is_reached[motor_ids[i]] = true;
//        } else {
//            is_reached[motor_ids[i]] = false;
//        }

//        auto body_idx = GetJointIdByName(body_part);
//        auto joint_error = (q(body_idx) - q_ext(body_idx)).norm();
        //int id = 17;
        //float error = 999.0;
        //auto setpoint = position[id];
//        auto setpoint_vel = 0.;
        float accumulated_error = 0.0;
        do {
            accumulated_error = 0;
            msg.global_id = {};
            msg.setpoint = {};
            if (nh->hasParam("Kp_dl"))
                nh->getParam("Kp_dl",Kp_dl);
            for (auto motor_id: motor_ids) {
                auto setpoint = position[motor_id];
                auto error = l_ext[motor_id] - l_target[motor_id];
                accumulated_error += abs(error);
                auto setpoint_vel = Kp_dl * error;
                setpoint += setpoint_vel;

                msg.global_id.push_back(motor_id);
                msg.setpoint.push_back(setpoint);
            }

            motor_command.publish(msg);
            init_rate.sleep();
            ROS_INFO_STREAM_THROTTLE(1, "accumulated error: " << accumulated_error);

        } while (abs(accumulated_error) > motor_ids.size()*0.0025);

//        while(abs(accumulated_error) > 0.01) {
//            accumulated_error = 0;
//            if (nh->hasParam("Kp_dl"))
//                nh->getParam("Kp_dl",Kp_dl);
//            for (auto motor_id: motor_ids) {
//                auto setpoint = position[motor_id];
//                auto error = l_ext[motor_id] - l_target[motor_id];
//                accumulated_error += abs(error);
//                auto setpoint_vel = Kp_dl * error;
//                setpoint += setpoint_vel;
//
//                msg.global_id.push_back(motor_id);
//                msg.setpoint.push_back(setpoint);
//            }
//
//////            if (nh->hasParam("Kd_dl"))
//////                nh->getParam("Kd_dl",Kd_dl);
//////            if (nh->hasParam("Ki_dl")) nh->getParam("Ki_dl",Ki_dl);
//////            if (nh->hasParam("integral_limit")) nh->getParam("integral_limit",integral_limit);
////
//////            for (int i = 0; i < motor_ids.size(); i++) {
////
////                error = l_ext[id] - l_target[id];
////
//////                if (abs(error) < 0.005) {
//////                    is_reached[motor_ids[i]] = true;
//////                } else {
//////                    is_reached[motor_ids[i]] = false;
//////                }
////
////
////                setpoint_vel = Kp_dl * error;
////                setpoint += setpoint_vel;
////
////                ROS_INFO_STREAM_THROTTLE(1, "*** m_error " << error << "\t sp " << setpoint);
////
//////                    set_point[motor_ids[i]] = -l[motor_ids[i]] + l_offset[motor_ids[i]];
////                // + setpoint_vel; //+ Kd_dl * (error - error_last[body_part][motor_ids[i]])/diff.toSec(); //+ integral[body_part][motor_ids[i]];
////
////                msg.global_id.push_back(17);
////                msg.setpoint.push_back(setpoint);
//////                error_last[body_part][17] = error;
////            }
//
//            motor_command.publish(msg);
//            init_rate.sleep();
//        }
////                 ROS_WARN_STREAM_THROTTLE(1, msg);


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

//        if (nh->hasParam("Kp_dl"))
//            nh->getParam("Kp_dl",Kp_dl);
//        if (nh->hasParam("Kd_dl"))
//            nh->getParam("Kd_dl",Kd_dl);
//        if (nh->hasParam("Ki_dl")) nh->getParam("Ki_dl",Ki_dl);
//        if (nh->hasParam("integral_limit")) nh->getParam("integral_limit",integral_limit);

        for (auto body_part: body_parts) {
            if (!init_called[body_part]) {
                ROS_WARN_STREAM_THROTTLE(10, body_part << " was not initialized. skipping");
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


                // str << endl << "motor_id | l_meter | ticks | error | integral" << endl;
                // char s[200];
                // str << endl;
                // ROS_INFO_STREAM_THROTTLE(2,str.str());

//                str << endl << "motor_id | l_meter | ticks | error | integral" << endl;
//                char s[200];
                roboy_middleware_msgs::MotorCommand msg;
                msg.global_id = {};
                msg.setpoint = {};

//                for (int i = 0; i < motor_ids.size(); i++) {
//                   int motor_id = motor_ids[i];
//                   // double l_change = l[motor_id] - l_offset[motor_id];
//
//                   double error = -l_target[motor_id] + l[motor_id] ;
//                   if(Ki_dl==0){
//                       integral[body_part][motor_id] = 0;
//                   }else {
//                       integral[body_part][motor_id] += Ki_dl * error;
//                       if (integral[body_part][motor_id] > integral_limit)
//                           integral[body_part][motor_id] = integral_limit;
//                       if (integral[body_part][motor_id] < -integral_limit)
//                           integral[body_part][motor_id] = -integral_limit;
//                       if (Ki_dl == 0) {
//                           integral[body_part][motor_id] = 0;
//                       }
//                   }
//                   l_meter[motor_id] = (-l[motor_ids[i]] + l_offset[motor_ids[i]]) +
//                        Kp_dl*error + integral[body_part][motor_id];
//
////                   sprintf(s,     "%d            | %.6f   | %.6f    |  %.6f   | %.6f\n",
////                           motor_id,l_meter[motor_id],l_meter[motor_id]),error,integral[body_part][motor_id];
////                   str <<  s;
//                    ROS_INFO_STREAM(motor_id << "\t" << error);
//                    msg.global_id.push_back(motor_id);
//                    msg.setpoint.push_back(l_meter[motor_id]);
//                }
//
//                ROS_INFO_STREAM_THROTTLE(1,msg);
//                motor_command.publish(msg);
//                str << endl;
//                ROS_INFO_STREAM_THROTTLE(2,str.str());


                for (int i = 0; i < motor_ids.size(); i++) {

//                   float error = l_ext[motor_ids[i]] - l_target[motor_ids[i]] ;

//                   if(Ki_dl==0){
//                       integral[body_part][motor_ids[i]] = 0;
//                   }else {
//                       integral[body_part][motor_ids[i]] += Ki_dl * error;
//                       if (integral[body_part][motor_ids[i]] > integral_limit)
//                           integral[body_part][motor_ids[i]] = integral_limit;
//                       if (integral[body_part][motor_ids[i]] < -integral_limit)
//                           integral[body_part][motor_ids[i]] = -integral_limit;
//                       if (Ki_dl == 0) {
//                           integral[body_part][motor_ids[i]] = 0;
//                       }
//                   }

//                    auto setpoint_vel = Kp_dl*error;
//                    if (abs(error) < 0.005) {
//                        is_reached[motor_ids[i]] = true;
//                    } else {
//                        is_reached[motor_ids[i]] = false;
//                    }
//
//                    if (is_reached[motor_ids[i]]) {
//                        is_reached_times[motor_ids[i]]++;
//                        set_point[motor_ids[i]] = -l[motor_ids[i]] + l_offset[motor_ids[i]];
//                        is_reached[motor_ids[i]] = false;
////                        if (is_reached_times[motor_ids[i]] == 2) {
////                            l_offset[motor_ids[i]] = l[motor_ids[i]] + position[motor_ids[i]];
////                            nh->setParam("Kp_dl", 0);
////                            ROS_INFO_STREAM("******* L_OFFSET ******" << l_offset[motor_ids[i]]);
////                        }
//
//                        if (motor_ids[i] == 17) {
//                            ROS_INFO_STREAM("******* REACHED ****** " << set_point[motor_ids[i]]);
//                        }
//                    } else {
//                        set_point[motor_ids[i]] += setpoint_vel;
//                    }
//
//                    if(motor_ids[i] == 17){
//                        ROS_INFO_STREAM_THROTTLE(1, "*** \t" << is_reached[motor_ids[i]] << "\t" << error << "\t" << set_point[motor_ids[i]]);
//                    }

                    set_point[motor_ids[i]] = -l[motor_ids[i]] + l_offset[motor_ids[i]];
                    // + setpoint_vel; //+ Kd_dl * (error - error_last[body_part][motor_ids[i]])/diff.toSec(); //+ integral[body_part][motor_ids[i]];


                    msg.global_id.push_back(motor_ids[i]);
//                    ROS_INFO_STREAM("motor id: " << motor_ids[i] << " l: " << l[motor_ids[i]] << " l_target: " << l_target[motor_ids[i]]);
                    msg.setpoint.push_back(set_point[motor_ids[i]]);
//                    msg.setpoint.push_back(l_meter[motor_ids[i]]);

//                    error_last[body_part][motor_ids[i]] = error;

                }
//                if(is_reached_times[17] < 3) {
                    motor_command.publish(msg);
//                }
//                 ROS_WARN_STREAM_THROTTLE(1, msg);


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
    ros::Rate rate(500); // changing this value affects the control speed of your running controllers
    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        cm->update(time, period);
        prev_time = time;
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {

    string robot_model(argv[1]);
    ROS_INFO_STREAM("launching " << robot_model);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, robot_model + "_upper_body");
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


    UpperBody robot(urdf, cardsflow_xml,robot_model);
    controller_manager::ControllerManager cm(&robot);

    if (nh.hasParam("simulated")) {
      nh.getParam("simulated", robot.simulated);
    }

    thread update_thread(update, &cm);
    update_thread.detach();

    ros::Rate rate(200);
    while(ros::ok()){
        robot.read();
        if (!robot.simulated)
          robot.write();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("TERMINATING...");
    update_thread.join();

    return 0;
}
