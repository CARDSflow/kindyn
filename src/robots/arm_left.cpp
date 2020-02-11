#include "kindyn/vrpuppet.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorState.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define LEGACY

using namespace std;

class LeftArmTestbed: public cardsflow::vrpuppet::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    LeftArmTestbed(string urdf, string cardsflow_xml){

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "left_arm_testbed");
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

#ifdef LEGACY
        motor_status_sub = nh->subscribe("/roboy/middleware/MotorStatus",1,&LeftArmTestbed::MotorStatus,this);
#else
        motor_state_sub = nh->subscribe("/roboy/middleware/MotorState",1,&LeftArmTestbed::MotorState,this);
#endif
        motor_config = nh->serviceClient<roboy_middleware_msgs::MotorConfigService>( "/roboy/middleware/shoulder_right/MotorConfig");
        control_mode = nh->serviceClient<roboy_middleware_msgs::ControlMode>( "/roboy/middleware/shoulder_right/ControlMode");
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        init_pose = nh->advertiseService("arm_init_pose",&LeftArmTestbed::initPose,this);


        ROS_INFO_STREAM("Finished setup");
    };

    bool initPose(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res){
        initialized = false;

        int pwm = 0;
        nh->getParam("pwm",pwm);
        ROS_INFO("changing control mode of motors to PWM with %d",pwm);
#ifdef LEGACY
        roboy_middleware_msgs::MotorConfigService msg;
        msg.request.legacy = true;
        msg.request.config.motor = real_motor_ids;
        msg.request.config.control_mode = {3,3,3,3,3,3,3,3};
        msg.request.config.Kp = {1,1,1,1,1,1,1,1};
        msg.request.config.PWMLimit = {500,500,500,500,500,500,500,500};
        msg.request.config.setpoint = {pwm,pwm,pwm,pwm,pwm,pwm,pwm,pwm};
        if(!motor_config.call(msg))
            return false;
#else
        roboy_middleware_msgs::ControlMode msg;
        msg.request.control_mode = DIRECT_PWM;
        msg.request.set_point = -pwm;
        msg.request.motor_id = {0,1,2,3,4,5,6,7};
        control_mode.call(msg);
#endif

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
        if(!motor_status_received) {
            ROS_ERROR("didi not receive motor status, try again");
            return false;
        }

        stringstream str;
        str << "saving position offsets:" << endl << "sim motor id   |  real motor id  |   position offset [ticks]  | length_sim[m] | length offset[m]" << endl;
        for (int i = 0; i<sim_motor_ids.size();i++) str << i << ": " << position[i] << ", ";
        str << endl;
        for(int i=0;i<sim_motor_ids.size();i++) {
#ifdef LEGACY
            l_offset[sim_motor_ids[i]] = l[sim_motor_ids[i]] + myoBrickMeterPerEncoderTicks(position[real_motor_ids[i]]);
#else
            l_offset[sim_motor_ids[i]] = -l[sim_motor_ids[i]] + myoBrickMeterPerEncoderTicks(position[real_motor_ids[i]]);
#endif
            str << sim_motor_ids[i] << "\t|\t" << real_motor_ids[i] << "\t|\t" << position[real_motor_ids[i]] << "\t|\t" << l[sim_motor_ids[i]] << "\t|\t" << l_offset[sim_motor_ids[i]] << endl;
        }
        ROS_INFO_STREAM(str.str());
#ifdef LEGACY
        ROS_INFO("changing control mode of myobus motors to POSITION");
        msg.request.legacy = true;
        msg.request.config.motor = real_motor_ids;
        msg.request.config.control_mode = {0,0,0,0,0,0,0,0};
        msg.request.config.Kp = {1,1,1,1,1,1,1,1};
        msg.request.config.PWMLimit = {500,500,500,500,500,500,500,500};
        for(int id:real_motor_ids){
            msg.request.config.setpoint.push_back(position[id]);
        }

        if(!motor_config.call(msg))
            return false;
#else
        roboy_middleware_msgs::ControlMode msg;
        msg.request.control_mode = ENCODER0_POSITION;
        msg.request.set_point = 0;
        msg.request.motor_id = {0,1,2,3,4,5,6,7};
        control_mode.call(msg);
#endif
        update();
        ROS_INFO("pose init done");
        initialized = true;
        return true;
    }

    float myoBrickMeterPerEncoderTicks(int motor_pos) {
#ifdef LEGACY
        return ((motor_pos/(53.0f*4.0f*512.0f))*(M_PI*0.0085f));
#else
        return ((motor_pos/2048.0f)*(M_PI*0.0085f));
#endif
    }

    float myoBrickEncoderTicksPerMeter(float meter){
#ifdef LEGACY
        return (meter/(M_PI*0.0085f))*(53.0f*4.0f*512.0f);
#else
        return (meter/(M_PI*0.0085f))*2048.0f;
#endif
    }

    void MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg){
        for (int i=0;i<msg->position.size();i++) {
            position[i] = msg->position[i];
        }
        motor_status_received = true;
    }

    void MotorState(const roboy_middleware_msgs::MotorState::ConstPtr &msg){
        for (int i=0;i<8;i++) {
            position[i] = msg->encoder0_pos[i];
        }
        motor_status_received = true;
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
        if(!initialized){
            ROS_INFO_THROTTLE(1,"waiting for init_pose service call");
        }else{
            stringstream str;
            vector<float> l_meter;
            l_meter.resize(sim_motor_ids.size());
            float Kp_dl = 0, Ki_dl = 0, Kp_disp = 0, integral_limit = 0;
            nh->getParam("Kp_dl",Kp_dl);
            nh->getParam("Ki_dl",Ki_dl);
            nh->getParam("integral_limit",integral_limit);

            str << endl << "sim_motor_id | l_meter | ticks | error | integral" << endl;
            char s[200];

            for (int i = 0; i < sim_motor_ids.size(); i++) {
                int sim_motor_id = sim_motor_ids[i];
                float error = l[sim_motor_id] - l_target[sim_motor_id];
                if(Ki_dl==0){
                    integral[sim_motor_id] = 0;
                }else {
                    integral[sim_motor_id] += Ki_dl * error;
                    if (integral[sim_motor_id] > integral_limit)
                        integral[sim_motor_id] = integral_limit;
                    if (integral[sim_motor_id] < -integral_limit)
                        integral[sim_motor_id] = -integral_limit;
                    if (Ki_dl == 0) {
                        integral[sim_motor_id] = 0;
                    }
                }
#ifdef LEGACY
                l_meter[sim_motor_id] = (l_offset[sim_motor_id] - l_target[sim_motor_id]) +
                        Kp_dl*error + integral[sim_motor_id];
#else
                l_meter[sim_motor_id] = (l_offset[sim_motor_id] + l_target[sim_motor_id]) +
                        Kp_dl*error + integral[sim_motor_id];
#endif
                sprintf(s,     "%d            | %.3f   | %.1f    |  %.3f   | %.3f\n",
                        sim_motor_id,l_meter[sim_motor_id],myoBrickEncoderTicksPerMeter(l_meter[sim_motor_id]),error,integral[sim_motor_id]);
                str <<  s;
            }
            str << endl;
            ROS_INFO_STREAM_THROTTLE(1,str.str());
            {
                roboy_middleware_msgs::MotorCommand msg;
#ifdef LEGACY
                msg.legacy = true;
#else
                msg.legacy = false;
#endif
                msg.motor = {};
                msg.setpoint = {};
                for (int i = 0; i < sim_motor_ids.size(); i++) {
                    msg.motor.push_back(real_motor_ids[i]);
                    msg.setpoint.push_back(myoBrickEncoderTicksPerMeter(l_meter[sim_motor_ids[i]]));
//                    msg.setpoint[real_motor_ids[i]] = myoBrickEncoder0TicksPerMeter(l_meter[i]);
                }
                motor_command.publish(msg);
            }
        }
    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::Subscriber motor_status_sub, motor_state_sub;
    ros::ServiceServer init_pose;
    ros::AsyncSpinner *spinner;
    ros::ServiceClient motor_control_mode, motor_config, control_mode;

    map<int,int> pos, initial_pos;
    bool motor_status_received;
#ifdef LEGACY
    vector<uint8_t> real_motor_ids = {0,1,2,3,4,5,6,7};
#else
    vector<int> real_motor_ids = {0,1,2,3,4,5,6,7};
#endif
    vector<int> sim_motor_ids = {0,1,2,3,4,5,6,7};
    map<int,float> l_offset, position;
    vector<float> integral = {0,0,0,0,0,0,0,0};
    boost::shared_ptr<tf::TransformListener> listener;
};

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "arm_left");
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


    LeftArmTestbed robot(urdf, cardsflow_xml);

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
