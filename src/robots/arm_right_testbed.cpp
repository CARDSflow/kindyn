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

//#define myoBrickMeterPerEncoder0Tick(encoderTicks) ((encoderTicks)/(512.0*4.0*53.0)*(M_PI*0.017))
//#define myoBrickEncoder0TicksPerMeter(meter) ((meter)*(512.0*4.0*53.0)/(M_PI*0.017))


using namespace std;

class RightArmTestbed: public cardsflow::vrpuppet::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    RightArmTestbed(string urdf, string cardsflow_xml){

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "right_arm_testbed");
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

//        motor_status_sub = nh->subscribe("/roboy/middleware/MotorStatus",1,&RightArmTestbed::MotorStatus,this);
        motor_state_sub = nh->subscribe("/roboy/middleware/MotorState",1,&RightArmTestbed::MotorState,this);
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        init_pose = nh->advertiseService("init_pose",&RightArmTestbed::initPose,this);
        motor_config = nh->serviceClient<roboy_middleware_msgs::MotorConfigService>( "/roboy/middleware/MotorConfig");
        control_mode = nh->serviceClient<roboy_middleware_msgs::ControlMode>( "/roboy/middleware/ControlMode");
        ROS_INFO_STREAM("Finished setup");
    };

    bool initPose(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res){
        initialized = false;

        ROS_INFO("changing control mode of icebus motors to PWM");
//        roboy_middleware_msgs::MotorConfigService msg;
//        msg.request.legacy = false;
//        msg.request.config.motor = {0,1,2,3,4,5,6,7};
//        msg.request.config.control_mode = {3,3,3,3,3,3,3,3};
//        msg.request.config.Kp = {1,1,1,1,1,1,1,1};
//        msg.request.config.PWMLimit = {500,500,500,500,500,500,500,500};
//        msg.request.config.setpoint = {150,150,150,150,150,150,150,150};
//        motor_config.call(msg);

        roboy_middleware_msgs::ControlMode msg;
        msg.request.control_mode = DIRECT_PWM;
        msg.request.set_point = -150;
        msg.request.motor_id = {0,1,2,3,4,5,6,7};
        control_mode.call(msg);

        ros::Time t0;
        t0= ros::Time::now();
        while((ros::Time::now()-t0).toSec()<5){
            ROS_INFO_THROTTLE(1,"waiting");
        }

        stringstream str;
        str << "saving position offsets:" << endl << "sim motor id   |  real motor id  |   position offset [ticks]  | length_sim[m] | length offset[m]" << endl;
        for (int i = 0; i<sim_motor_ids.size();i++) str << i << ": " << position[i] << ", ";
        str << endl;
        for(int i=0;i<sim_motor_ids.size();i++) {
            l_offset[i] = -l[sim_motor_ids[i]] + myoBrickMeterPerEncoderTicks(position[i]);
            str << sim_motor_ids[i] << "\t|\t" << real_motor_ids[i] << "\t|\t" << position[i] << "\t|\t" << l[sim_motor_ids[i]] << "\t|\t" << l_offset[i] << endl;
        }
        ROS_INFO_STREAM(str.str());

//        ROS_INFO("changing control mode of icebus motors to POSITION");
//        msg.request.legacy = false;
//        msg.request.config.motor = {0,1,2,3,4,5,6,7};
//        msg.request.config.control_mode = {0,0,0,0,0,0,0,0};
//        msg.request.config.Kp = {1,1,1,1,1,1,1,1};
//        msg.request.config.PWMLimit = {500,500,500,500,500,500,500,500};
//        msg.request.config.setpoint = {(int)position[0],
//                                       (int)position[1],
//                                       (int)position[2],
//                                       (int)position[3],
//                                       (int)position[4],
//                                       (int)position[5],
//                                       (int)position[6],
//                                       (int)position[7]};
//        motor_config.call(msg);
        msg.request.control_mode = ENCODER0_POSITION;
        msg.request.set_point = 0;
        msg.request.motor_id = {0,1,2,3,4,5,6,7};
        control_mode.call(msg);

        ROS_INFO("pose init done");
        initialized = true;
        return true;
    }

    float myoBrickMeterPerEncoderTicks(int motor_pos) {
        return (motor_pos)/(512.0*4.0*53.0)*(M_PI*0.0165);
    }

    int myoBrickEncoderTicksPerMeter(float meter){
        return (meter)*(512.0*4.0*53.0)/(M_PI*0.0165);
    }

    void MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg){
        for (int i=0;i<8;i++) {
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
            vector<double> l_meter;
            double Kp_dl = 0, Ki_dl = 0, Kp_disp = 0;
            nh->getParam("Kp_dl",Kp_dl);
            nh->getParam("Ki_dl",Ki_dl);

            for (int i = 0; i < sim_motor_ids.size(); i++) {
                float error = l[sim_motor_ids[i]] - l_target[sim_motor_ids[i]];
                integral[i] += Ki_dl * error;
                if(integral[i]>0.01)
                    integral[i] = 0.01;
                if(integral[i]<-0.01)
                    integral[i] = -0.01;
                if(Ki_dl==0){
                    integral[i] = 0;
                }
                l_meter.push_back(
                        (l_offset[i] + l_target[i]) +
                        Kp_dl*error + integral[i]
                );
                str << sim_motor_ids[i] << "\n" << l_meter[i] << "\n";
                str << myoBrickEncoderTicksPerMeter(l_meter[i]) << "\n" << error << "\n";
            }
            str << endl;
            ROS_INFO_STREAM_THROTTLE(1,str.str());
            {
                roboy_middleware_msgs::MotorCommand msg;
                msg.legacy = false;
                msg.motor = {};
                msg.setpoint = {};
                for (int i = 0; i < 8; i++) {
                    msg.motor.push_back(real_motor_ids[i]);
                    msg.setpoint.push_back(myoBrickEncoderTicksPerMeter(l_meter[i]));
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
    vector<int> real_motor_ids = {0,1,2,3,4,5,6,7}, sim_motor_ids = {0,1,2,3,4,5,6,7};
    map<int,double> l_offset, position;
    vector<float> integral = {0,0,0,0,0,0,0,0};
    boost::shared_ptr<tf::TransformListener> listener;
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


    RightArmTestbed robot(urdf, cardsflow_xml);

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
