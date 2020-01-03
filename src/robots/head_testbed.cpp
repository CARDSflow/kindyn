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
#include <common_utilities/UDPSocket.hpp>

#define SPINDLE_RADIUS 0.0055
#define m3MeterPerEncoderTick(encoderTicks) ((encoderTicks/360.0f)*(2.0*M_PI*SPINDLE_RADIUS))
#define m3EncoderTickPerMeter(meter) ((2.0*M_PI*SPINDLE_RADIUS))

#define POSITION 0
#define VELOCITY 1
#define DISPLACEMENT 2

using namespace std;

class HeadTestbed: public cardsflow::vrpuppet::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    HeadTestbed(string urdf, string cardsflow_xml){

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

        init_pose = nh->advertiseService("init_pose",&HeadTestbed::initPose,this);
        motor_status = nh->advertise<roboy_middleware_msgs::MotorStatus>("/roboy/middleware/m3/MotorStatus",1);

        uint32_t ip;
        inet_pton(AF_INET, "192.168.255.255", &ip);
        udp.reset(new UDPSocket(8000));
        udp_command.reset(new UDPSocket(8001));
        udp_thread.reset(new std::thread(&HeadTestbed::receiveStatusUDP, this));
        udp_thread->detach();

        ROS_INFO("waiting a bit for new motors");
        ros::Duration d(3);
        d.sleep();

        ROS_INFO_STREAM("Finished setup");
    };

    bool initPose(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res){
        initialized = false;
        for(auto m:ip_address){
            Kp[m.first] = 1;
            Kd[m.first] = 0;
            Ki[m.first] = 0;
            control_mode[m.first] = DISPLACEMENT;
            set_points[m.first] = 500;
        }
        ROS_INFO("changing to displacement, setpoint 500");
        controlModeChanged();
        ROS_INFO("waiting 5 sec");
        ros::Duration d(3);
        d.sleep();
        ROS_INFO("changing to position control");
        stringstream str;
        str << "saving position offsets:" << endl << "sim motor id   |  real motor id  |   position offset [ticks]  | length_sim[m] | length offset[m]" << endl;
        for (int i = 0; i<sim_motor_ids.size();i++) str << i << ": " << motor_position[i] << ", ";
        str << endl;
        for(auto m:ip_address) {
            control_mode[m.first] = POSITION;
            l_offset[m.first] = l[sim_motor_ids[m.first]] + m3MeterPerEncoderTick(motor_position[m.first]);
            str << sim_motor_ids[m.first] << "\t|\t" << real_motor_ids[m.first] << "\t|\t" << motor_position[m.first] << "\t|\t" << l[sim_motor_ids[m.first]] << "\t|\t" << l_offset[m.first] << endl;
        }
        ROS_INFO_STREAM(str.str());
        controlModeChanged();

        ROS_INFO("pose init done");
        initialized = true;
        return true;
    }

    void receiveStatusUDP() {
        ROS_INFO("start receiving udp");
        ros::Time t0 = ros::Time::now(), t1;
        while (ros::ok()) {
            int bytes_received = udp->receiveUDPFromClient();
            if (bytes_received == 20) {
                t1 = ros::Time::now();
                ros::Duration d = (t1-t0);
                t0 = t1;
                float hz = d.toSec();
                int approx_hz = hz;
                ROS_INFO_THROTTLE(60,"receiving motor status at %f Hz", hz);
                int motor = udp->buf[0];
                auto it = ip_address.find(motor);
                if (it == ip_address.end()) {;
                    char IP[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &udp->client_addr.sin_addr, IP, INET_ADDRSTRLEN);
                    ROS_INFO("new motor %d %s", motor, IP);
                    ip_address[motor] = IP;
                    continue;
                }
                int32_t pos = (int32_t) ((uint8_t) udp->buf[7] << 24 | (uint8_t) udp->buf[6] << 16 |
                                         (uint8_t) udp->buf[5] << 8 | (uint8_t) udp->buf[4]);
                int32_t vel = (int32_t) ((uint8_t) udp->buf[11] << 24 | (uint8_t) udp->buf[10] << 16 |
                                         (uint8_t) udp->buf[9] << 8 | (uint8_t) udp->buf[8]);
                int32_t dis = (int32_t) ((uint8_t) udp->buf[15] << 24 | (uint8_t) udp->buf[14] << 16 |
                                         (uint8_t) udp->buf[13] << 8 | (uint8_t) udp->buf[12]);
                int32_t pwm = (int32_t) ((uint8_t) udp->buf[19] << 24 | (uint8_t) udp->buf[18] << 16 |
                                         (uint8_t) udp->buf[17] << 8 | (uint8_t) udp->buf[16]);
                motor_position[motor] = pos;
                motor_velocity[motor] = vel;
                motor_displacement[motor] = dis;
                motor_pwm[motor] = pwm;

                roboy_middleware_msgs::MotorStatus msg;
                msg.id = 69;
                for(auto m:ip_address){
                    msg.position.push_back(motor_position[m.first]);
                    msg.velocity.push_back(motor_velocity[m.first]);
                    msg.displacement.push_back(motor_displacement[m.first]);
                    msg.pwm_ref.push_back(motor_pwm[m.first]);
                }
                motor_status.publish(msg);
            }
        }
        ROS_INFO("stop receiving udp");
    }

    void sendCommand(){
        udp_command->client_addr.sin_port = htons(8001);
        udp_command->numbytes = 10;
        for (auto m:ip_address) {
            mempcpy(udp_command->buf, &set_points[m.first], 4);
            mempcpy(&udp_command->buf[4], &m.first, 4);
            udp_command->numbytes = 10;
            udp_command->client_addr.sin_addr.s_addr = inet_addr(m.second.c_str());
            udp_command->sendUDPToClient();
        }
    }

    void controlModeChanged(){
        for(auto m:ip_address) {
            mempcpy(&udp_command->buf[0], &Kd, 4);
            mempcpy(&udp_command->buf[4], &Ki, 4);
            mempcpy(&udp_command->buf[8], &Kp, 4);
            mempcpy(&udp_command->buf[12], &control_mode[m.first], 4);
            mempcpy(&udp_command->buf[16], &m.first, 4);

            udp_command->client_addr.sin_port = htons(8001);
            udp_command->numbytes = 20;
            udp_command->client_addr.sin_addr.s_addr = inet_addr(m.second.c_str());
            udp_command->sendUDPToClient();

            if (control_mode[m.first] == POSITION) {
                udp_command->numbytes = 10;
                set_points[m.first] = motor_position[m.first];
                mempcpy(udp_command->buf, &set_points[m.first], 4);
                mempcpy(&udp_command->buf[4], &m.first, 4);
                udp_command->sendUDPToClient();
            }else{
                udp_command->numbytes = 10;
                mempcpy(udp_command->buf, &set_points[m.first], 4);
                mempcpy(&udp_command->buf[4], &m.first, 4);
                udp_command->sendUDPToClient();
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
        if(!initialized){
            ROS_INFO_THROTTLE(1,"waiting for init_pose service call");
        }else{
            stringstream str;
            vector<double> l_meter;
            for (int i = 0; i < sim_motor_ids.size(); i++) {
                l_meter.push_back(-l_target[i] + l_offset[i]);
                str << sim_motor_ids[i] << "\t" << l_meter[i] << "\t";
                str << m3EncoderTickPerMeter(l_meter[i]) << "\t";
            }
            str << endl;
//            {
//                roboy_middleware_msgs::MotorCommand msg;
//                msg.legacy = true;
//                msg.motor = {};
//                msg.setpoint = {}; //.resize(4);
//                for (int i = 0; i < 8; i++) {
//                    msg.motor.push_back(real_motor_ids[i]);
//                    msg.setpoint.push_back(myoBrickEncoder0TicksPerMeter(l_meter[i]));
////                    msg.setpoint[real_motor_ids[i]] = myoBrickEncoder0TicksPerMeter(l_meter[i]);
//                }
//                motor_command.publish(msg);
//            }
        }
    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command, motor_status; /// motor command publisher
    ros::ServiceServer init_pose;
    ros::AsyncSpinner *spinner;
    ros::ServiceClient motor_control_mode, motor_config;

    map<int,int> Kp,Ki,Kd;
    map<int,int> control_mode;
    map<int,int> pos, initial_pos;
    bool motor_status_received;
    vector<int> real_motor_ids = {0,1,2,3,4,5}, sim_motor_ids = {0,1,2,3,4,5};
    boost::shared_ptr<tf::TransformListener> listener;
    int counter = 0;
    map<int,int> set_points;
    map<int,double> l_offset, motor_position, motor_velocity, motor_displacement, motor_pwm;
    map<int,string> ip_address;
    UDPSocketPtr udp, udp_command;
    boost::shared_ptr<thread> udp_thread;
};

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "head_testbed");
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


    HeadTestbed robot(urdf, cardsflow_xml);

    if (nh.hasParam("simulated")) {
      nh.getParam("simulated", robot.simulated);
    }

    ros::Rate rate(30);
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