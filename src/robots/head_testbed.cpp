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

#define POSITION 0
#define VELOCITY 1
#define DISPLACEMENT 2
#define DIRECT_PWM 3

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
            ros::init(argc, argv, "head_testbed");
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
        inet_pton(AF_INET, "192.168.2.255", &ip);
        udp.reset(new UDPSocket(8000));
        udp_command.reset(new UDPSocket(8001));
        udp_thread.reset(new std::thread(&HeadTestbed::receiveStatusUDP, this));
        udp_thread->detach();

        bool wait_for_all_motors = false;
        nh->getParam("wait_for_all_motors", wait_for_all_motors);

        while(ip_address.size()!=6 && ros::ok() && wait_for_all_motors){
            ROS_INFO_THROTTLE(1,"waiting for all motors, have %d/6", ip_address.size());
        }

        nh->getParam("external_robot_state", external_robot_state);

        ROS_INFO_STREAM("Finished setup");
    };

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
                for(auto &m:ip_address){
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

    void sendCommand(int m){
        udp_command->client_addr.sin_port = htons(8001);
        udp_command->numbytes = 10;
        mempcpy(udp_command->buf, &set_points[m], 4);
        mempcpy(&udp_command->buf[4], &m, 4);
        udp_command->numbytes = 10;
        udp_command->client_addr.sin_addr.s_addr = inet_addr(ip_address[m].c_str());
        udp_command->sendUDPToClient();
    }

    void controlModeChanged(){
        for(auto &m:ip_address) {
            ROS_INFO("updating control_mode of motor %d with IP %s to control_mode %d",m.first,m.second.c_str(),control_mode[m.first]);
            mempcpy(&udp_command->buf[0], &Kd[m.first], 4);
            mempcpy(&udp_command->buf[4], &Ki[m.first], 4);
            mempcpy(&udp_command->buf[8], &Kp[m.first], 4);
            mempcpy(&udp_command->buf[12], &control_mode[m.first], 4);
            mempcpy(&udp_command->buf[16], &m.first, 4);

            udp_command->client_addr.sin_port = htons(8001);
            udp_command->numbytes = 20;
            udp_command->client_addr.sin_addr.s_addr = inet_addr(m.second.c_str());
            udp_command->sendUDPToClient();

            ros::Duration d(0.1);
            d.sleep();

            if (control_mode[m.first] == POSITION) {
                set_points[m.first] = motor_position[m.first];
                sendCommand(m.first);
            }else{
                sendCommand(m.first);
            }
        }
    }

    float m3MeterPerEncoderTick(int motor_pos, float spindelRadius) {
        return (motor_pos / 360.0f) * (2.0 * M_PI * spindelRadius);
    }

    int m3EncoderTickPerMeter(float meter,float spindelRadius){
        return meter/(2.0*M_PI*spindelRadius)*360;
    }

    bool initPose(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res){
        initialized = false;
        for(auto &m:ip_address){
            Kp[m.first] = 1;
            Kd[m.first] = 0;
            Ki[m.first] = 0;
            control_mode[m.first] = DIRECT_PWM;
            set_points[m.first] = -100;
            integral[m.first] = 0;
        }
        ROS_INFO("changing to displacement, setpoint 500");
        controlModeChanged();
        ROS_INFO("waiting 3 sec");
        ros::Duration d(3);
        d.sleep();
        ROS_INFO("changing to position control");
        stringstream str;
        str << "saving position offsets:" << endl << "sim motor id   |  real motor id  |   position offset [ticks]  | length_sim[m] | length offset[m]" << endl;
        for (int i = 0; i<sim_motor_ids.size();i++) str << i << ": " << motor_position[i] << ", ";
        str << endl;
        for(auto &m:ip_address) {
            Kp[m.first] = 300;
            Kd[m.first] = 0;
            Ki[m.first] = 10;
            control_mode[m.first] = POSITION;
            l_offset[m.first] = l[sim_motor_ids[m.first]] + m3MeterPerEncoderTick(motor_position[m.first],spindelRadius[m.first]);
            str << sim_motor_ids[m.first] << "\t|\t" << real_motor_ids[m.first] << "\t|\t" << motor_position[m.first] << "\t|\t" << l[sim_motor_ids[m.first]] << "\t|\t" << l_offset[m.first] << endl;
        }
        ROS_INFO_STREAM(str.str());
        controlModeChanged();

        ROS_INFO("pose init done");
        initialized = true;
        return true;
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
            double Kp_dl = 0, Ki_dl = 0, Kp_disp = 0;
            nh->getParam("Kp_dl",Kp_dl);
            nh->getParam("Ki_dl",Ki_dl);
            nh->getParam("Kp_disp",Kp_disp);
            stringstream str;
            vector<double> l_meter;
            str << "-------------------" << endl;
            char s[200];
            for (int i = 0; i < sim_motor_ids.size(); i++) {
                integral[i] += Ki_dl * (l_target[sim_motor_ids[i]]-l[sim_motor_ids[i]]);
                if(integral[i]>0.05)
                    integral[i] = 0.05;
                if(integral[i]<-0.05)
                    integral[i] = -0.05;
                l_meter.push_back(
                        -l_target[sim_motor_ids[i]] + l_offset[sim_motor_ids[i]]-
                        Kp_dl*(l_target[sim_motor_ids[i]]-l[sim_motor_ids[i]])  - integral[i] +
                                (400 - motor_displacement[sim_motor_ids[i]]) * Kp_disp
                );
                sprintf(s,"motor %d: %f meter, %d ticks, %d disp, %f disp_meter\n", sim_motor_ids[i],
                        l_meter[i],m3EncoderTickPerMeter(l_meter[i],spindelRadius[i]),
                        motor_displacement[sim_motor_ids[i]],
                        motor_displacement[sim_motor_ids[i]]/2000.0f*0.018f
                        );
                str << s;
            }
            str << endl;
            ROS_INFO_STREAM_THROTTLE(1,str.str());
            for(auto &m:ip_address){
//                float disp_error = 3000-motor_displacement[real_motor_ids[m.first]];
//                integral[real_motor_ids[m.first]]+=disp_error *0.1;
//                if(integral[real_motor_ids[m.first]]>200)
//                    integral[real_motor_ids[m.first]] = 200;
//                if(integral[real_motor_ids[m.first]]<-200)
//                    integral[real_motor_ids[m.first]] = -200;
                set_points[real_motor_ids[m.first]] = m3EncoderTickPerMeter(l_meter[m.first],spindelRadius[m.first]);
//                        +0.1*disp_error+integral[real_motor_ids[m.first]];
//                        m3EncoderTickPerMeter(motor_displacement[real_motor_ids[m.first]]/2000.0f*0.018f,spindelRadius[m.first]);
                sendCommand(m.first);
            }
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
    vector<float> spindelRadius = {0.005,0.006,0.009,0.006,0.009,0.005};
    vector<float> integral = {0,0,0,0,0,0};
    boost::shared_ptr<tf::TransformListener> listener;
    int counter = 0;
    map<int,int> set_points;
    map<int,double> l_offset;
    map<int,int> motor_position, motor_velocity, motor_displacement, motor_pwm;
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
