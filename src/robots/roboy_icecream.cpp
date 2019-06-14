#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_simulation_msgs/Tendon.h>
#include <common_utilities/CommonDefinitions.h>
#include "sensor_msgs/JointState.h"
//#include <roboy_simulation_msgs/CardsflowStatus.h>

#include <mutex>

#define NUMBER_OF_MOTORS 28
#define SPINDLERADIUS 0.0045
#define msjMeterPerEncoderTick(encoderTicks) (((encoderTicks)/(4096.0)*(2.0*M_PI*SPINDLERADIUS)))
#define msjEncoderTicksPerMeter(meter) ((meter)*(4096.0)/(2.0*M_PI*SPINDLERADIUS))

using namespace std;

class RoboyIcecream: public cardsflow::kindyn::Robot{

    private:
    mutex mux;
    ros::Subscriber gazebo_robot_state_sub, tendon_states_sub, motor_status_sub;
    ros::Publisher cardsflow_status_pub;
    void JointStatesCallback(const sensor_msgs::JointStateConstPtr &msg) {
        for(int i=0; i<msg->name.size(); i++) {
            auto idx =  distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), msg->name[i]));
            if(idx == -1)
                continue;
            q[idx] = msg->position[i];
            qd[idx] = msg->velocity[i];
        }
    }

    void TendonStatesCallback(const roboy_simulation_msgs::TendonConstPtr &msg) {
        for (int i=0; i<msg->ld.size(); i++) {
            l[i] = msg->l[i];
            Ld_curr[i] = msg->ld[i];
        }
    }

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
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
//        cardsflow_status_pub = nh->advertise<roboy_simulation_msgs::CardsflowStatus>("/cardsflow/status", 1);
        // first we retrieve the active joint names from the parameter server
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        // then we initialize the robot with the cardsflow xml and the active joints
        init(urdf,cardsflow_xml,joint_names);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);

        if (external_robot_state) {
            gazebo_robot_state_sub = nh->subscribe("/joint_states", 1, &RoboyIcecream::JointStatesCallback, this);
//
        }
        tendon_states_sub = nh->subscribe("/tendon_states", 1, &RoboyIcecream::TendonStatesCallback, this);
        update();
        for(int i=0;i<NUMBER_OF_MOTORS;i++)
            l_offset[i] = l[i];
    };

    /**
     * Updates the robot model and if we do not use gazebo for simulation, we integrate using the forwardKinematics function
     * with a small step length
     */
    void read(){

        update();
        auto dt = 0.00005;
//        ROS_INFO_STREAM_THROTTLE(1, Ld);
        if(!external_robot_state) // else q, qd, l are filled by gazebo_robot_state_sub & tendon_states_sub
            forwardKinematics(dt);
        else {
            for(int i = 0; i<endeffectors.size();i++) {
                int dof_offset = endeffector_dof_offset[i];
                Ld.setZero();
                for (int j = dof_offset; j < endeffector_number_of_dofs[i] + dof_offset; j++) {
                    Ld -= ld[j];
                }
            }
        }
    };

    /**
     * Sends motor commands to the real robot
     */
    void write(){
        roboy_middleware_msgs::MotorCommand msg;
//        roboy_simulation_msgs::CardsflowStatus cf_msg;
        double coef = sqrt((q-q_target).norm());
        msg.id = 3;
        for (auto idx: sim_motors["head"]) {
            msg.motors.push_back(idx);
            msg.set_points.push_back(coef*myoMuscleEncoderTicksPerMeter(l_offset[idx]-l[idx])));
        }
//        for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
//            msg.motors.push_back(i);
//
//
//            msg.set_points.push_back(myoMuscleEncoderTicksPerMeter(Ld[i]));
////            msg.set_points.push_back(coef*myoMuscleEncoderTicksPerMeter(l_offset[i]-l[i]));
//
////            cf_msg.current.push_back(Ld_curr[i]);
////            cf_msg.target.push_back(Ld[i]);
//
//        }
        ROS_INFO_STREAM_THROTTLE(1, "Ld_target " << Ld.transpose().format(fmt));
//        ROS_INFO_STREAM_THROTTLE(1,str.str());
        motor_command.publish(msg);
//        cardsflow_status_pub.publish(cf_msg);
    };
    bool external_robot_state; /// indicates if we get the robot state externally
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    double l_offset[NUMBER_OF_MOTORS];
    map<string,vector<short unsigned int>> sim_motors = {
            {"head",{18,19,20,21,22,23}},
            {"shoulder_left",{0,1,2,3,4,5,6,7,8,9,10}},
            {"shoulder_right",{0,1,2,3,4,5,6,7,8,9,11}},
            {"spine_right",{11,10,13,14,12,9}}
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
