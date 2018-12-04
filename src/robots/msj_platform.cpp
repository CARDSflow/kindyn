#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_communication_middleware/MotorCommand.h>

#define NUMBER_OF_MOTORS 8
#define SPINDLERADIUS 0.00575
#define msjMeterPerEncoderTick(encoderTicks) (((encoderTicks)/(4096.0)*(2.0*M_PI*SPINDLERADIUS)))
#define msjEncoderTicksPerMeter(meter) ((meter)*(4096.0)/(2.0*M_PI*SPINDLERADIUS))

using namespace std;

class MsjPlatform: public cardsflow::kindyn::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    MsjPlatform(string urdf, string cardsflow_xml){
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "msj_platform");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        motor_command = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand",1);
        // first we retrieve the active joint names from the parameter server
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        // then we initialize the robot with the cardsflow xml and the active joints
        init(urdf,cardsflow_xml,joint_names);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);
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
        if(!external_robot_state)
            forwardKinematics(0.00001);
    };

    /**
     * Sends motor commands to the real robot
     */
    void write(){
        roboy_communication_middleware::MotorCommand msg;
        msg.id = 5;
        stringstream str;
        for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
            msg.motors.push_back(i);
            double l_change = l[i]-l_offset[i];
            msg.setPoints.push_back(-msjEncoderTicksPerMeter(l_change)); //
            str << l_change << "\t";
        }
        ROS_INFO_STREAM_THROTTLE(1,str.str());
        motor_command.publish(msg);
    };
    bool external_robot_state; /// indicates if we get the robot state externally
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    double l_offset[NUMBER_OF_MOTORS];
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

    MsjPlatform robot(urdf, cardsflow_xml);

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
