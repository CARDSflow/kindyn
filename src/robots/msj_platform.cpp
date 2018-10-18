#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_communication_middleware/MotorCommand.h>

using namespace std;

class Robot: public cardsflow::kindyn::Robot{
public:
    Robot(string urdf, string cardsflow_xml){
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "msj_platform");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        motor_command = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand",1);
        init(urdf,cardsflow_xml);
    };
    void read(){
        ros::Time t0 = ros::Time::now();
        update();
        ROS_INFO_THROTTLE(1,"update takes %f seconds", (ros::Time::now()-t0).toSec());
        t0 = ros::Time::now();
        forwardKinematics(0.000001);
        ROS_INFO_THROTTLE(1,"forwardKinematics takes %f seconds", (ros::Time::now()-t0).toSec());
    };

    void write(){
        roboy_communication_middleware::MotorCommand msg;
        msg.id = 5;
        for (int i = 0; i < number_of_cables; i++) {
            msg.motors.push_back(i);
            msg.setPoints.push_back(
                    512 + (l[i] / (2.0 * M_PI * 0.016 * (301.0 / 1024.0 / 360.0)))); //
        }
        motor_command.publish(msg);
    };
    ros::NodeHandlePtr nh;
    ros::Publisher motor_command;
};

void update(controller_manager::ControllerManager *cm) {
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(100);
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
        ros::init(argc, argv, "test_robot");
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

    Robot robot(urdf, cardsflow_xml);

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
