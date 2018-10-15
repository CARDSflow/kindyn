#include "kindyn/robot.hpp"
#include <thread>

using namespace std;

void update(controller_manager::ControllerManager *cm) {
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10);
    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        cm->update(time, period);
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "test_robot", ros::init_options::NoSigintHandler);
    }
    if(argc!=3){
        ROS_ERROR("USAGE: rosrun kindyn test_robot path_to_urdf path_to_viapoints_xml");
    }
    ROS_INFO("%s %s", argv[1], argv[2]);

    cardsflow::kindyn::Robot robot(argv[1], argv[2]);

    controller_manager::ControllerManager cm(&robot);

    // we need an additional update thread, otherwise the controllers won't switch
    thread update_thread(update, &cm);
    update_thread.detach();

    ROS_INFO("STARTING ROBOT MAIN LOOP...");

    while(ros::ok()){
        robot.update(0.0001);
        robot.updateController();
        ros::spinOnce();
    }

    ROS_INFO("TERMINATING...");

    update_thread.join();

    return 0;
}
