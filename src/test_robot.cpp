#include "kindyn/robot.hpp"


int main(int argc, char *argv[]){
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

    int i = 0;
    while(ros::ok()){
        robot.update(0.0001);
        robot.updateController();
        i++;
    }

    return 0;
}