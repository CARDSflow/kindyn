//
// Created by roboy on 01.07.21.
//

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
