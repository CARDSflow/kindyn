#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <common_utilities/CommonDefinitions.h>

using namespace std;

class RoboyXylophone: public cardsflow::kindyn::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    RoboyXylophone(string urdf, string cardsflow_xml){
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "roboy_xylophone");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        init(urdf,cardsflow_xml,joint_names);
        update();
        for(int i=0;i<number_of_cables;i++)
            l_offset.push_back(l[i]);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);
    };
    /**
     * Updates the robot model and integrates the robot model using the forwardKinematics function
     * with a small step length
     */
    void read(){
        update();
        if(!external_robot_state)
            forwardKinematics(0.0001);
    };
    /**
     * Sends motor commands to the real robot
     */
    void write(){
        roboy_middleware_msgs::MotorCommand msg;
        msg.id = 5;
        for(int i=0;i<number_of_cables;i++){
            msg.motors.push_back(i);
            msg.set_points.push_back(l_target[i]);
        }
//        for(int i=0;i<number_of_cables;i++){
//            msg.motors.push_back(i);
//            msg.set_points.push_back(myoMuscleEncoderTicksPerMeter(l[i]-l_offset[i]));
//        }
        motor_command.publish(msg);
    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    bool external_robot_state; /// indicates if we get the robot state externally
    vector<double> l_offset;
};

/**
 * controller manager update thread. Here you can define how fast your controllers should run
 * @param cm pointer to the controller manager
 */
void update(controller_manager::ControllerManager *cm) {
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(1000); // changing this value affects the control speed of your running controllers
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

    RoboyXylophone robot(urdf, cardsflow_xml);

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
