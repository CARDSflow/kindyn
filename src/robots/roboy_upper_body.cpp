#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/ControlMode.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_communication_control/SetControllerParameters.h>

using namespace std;

class RoboyUpperBody: public cardsflow::kindyn::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    RoboyUpperBody(string urdf, string cardsflow_xml){
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "roboy_upper_body");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        motor_command = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand",1);
        motor_control_mode = nh->serviceClient<roboy_communication_middleware::ControlMode>("/roboy/shoulder_left/middleware/ControlMode");
//        motor_config = nh->serviceClient<roboy_communication_middleware::MotorC>("/roboy/shoulder_left/middleware/ControlMode");
        sphere_left_axis0_params = nh->serviceClient<roboy_communication_control::SetControllerParameters>("/sphere_left_axis0/sphere_left_axis0/params");
        sphere_left_axis1_params = nh->serviceClient<roboy_communication_control::SetControllerParameters>("/sphere_left_axis1/sphere_left_axis1/params");
        sphere_left_axis2_params = nh->serviceClient<roboy_communication_control::SetControllerParameters>("/sphere_left_axis2/sphere_left_axis2/params");
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        init(urdf,cardsflow_xml,joint_names);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);
        roboy_communication_middleware::ControlMode msg;
        msg.request.control_mode = VELOCITY;
        msg.request.setPoint = 0;
        if(!motor_control_mode.call(msg))
            ROS_WARN("failed to change control mode to velocity");
    };
    /**
     * Updates the robot model and integrates the robot model using the forwardKinematics function
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
        msg.id = SHOULDER_LEFT;
        msg.motors = left_arm_motors;
        stringstream str;
        for (int i = 0; i < left_arm_motors.size(); i++) {
            double ld_meter = -ld[0][left_arm_motors[i]];
            str << ld_meter << "\t";
            msg.setPoints.push_back(myoMuscleEncoderTicksPerMeter(ld_meter)); //
        }
        str << endl;
        ROS_INFO_STREAM_THROTTLE(1,str.str());
        motor_command.publish(msg);
    };
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::ServiceClient motor_control_mode, motor_config, sphere_left_axis0_params, sphere_left_axis1_params, sphere_left_axis2_params;
    bool external_robot_state; /// indicates if we get the robot state externally
    vector<short unsigned int> left_arm_motors = {0,1,2,3,4,5,6,7,8};
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

    RoboyUpperBody robot(urdf, cardsflow_xml);

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
