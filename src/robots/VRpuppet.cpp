#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>

using namespace std;

#define SPINDLERADIUS 0.0085
#define meterPerDegree(degree) (degreesToRadians(degree)*(2.0*M_PI*SPINDLERADIUS))
#define degreePerMeter(meter) (radiansToDegrees(meter)/(2.0*M_PI*SPINDLERADIUS))

class VRpuppet: public cardsflow::kindyn::Robot{
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    VRpuppet(string urdf, string cardsflow_xml){
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "VRpuppet");
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        spinner.reset(new ros::AsyncSpinner(0));
        spinner->start();
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        for(auto ef:endeffectors) {
            motor_control_mode[ef] = nh->serviceClient<roboy_middleware_msgs::ControlMode>(
                    "/"+ ef + "/ControlMode");
            nh->getParam((ef+"/joints").c_str(),endeffector_jointnames[ef]);
        }
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        init(urdf,cardsflow_xml,joint_names);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);
        {
            roboy_middleware_msgs::ControlMode msg;
            msg.request.control_mode = DISPLACEMENT;
            msg.request.set_point = 200;
            msg.request.motor_id = {0, 1, 2, 3, 4, 5};
            ros::Duration d(5);
            bool success = false;
            while (!success && ros::ok()) {
                for (auto control:motor_control_mode) {
                    success = control.second.call(msg);
                    if (!success) {
                        ROS_WARN("failed to change control mode to DISPLACEMENT, trying again in 5 seconds");
                        d.sleep();
                    }
                }
            }

            ROS_INFO("setting muscles to DISPLACEMENT and sleeping for 5 seconds");
        }
        {
            while (!status_received && ros::ok()) {
                ROS_INFO_THROTTLE(5, "waiting for motor status from cage, did you start the rqt plugin?!");
            }
            bool success = false;
            roboy_middleware_msgs::ControlMode msg;
            msg.request.control_mode = POSITION;
            for (auto control:motor_control_mode) {
                success = control.second.call(msg);
                if (!success)
                    ROS_WARN("failed to change control mode to position");
            }
            update();

            for (int i = 0; i < 6; i++)
                l_offset[i] = l[i];
        }
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
        msg.id = 69;
        msg.motors = {0,1,2,3,4,5};
        stringstream str;
        float kp;
        nh->getParam("kp",kp);
        for(int i=0;i<6;i++){
            float setpoint = degreePerMeter((l[i]-l_offset[i])-l_target[i]);
            str << setpoint << "\t";
            msg.set_points.push_back(setpoint);
        }
        ROS_INFO_STREAM_THROTTLE(1,str.str());
        motor_command.publish(msg);
    };
    void MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg){
        status_received = true;
        for(int i=0;i<msg->position.size();i++){
            pos[i] = meterPerDegree(msg->position[i]);
            vel[i] = meterPerDegree(msg->velocity[i]);
            dis[i] = msg->displacement[i];
        }
    }
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::ServiceClient motor_config, sphere_left_axis0_params, sphere_left_axis1_params, sphere_left_axis2_params;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    map<string,ros::ServiceClient> motor_control_mode;
    vector<string> endeffectors = {"VRpuppet"};
    map<string, vector<string>> endeffector_jointnames;
    vector<int> l_offset = {0,0,0,0,0,0,0,0,0,0,0},
                pos = {0,0,0,0,0,0,0,0,0,0,0},
                vel = {0,0,0,0,0,0,0,0,0,0,0},
                dis = {0,0,0,0,0,0,0,0,0,0,0};
    bool external_robot_state; /// indicates if we get the robot state externally
    bool status_received = false;
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

    VRpuppet robot(urdf, cardsflow_xml);

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
