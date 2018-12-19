#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_simulation_msgs/GymStep.h>
#include <roboy_simulation_msgs/GymReset.h>

#define SPINDLERADIUS 0.0045
#define FS5103R_MAX_SPEED (2.0*M_PI/0.9) // radian per second

#define FS5103R_FULL_SPEED_BACKWARDS 400.0
#define FS5103R_STOP 375.0
#define FS5103R_FULL_SPEED_FORWARDS 350.0

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
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
        gym_step = nh->advertiseService("/gym_step", &MsjPlatform::GymStepService,this);
        gym_reset = nh->advertiseService("/gym_reset", &MsjPlatform::GymResetService,this);
        // first we retrieve the active joint names from the parameter server
        vector<string> joint_names;
        nh->getParam("joint_names", joint_names);
        // then we initialize the robot with the cardsflow xml and the active joints
        init(urdf,cardsflow_xml,joint_names);
        // if we do not get the robot state externally, we use the forwardKinematics function to integrate the robot state
        nh->getParam("external_robot_state", external_robot_state);
        string path = ros::package::getPath("robots");
        path+="/msj_platform/joint_limits.txt";
        FILE*       file = fopen(path.c_str(),"r");
        cout << "file:" << file;
        if (NULL != file) {
            fscanf(file, "%*[^\n]\n", NULL);
            cout << "fscanf"  << fscanf(file, "%*[^\n]\n", NULL)<< endl;
            float qx,qy;
            int i =0;
            cout << "fscanf file qx qy: "<< fscanf(file,"%f %f\n",&qx,&qy) << endl;
            while(fscanf(file,"%f %f\n",&qx,&qy) == 2){
                limits[0].push_back(qx);
                limits[1].push_back(qy);
                cout << qx << "\t" << qy << endl;
                i++;
            }
            printf("read %d joint limit values\n", i);
        }else{
            cout << "could not open " << path << endl;
        }
    };

    /**
     * Updates the robot model and if we do not use gazebo for simulation, we integrate using the forwardKinematics function
     * with a small step length
     */
    void read(){
        update();
        if(!external_robot_state)
            forwardKinematics(0.000001);
    };

    /**
     * Converts tendon length chages from the cable model to pwm commands of the real robot
     * @param meterPerSecond tendon length change
     * @return pwm
     */
    int meterPerSecondToServoSpeed(double meterPerSecond){
        double radianPerSecond = meterPerSecond/(2.0 * M_PI * SPINDLERADIUS);
        double pwm = radianPerSecond;
        if(pwm<=-1){
            return FS5103R_FULL_SPEED_BACKWARDS;
        }else if(pwm>-1 && pwm<1){
            return -pwm*(FS5103R_FULL_SPEED_BACKWARDS-FS5103R_STOP)+FS5103R_STOP;
        }else{
            return FS5103R_FULL_SPEED_FORWARDS;
        }
    }

    /**
     * Sends motor commands to the real robot
     */
    void write(){
        roboy_middleware_msgs::MotorCommand msg;
        msg.id = 5;
        stringstream str;
        for (int i = 0; i < number_of_cables; i++) {
            msg.motors.push_back(i);
            msg.set_points.push_back(meterPerSecondToServoSpeed(Ld[i])); //
            str << meterPerSecondToServoSpeed(Ld[i]) << "\t" << Ld[i] << "\t";
        }
		//ROS_INFO_STREAM_THROTTLE(1,str.str());
        motor_command.publish(msg);
    };

    int pnpoly(vector<double> limits_x, vector<double> limits_y, double testx, double testy){
        int i, j, c = 0;
        for (i = 0, j = limits_x.size()-1; i < limits_x.size(); j = i++) {
            if ( ((limits_y[i]>testy) != (limits_y[j]>testy)) &&
                 (testx < (limits_x[j]-limits_x[i]) * (testy-limits_y[i]) / (limits_y[j]-limits_y[i]) + limits_x[i]) )
                c = !c;
        }
        return c;
    }

    bool GymStepService(roboy_simulation_msgs::GymStep::Request &req,
                        roboy_simulation_msgs::GymStep::Response &res){
        
        if(req.set_points.size() != 0){ //If no set_point set then jut return observation.
        	ROS_INFO("Gymstep is called");
        	update();
	        for(int i=0; i< number_of_cables; i++){
	        	//Set the commanded tendon velocity from RL agent to simulation 
	        	Ld[i] = req.set_points[i]; 	
	        }
	        if(!external_robot_state)
	            forwardKinematics(req.step_size);

	        //ROS_INFO_STREAM_THROTTLE(5, "Ld = " << Ld.format(fmt));
	        write();
	        ROS_INFO("Gymstep is done");
	    }
        for(int i=0; i< number_of_dofs; i++ ){
        	res.q.push_back(q[i]);
        	res.qdot.push_back(qd[i]);
        }
        if(pnpoly(limits[0],limits[1],q[0],q[1])){
            //task space is feasible
            res.feasible = true;
        }
        else{
            //task space is not feasible
            res.feasible = false;
        }
        return true;
    }
    bool GymResetService(roboy_simulation_msgs::GymReset::Request &req,
                        roboy_simulation_msgs::GymReset::Response &res){
    	ROS_INFO("Gymreset is called");      
    	integration_time = 0.0;
        for(int i=0; i< number_of_dofs; i++){
	       	//Set the commanded tendon velocity from RL agent to simulation 
	       	//Set the joint states to arrange the initial condition or reset it. Not the q and qdot
			joint_state[i][0] = 0.0;		//Velocity of ith joint
			joint_state[i][1] = 0.0;		//Position of ith joint
            q[i] = 0.0;
            qd[i] = 0.0;
	    }

	    for(int i=0; i< number_of_cables; i++){
	        //Set the commanded tendon velocity from RL agent to simulation 
	        motor_state[i][0] =0.0; 	//Length of the ith cable
			motor_state[i][1] = 0.0;	//Velocity of the ith cable
	    }

	 	update();

        //ROS_INFO_STREAM_THROTTLE(5, "q = \n" << q.format(fmt));
        for(int i=0; i< number_of_dofs; i++ ){
        	res.q.push_back(q[i]);
        	res.qdot.push_back(qd[i]);
        }
        ROS_INFO("Gymreset is done");
        return true;
    }
    bool external_robot_state; /// indicates if we get the robot state externally
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command; /// motor command publisher
    ros::ServiceServer gym_step; //OpenAI Gym training environment step function, ros service instance
    ros::ServiceServer gym_reset; //OpenAI Gym training environment reset function, ros service instance
    vector<double> limits[3];
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
        //robot.read();
        //robot.write();
        ros::spinOnce();
    }

    ROS_INFO("TERMINATING...");

    update_thread.join();

    return 0;
}
