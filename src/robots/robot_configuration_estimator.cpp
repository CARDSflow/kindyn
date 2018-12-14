// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

// std
#include <iostream>
#include "kindyn/robot.hpp"
#include <std_msgs/Float32.h>

using namespace Eigen;
using namespace std;

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
        motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);
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
            forwardKinematics(0.000005);
    };

    /**
     * Sends motor commands to the real robot
     */
    void write(){
        roboy_middleware_msgs::MotorCommand msg;
        msg.id = 5;
        stringstream str;
        for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
            msg.motors.push_back(i);
            double l_change = l[i]-l_offset[i];
            msg.set_points.push_back(-msjEncoderTicksPerMeter(l_change)); //
            str << l_change << "\t";
        }
        ROS_INFO_STREAM_THROTTLE(1,str.str());
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

    void randomPoseRecord(int number_of_samples, vector<double> &qd_record, vector<double> &ld_record, vector<MatrixXd> &W_record){
        ros::Time start_time = ros::Time::now();
        double min[3] = {0,0,-1}, max[3] = {0,0,1};
        for(int i=0;i<limits[0].size();i++){
            if(limits[0][i]<min[0])
                min[0] = limits[0][i];
            if(limits[1][i]<min[1])
                min[1] = limits[1][i];
            if(limits[0][i]>max[0])
                max[0] = limits[0][i];
            if(limits[1][i]>max[1])
                max[1] = limits[1][i];
        }
        while((ros::Time::now()-start_time).toSec()<60){
            double q0 = rand()/(double)RAND_MAX*(max[0]-min[0])+min[0];
            double q1 = rand()/(double)RAND_MAX*(max[1]-min[1])+min[1];
            double q2 = rand()/(double)RAND_MAX*(max[2]-min[2])+min[2];
            if(pnpoly(limits[0],limits[1],q0,q1)){
                ROS_INFO("new pose");
                std_msgs::Float32 msg;
                msg.data = q0;
                sphere_axis0.publish(msg);
                msg.data = q1;
                sphere_axis1.publish(msg);
                msg.data = q2;
                sphere_axis2.publish(msg);
                ros::Time t0 = ros::Time::now();
                while((ros::Time::now()-t0).toSec()<3 && qd_record.size()<number_of_samples){
                    read();
//                    qd_record.push_back(qd);
//                    ld_record.push_back(Ld);
//                    W_record.push_back(W);
                    ros::spinOnce();
                }
            }
        }
    }

    bool external_robot_state; /// indicates if we get the robot state externally
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command, sphere_axis0, sphere_axis1, sphere_axis2; /// motor command publisher
    double l_offset[NUMBER_OF_MOTORS];
    vector<double> limits[3];
};

// Generic functor for Eigen Levenberg-Marquardt minimizer
template<typename _Scalar, int NX = Dynamic, int NY = Dynamic>
struct Functor {
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}

    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }

    int values() const { return m_values; }
};

struct RobotConfigurationEstimator : Functor<double> {

    RobotConfigurationEstimator(int cables, int number_of_samples):
            Functor<double>(3*cables, 3 * number_of_samples), number_of_samples(number_of_samples), cables(cables) {

    };

    /**
     * This is the function that is called in each iteration
     * @param x the pose vector (3 rotational 3 translational parameters)
     * @param fvec the error function (the difference between the sensor positions)
     * @return
     */
    int operator()(const VectorXd &x, VectorXd &fvec) const{

    };



    vector<VectorXd> qd_record, ld_record;

    bool external_robot_state; /// indicates if we get the robot state externally
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher sphere_axis0, sphere_axis1, sphere_axis2;
    vector<double> limits[3];
    int number_of_samples, cables;
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

int main(){
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

//    robot.randomPoseRecord();
//    NumericalDiff<RobotConfigurationEstimator> *numDiff1;
//    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<RobotConfigurationEstimator>, double> *lm;
//    numDiff1 = new NumericalDiff<RobotConfigurationEstimator>(robot);
//    lm = new LevenbergMarquardt<NumericalDiff<RobotConfigurationEstimator>, double>(*numDiff1);
//    lm->parameters.maxfev = 1000;
//    lm->parameters.xtol = 1e-10;
//    VectorXd x(10);
//    x << 0.0000001, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//    int ret = lm->minimize(x);
//    double error = lm->fnorm;

    ROS_INFO("TERMINATING...");

    update_thread.join();
    return 0;
}