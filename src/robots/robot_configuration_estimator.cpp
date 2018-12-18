// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

// std
#include <iostream>
#include "kindyn/robot.hpp"
#include <std_msgs/Float32.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <tf/transform_broadcaster.h>

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
        sphere_axis0 = nh->advertise<std_msgs::Float32>("/sphere_axis0/sphere_axis0/target", 1);
        sphere_axis1 = nh->advertise<std_msgs::Float32>("/sphere_axis1/sphere_axis1/target", 1);
        sphere_axis2 = nh->advertise<std_msgs::Float32>("/sphere_axis2/sphere_axis2/target", 1);
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

        string path = ros::package::getPath("robots");
        path+="/msj_platform/joint_limits.txt";
        FILE*       file = fopen(path.c_str(),"r");
        if (NULL != file) {
            fscanf(file, "%*[^\n]\n", NULL);
            float qx,qy;
            int i =0;
            while(fscanf(file,"%f %f\n",&qx,&qy)==2){
                limits[0].push_back(qx);
                limits[1].push_back(qy);
                cout << qx << "\t" << qy << endl;
                i++;
            }
            printf("read %d joint limit values\n", i);
        }else{
            cout << "could not open " << path << endl;
        }

        pose_thread.reset(new std::thread(&MsjPlatform::randomPose, this));
        pose_thread->detach();
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

    void randomPose(){
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

        int sample = 0;
        while(ros::ok()){
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
                while((ros::Time::now()-t0).toSec()<3){
                    update();
                    forwardKinematics(0.0000001);
                }
                sample++;
            }
            ROS_INFO_STREAM(sample);
        }
    }

    bool external_robot_state; /// indicates if we get the robot state externally
    ros::NodeHandlePtr nh; /// ROS nodehandle
    ros::Publisher motor_command, sphere_axis0, sphere_axis1, sphere_axis2; /// motor command publisher
    double l_offset[NUMBER_OF_MOTORS];
    vector<double> limits[3];
    boost::shared_ptr<std::thread> pose_thread;
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

    RobotConfigurationEstimator(int number_of_samples, int number_of_cables, int number_of_links):
            Functor<double>(3*number_of_cables, 3*number_of_cables), number_of_samples(number_of_samples), number_of_cables(number_of_cables), number_of_links(number_of_links) {
    };

    /**
     * This is the function that is called in each iteration
     * @param x the pose vector (3 rotational 3 translational parameters)
     * @param fvec the error function (the difference between the sensor positions)
     * @return
     */
    int operator()(const VectorXd &x, VectorXd &fvec) const{
        MatrixXd V;
        V.resize(number_of_cables, 6 * number_of_links);
        V.setZero();
        fvec.setZero();
        V.setZero(number_of_cables, 6 * number_of_links);
        for (int cable = 0; cable < number_of_cables; cable++) {
            // V term associated with segment translation
            Vector3d segmentVector;
            Vector3d global_estimate(x[cable*3+0],x[cable*3+1],x[cable*3+2]);
            segmentVector =  robot->segments[cable][0].second->global_coordinates - global_estimate;
            segmentVector.normalize();

            // Total V term in translations
            Vector3d V_ijk_T = robot->world_to_link_transform.back().block(0, 0, 3, 3) * segmentVector;

            Vector3d V_itk_T = robot->segments[cable][0].second->local_coordinates.cross(V_ijk_T);

            V.block(cable, 6 * 6, 1, 3) = V.block(cable, 6 * 6, 1, 3) + V_ijk_T.transpose();
            V.block(cable, 6 * 6 + 3, 1, 3) =
                    V.block(cable, 6 * 6 + 3, 1, 3) + V_itk_T.transpose();
        }
        MatrixXd L = V * robot->W;
//        VectorXd ld_error = L*robot->qd-robot->Ld; // TODO adapt to multiple endeffectors
//        for(int j=0;j<number_of_cables;j++) {
//            fvec[j*3 ] += abs(ld_error[j]);
//            fvec[j*3 + 1] += abs(ld_error[j]);
//            fvec[j*3 + 2] += abs(ld_error[j]);
//        }

        static int counter = 0;
        if((counter++%100)==0){
            ROS_INFO_STREAM_THROTTLE(5,fvec.norm() << endl << x.transpose());
            for(int cable=0;cable<number_of_cables;cable++) {
                char str[100];
                sprintf(str, "cable_estimate_%d", cable);
                geometry_msgs::Pose msg;
                msg.position.x = x[cable*3];
                msg.position.y = x[cable*3+1];
                msg.position.z = x[cable*3+2];
                robot->publishTransform("world",str,msg);
            }
        }
    };

    static cardsflow::kindyn::Robot *robot;

    int number_of_samples, number_of_cables, number_of_links;
};

cardsflow::kindyn::Robot *RobotConfigurationEstimator::robot;

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

    ros::AsyncSpinner spinner(0);
    spinner.start();

    MsjPlatform robot(urdf, cardsflow_xml);

    controller_manager::ControllerManager cm(&robot);

    // we need an additional update thread, otherwise the controllers won't switch
    thread update_thread(update, &cm);
    update_thread.detach();

    ROS_INFO("STARTING ROBOT MAIN LOOP...");

    ros::ServiceClient load_controller = nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    ros::ServiceClient switch_controller = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    controller_manager_msgs::LoadController msg;
    msg.request.name = "sphere_axis0";
    load_controller.call(msg);
    msg.request.name = "sphere_axis1";
    load_controller.call(msg);
    msg.request.name = "sphere_axis2";
    load_controller.call(msg);

    controller_manager_msgs::SwitchController msg2;
    msg2.request.start_controllers.push_back("sphere_axis0");
    msg2.request.start_controllers.push_back("sphere_axis1");
    msg2.request.start_controllers.push_back("sphere_axis2");
    msg2.request.strictness = msg2.request.BEST_EFFORT;
    switch_controller.call(msg2);

    RobotConfigurationEstimator estimator(100,8,7);
    estimator.robot = &robot;

    ros::Time t0 = ros::Time::now();
    while((ros::Time::now()-t0).toSec()<3){
        robot.update();
        robot.forwardKinematics(0.000001);
    }

    NumericalDiff<RobotConfigurationEstimator> *numDiff1;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<RobotConfigurationEstimator>, double> *lm;
    numDiff1 = new NumericalDiff<RobotConfigurationEstimator>(estimator);
    lm = new LevenbergMarquardt<NumericalDiff<RobotConfigurationEstimator>, double>(*numDiff1);
    lm->parameters.maxfev = 50000;
    lm->parameters.xtol = 1e-10;
    VectorXd x(24);
    x.setZero();
    for(int cable=0;cable<8;cable++){
        x[cable*3] = 0.001*rand()/(double)RAND_MAX;
        x[cable*3+1] = 0.001*rand()/(double)RAND_MAX;
        x[cable*3+2] = 0;
    }
    ROS_INFO_STREAM(x.transpose());
//    x = 0.001*x;
    int ret = lm->minimize(x);
    double error = lm->fnorm;
    robot.number_of_markers_to_publish_at_once = 1;
    for(int cable=0;cable<8;cable++){
        geometry_msgs::Pose msg;
        msg.position.x = x[cable*3];
        msg.position.y = x[cable*3+1];
        msg.position.z = x[cable*3+2];
        msg.orientation.w = 1;
        char str[100];
        sprintf(str,"cable_estimate_%d", cable);
        robot.publishTransform("world",str,msg);
        ros::spinOnce();
    }

    ROS_INFO_STREAM(ret << endl << error << endl << x.transpose());

    update_thread.join();
    return 0;
}