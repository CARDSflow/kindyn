//
// Created by roboy on 07.06.21.
//

#ifndef ROBOY3_JOINTS_KALMANFILTER_H
#define ROBOY3_JOINTS_KALMANFILTER_H

#include <ros/ros.h>
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include "kindyn/filter/nonlinearanalyticconditionalgaussianjointangles.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace BFL {
    class KinDynEKF {
    public:
        /// constructor
        KinDynEKF(int number_of_joints);

        /// destructor
        virtual ~KinDynEKF();

        /** update the extended Kalman filter
        * \param odom_active specifies if the odometry sensor is active or not
        * \param imu_active specifies if the imu sensor is active or not
        * \param gps_active specifies if the gps sensor is active or not
        * \param vo_active specifies if the vo sensor is active or not
        * \param filter_time update the ekf up to this time
        * \param diagnostics_res returns false if the diagnostics found that the sensor measurements are inconsistent
        * returns true on successfull update
        */
        bool model_update(double dt, const Eigen::VectorXd &model_action);

        bool sensor_update(const Eigen::VectorXd &meas_state);

        /** initialize the extended Kalman filter
         * \param prior the prior robot pose
         * \param time the initial time of the ekf
         */
        void initialize(const Eigen::VectorXd &prior, double time);

        /** check if the filter is initialized
         * returns true if the ekf has been initialized already
         */
        bool isInitialized() { return filter_initialized_; };

        /** Add a sensor measurement to the measurement buffer
        * \param meas the measurement to add
        */
        void addMeasurement(const Eigen::VectorXd &meas);

        void getEstimate(Eigen::VectorXd& estimate, Eigen::VectorXd& estimate_vel);

    private:
        // pdf / model / filter
        BFL::AnalyticSystemModelGaussianUncertainty *sys_model_;
        BFL::NonLinearAnalyticConditionalGaussianJointAngles *sys_pdf_;
        BFL::LinearAnalyticConditionalGaussian *gps_meas_pdf_;
        BFL::LinearAnalyticMeasurementModelGaussianUncertainty *gps_meas_model_;
        BFL::Gaussian *prior_;
        BFL::ExtendedKalmanFilter *filter_;
        MatrixWrapper::SymmetricMatrix gps_covariance_;
        // vars
        MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
        Eigen::VectorXd filter_estimate_old_;
        Eigen::VectorXd gps_meas_, gps_meas_old_;
        int number_of_joints_;
        double filter_time_old_;
        bool filter_initialized_, odom_initialized_, imu_initialized_, vo_initialized_, gps_initialized_;

        /// correct for angle overflow
        void angleOverflowCorrect(double &a, double ref);
    };
}

#endif //ROBOY3_JOINTS_KALMANFILTER_H
