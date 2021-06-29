//
// Created by roboy on 07.06.21.
//

#include "kindyn/filter/joints_kalmanfilter.h"

namespace BFL {
    using namespace MatrixWrapper;

    BFL::KinDynEKF::KinDynEKF(int number_of_joints) :
            prior_(NULL),
            filter_(NULL),
            filter_initialized_(false),
            gps_initialized_(false),
            number_of_joints_(number_of_joints) {
        // create SYSTEM MODEL
        ColumnVector
        sysNoise_Mu(number_of_joints_ * 2);
        sysNoise_Mu = 0;
        SymmetricMatrix
        sysNoise_Cov(number_of_joints_ * 2);
        sysNoise_Cov = 0;
        for (unsigned int i = 1; i <= number_of_joints_ * 2; i++) sysNoise_Cov(i, i) = pow(10.0, 2);
        Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
        sys_pdf_ = new NonLinearAnalyticConditionalGaussianJointAngles(system_Uncertainty);
        sys_pdf_->initialize(number_of_joints_);
        sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

        // create MEASUREMENT MODEL GPS
        ColumnVector measNoiseGps_Mu(number_of_joints_);
        measNoiseGps_Mu = 0;
        SymmetricMatrix measNoiseGps_Cov(number_of_joints_);
        measNoiseGps_Cov = 0;
        for (unsigned int i = 1; i <= number_of_joints_; i++) measNoiseGps_Cov(i, i) = pow(15.0, 2);
        Gaussian measurement_Uncertainty_GPS(measNoiseGps_Mu, measNoiseGps_Cov);
        MatrixWrapper::Matrix
        Hgps(number_of_joints_, number_of_joints_ * 2);
        Hgps = 0;
        for (unsigned int i = 1; i <= number_of_joints_; i++) Hgps(i, i) = 1;
        gps_meas_pdf_ = new LinearAnalyticConditionalGaussian(Hgps, measurement_Uncertainty_GPS);
        gps_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(gps_meas_pdf_);

        MatrixWrapper::SymmetricMatrix covar(number_of_joints_);
        covar = 0;
        for (unsigned int i = 1; i <= number_of_joints_; i++) covar(i, i) = pow(5.0, 2);
        gps_covariance_ = covar;
    }

    // destructor
    BFL::KinDynEKF::~KinDynEKF() {
        if (filter_) delete filter_;
        if (prior_) delete prior_;
        delete gps_meas_model_;
        delete gps_meas_pdf_;
        delete sys_pdf_;
        delete sys_model_;
    };

    // initialize prior density of filter
    void BFL::KinDynEKF::initialize(const Eigen::VectorXd &prior, double time) {
        // set prior of filter
        ColumnVector
        prior_Mu(number_of_joints_ * 2);

        for (unsigned int i = 1; i <= number_of_joints_; i++) {
            prior_Mu(i) = prior(i - 1);
        }

        SymmetricMatrix
        prior_Cov(number_of_joints_ * 2);
        for (unsigned int i = 1; i <= number_of_joints_ * 2; i++) {
            for (unsigned int j = 1; j <= number_of_joints_ * 2; j++) {
                if (i == j) prior_Cov(i, j) = pow(0.001, 2);
                else prior_Cov(i, j) = 0;
            }
        }
        prior_ = new Gaussian(prior_Mu, prior_Cov);
        filter_ = new ExtendedKalmanFilter(prior_);

        // remember prior
        filter_estimate_old_vec_ = prior_Mu;

        // filter initialized
        filter_initialized_ = true;

        ROS_INFO("EKF is already initialized");
    }

    bool BFL::KinDynEKF::model_update(double dt, const Eigen::VectorXd &model_action) {
        // only update filter when it is initialized
        if (!filter_initialized_) {
            ROS_INFO("Cannot update filter when filter was not initialized first.");
            return false;
        }

//        // only update filter for time later than current filter time
//        double dt = (filter_time - filter_time_old_).toSec();
        if (dt == 0) return false;
        if (dt < 0) {
            ROS_INFO("Will not update robot pose with time %f sec in the past.", dt);
            return false;
        }
//        ROS_DEBUG("Update filter at time %f with dt %f", filter_time.toSec(), dt);


        // system update filter
        // --------------------
        // for now only add system noise
        ColumnVector vel_desi(number_of_joints_);
        for (unsigned int i = 1; i <= number_of_joints_; i++) {
            vel_desi(i) = model_action(i - 1);
        }
        filter_->Update(sys_model_, vel_desi);

        // remember last estimate
        filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
        return true;
    };

    bool BFL::KinDynEKF::sensor_update(const Eigen::VectorXd &meas_state) {
        // process gps measurement
        // ----------------------
        bool gps_active = true;
        if (gps_active){

            if (gps_initialized_){
//                gps_meas_pdf_->AdditiveNoiseSigmaSet(gps_covariance_ * pow(dt,2));
                ColumnVector gps_vec(number_of_joints_);
                for(unsigned int i=1; i<=number_of_joints_; i++){
                    gps_vec(i) = meas_state(i-1);
                }
                //Take gps as an absolute measurement, do not convert to relative measurement
                filter_->Update(gps_meas_model_,  gps_vec);
            }
            else {
                gps_initialized_ = true;
                gps_meas_old_ = gps_meas_;
            }
        }
            // sensor not active
        else gps_initialized_ = false;

        // remember last estimate
        // filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
        return true;
    };

    void BFL::KinDynEKF::getEstimate(Eigen::VectorXd& estimate, Eigen::VectorXd& estimate_vel)
    {
        for(unsigned int i=1; i<=number_of_joints_; i++) {
            estimate(i-1) = filter_estimate_old_vec_(i);
        }

        for(unsigned int i=1; i<=number_of_joints_; i++) {
            estimate_vel(i-1) = filter_estimate_old_vec_(i+number_of_joints_);
        }
    };
}