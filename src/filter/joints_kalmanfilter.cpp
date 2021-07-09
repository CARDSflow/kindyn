//
// Created by roboy on 07.06.21.
//

#include "kindyn/filter/joints_kalmanfilter.h"

namespace BFL {
    using namespace MatrixWrapper;

    BFL::KinDynEKF::KinDynEKF(int number_of_states) :
            prior_(NULL),
            filter_(NULL),
            filter_initialized_(false),
            magnetic_initialized_(false),
            number_of_states_(number_of_states) {

        // create SYSTEM MODEL
        ColumnVector
        sysNoise_Mu(number_of_states_ * 2);
        sysNoise_Mu = 0;
        SymmetricMatrix
        sysNoise_Cov(number_of_states_ * 2);
        sysNoise_Cov = 0;
        for (unsigned int i = 1; i <= number_of_states_ * 2; i++) sysNoise_Cov(i, i) = pow(30.0, 2);
        Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
        sys_pdf_ = new NonLinearAnalyticConditionalGaussianJointAngles(system_Uncertainty);
        sys_pdf_->initialize(number_of_states_);
        sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

        // create MEASUREMENT MODEL GPS
        ColumnVector measNoiseGps_Mu(number_of_states_);
        measNoiseGps_Mu = 0;
        SymmetricMatrix measNoiseGps_Cov(number_of_states_);
        measNoiseGps_Cov = 0;
        for (unsigned int i = 1; i <= number_of_states_; i++) measNoiseGps_Cov(i, i) = pow(1.0, 2);
        Gaussian measurement_Uncertainty_GPS(measNoiseGps_Mu, measNoiseGps_Cov);
        MatrixWrapper::Matrix Hmagnetic(number_of_states_, number_of_states_ * 2);
        Hmagnetic = 0;
        for (unsigned int i = 1; i <= number_of_states_; i++) Hmagnetic(i, i) = 1;
        magnetic_meas_pdf_ = new LinearAnalyticConditionalGaussian(Hmagnetic, measurement_Uncertainty_GPS);
        magnetic_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(magnetic_meas_pdf_);
    };

    // destructor
    BFL::KinDynEKF::~KinDynEKF() {
        if (filter_) delete filter_;
        if (prior_) delete prior_;
        delete magnetic_meas_model_;
        delete magnetic_meas_pdf_;
        delete sys_pdf_;
        delete sys_model_;
    };

    // initialize prior density of filter
    void BFL::KinDynEKF::initialize(const Eigen::VectorXd &prior, double time) {
        // set prior of filter
        ColumnVector
        prior_Mu(number_of_states_ * 2);

        for (unsigned int i = 1; i <= number_of_states_; i++) {
            prior_Mu(i) = prior(i - 1);
        }

        SymmetricMatrix
        prior_Cov(number_of_states_ * 2);
        for (unsigned int i = 1; i <= number_of_states_ * 2; i++) {
            for (unsigned int j = 1; j <= number_of_states_ * 2; j++) {
                if (i == j) prior_Cov(i, j) = pow(1.0, 2);
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
            ROS_WARN_STREAM_THROTTLE(5, "Cannot update filter when filter was not initialized first.");
            return false;
        }

        if (dt == 0) return false;
        if (dt < 0) {
            ROS_INFO("Will not update robot pose with time %f sec in the past.", dt);
            return false;
        }

        // system update filter
        // --------------------
        // for now only add system noise
        ColumnVector vel_desi(number_of_states_);
        for (unsigned int i = 1; i <= number_of_states_; i++) {
            vel_desi(i) = dt*model_action(i - 1);
        }
        filter_->Update(sys_model_, vel_desi);

        // remember last estimate
        filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
        return true;
    };

    bool BFL::KinDynEKF::sensor_update(const Eigen::VectorXd &meas_state) {
        // process magnetic measurement
        // ----------------------

        if (magnetic_initialized_){
            ColumnVector magnetic_vec(number_of_states_);
            for(unsigned int i=1; i<=number_of_states_; i++){
                if(abs(magnetic_meas_old_(i-1) - meas_state(i-1)) < 0.25){
                    magnetic_vec(i) = meas_state(i-1);
                }else{
                    magnetic_vec(i) = magnetic_meas_old_(i-1);
                    ROS_WARN_STREAM("Reject measurement for joint " << i-1 << " with value " << meas_state(i-1));
                }
            }

            //Take magnetic as an absolute measurement, do not convert to relative measurement
            filter_->Update(magnetic_meas_model_,  magnetic_vec);
        }
        else {
            magnetic_initialized_ = true;
        }

        // remember last estimate
        magnetic_meas_old_ = meas_state;
        return true;
    };

    void BFL::KinDynEKF::getEstimate(Eigen::VectorXd& estimate, Eigen::VectorXd& estimate_vel)
    {
        for(unsigned int i=1; i<=number_of_states_; i++) {
            estimate(i-1) = filter_estimate_old_vec_(i);
        }

        for(unsigned int i=1; i<=number_of_states_; i++) {
            estimate_vel(i-1) = filter_estimate_old_vec_(i+number_of_states_);
        }
    };
}