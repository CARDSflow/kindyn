//
// Created by roboy on 07.06.21.
//

#include <kindyn/filter/nonlinearanalyticconditionalgaussianjointangles.h>
#include <bfl/wrappers/rng/rng.h> // Wrapper around several rng libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
    using namespace MatrixWrapper;


    NonLinearAnalyticConditionalGaussianJointAngles::NonLinearAnalyticConditionalGaussianJointAngles(const Gaussian& additiveNoise)
            : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE)
    {

    }

    void NonLinearAnalyticConditionalGaussianJointAngles::initialize(int number_of_joints){
        number_of_joints_ = number_of_joints;
        df.resize(number_of_joints_*2,number_of_joints_*2);

        // initialize df matrix
        for (unsigned int i=1; i<=number_of_joints_*2; i++){
            for (unsigned int j=1; j<=number_of_joints_*2; j++){
                if (i==j) df(i,j) = 1;
                else df(i,j) = 0;
            }
        }
    }


    NonLinearAnalyticConditionalGaussianJointAngles::~NonLinearAnalyticConditionalGaussianJointAngles(){}

    ColumnVector NonLinearAnalyticConditionalGaussianJointAngles::ExpectedValueGet() const
    {
        ColumnVector state = ConditionalArgumentGet(0);
        ColumnVector vel  = ConditionalArgumentGet(1);

//        state(1) += cos(state(6)) * vel(1);
//        state(2) += sin(state(6)) * vel(1);
//        state(6) += vel(2);

//        state(1) += vel(1);
//        state(2) += vel(2);
//        state(3) += vel(3);

        for (unsigned int i=1; i<=number_of_joints_; i++){
            state(i) += 0.005*vel(i);
        }

        return state + AdditiveNoiseMuGet();
    }

    Matrix NonLinearAnalyticConditionalGaussianJointAngles::dfGet(unsigned int i) const
    {
        if (i==0)//derivative to the first conditional argument (x)
        {
            ColumnVector vel = ConditionalArgumentGet(1);
//            double yaw = ConditionalArgumentGet(0)(6);

//            df(1,3)=-vel_trans*sin(yaw);
//            df(2,3)= vel_trans*cos(yaw);

//            df(1,1)= vel(1);
//            df(2,2)= vel(2);
//            df(3,3)= vel(3);

            for (unsigned int i=1; i<=number_of_joints_; i++){
                df(i, i) = vel(i);
            }

            return df;
        }
        else
        {
            if (i >= NumConditionalArgumentsGet())
            {
                cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
                exit(-BFL_ERRMISUSE);
            }
            else{
                cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
                exit(-BFL_ERRMISUSE);
            }
        }
    }

}//namespace BFL