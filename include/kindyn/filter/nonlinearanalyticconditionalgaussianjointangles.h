//
// Created by roboy on 07.06.21.
//

#ifndef ROBOY3_NONLINEARANALYTICCONDITIONALGAUSSIANJOINTANGLES_H
#define ROBOY3_NONLINEARANALYTICCONDITIONALGAUSSIANJOINTANGLES_H

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{
    /// Non Linear Conditional Gaussian
    /**
       - \f$ \mu = Matrix[1] . ConditionalArguments[0] +
       Matrix[2]. ConditionalArguments[1]  + ... + Noise.\mu \f$
       - Covariance is independent of the ConditionalArguments, and is
       the covariance of the Noise pdf
    */
    class NonLinearAnalyticConditionalGaussianJointAngles : public AnalyticConditionalGaussianAdditiveNoise
    {
    public:
        /// Constructor
        /** @pre:  Every Matrix should have the same amount of rows!
        This is currently not checked.  The same goes for the number
        of columns, which should be equal to the number of rows of
        the corresponding conditional argument!
        @param additiveNoise Pdf representing the additive Gaussian uncertainty
        */
        NonLinearAnalyticConditionalGaussianJointAngles( const Gaussian& additiveNoise);

        /// Destructor
        virtual ~NonLinearAnalyticConditionalGaussianJointAngles();

        // redefine virtual functions
        virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
        virtual MatrixWrapper::Matrix          dfGet(unsigned int i)       const;

        void initialize(int number_of_joints);

    private:
        mutable MatrixWrapper::Matrix df;
        int number_of_joints_;
    };

} // End namespace BFL

#endif //ROBOY3_NONLINEARANALYTICCONDITIONALGAUSSIANJOINTANGLES_H
