#include "kindyn/EigenExtension.hpp"

namespace EigenExtension {
    // Function to convert cross product into a matrix multiplication.
    Matrix3f SkewSymmetric(Vector3f v) {
        Matrix3f V;
        V << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
        return V;
    }

    Matrix3d SkewSymmetric2(Vector3d v) {
        Matrix3d V;
        V <<    0,   -v(2),  v(1),
                v(2),    0, -v(0),
                -v(1), v(0), 0;
        return V;
    }

    // SHOULD THE SIZE BE ENFORCED

    // // Function to obtain the coefficients of a spline.
    Matrix<float, 2, 1> GetLinearSplineCoefficients(float q_r_i, float q_r_ip1, double t) {
        Matrix<float, 2, 2> T;
        T << 1, 0,
                1, t;
        Matrix<float, 2, 1> b_x;
        b_x << q_r_i,
                q_r_ip1;
        return T.inverse() * b_x;
    }

    // Function to obtain the coefficients of a spline.
    Matrix<float, 4, 1>
    GetCubicSplineCoefficients(float q_r_i, float q_d_r_i, float q_r_ip1, float q_d_r_ip1, double t) {
        Matrix<float, 4, 4> T;
        T << 1, 0, 0, 0,
                0, 1, 0, 0,
                1, t, pow(t, 2), pow(t, 3),
                0, 1, 2 * t, 3 * pow(t, 2);
        Matrix<float, 4, 1> b_x;
        b_x << q_r_i,
                q_d_r_i,
                q_r_ip1,
                q_d_r_ip1;
        return T.inverse() * b_x;
    }

    // Function to obtain the coefficients of a spline.
    Matrix<float, 6, 1> GetQuinticSplineCoefficients(float q_r_i, float q_d_r_i, float q_dd_r_i,
                                                     float q_r_ip1, float q_d_r_ip1, float q_dd_r_ip1, double t) {
        Matrix<float, 6, 6> T;
        T << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 2, 0, 0, 0,
                1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
                0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
                0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);
        Matrix<float, 6, 1> b_x;
        b_x << q_r_i,
                q_d_r_i,
                q_dd_r_i,
                q_r_ip1,
                q_d_r_ip1,
                q_dd_r_ip1;
        return T.inverse() * b_x;
    }

    // Function to interpolate a point on a spline
    void LinearSplineInterpolate(float *q_r, float *q_d_r, float *q_dd_r, Matrix<float, 2, 1> a, double t) {
        Matrix<float, 2, 1> t1, t2, t3;
        t1 << 1,
                t;
        t2 << 0,
                1;
        t3 << 0,
                0;
        *q_r = a.dot(t1);
        *q_d_r = a.dot(t2);
        *q_dd_r = a.dot(t3);
    }

    // // Function to interpolate a point on a spline
    void CubicSplineInterpolate(float *q_r, float *q_d_r, float *q_dd_r, Matrix<float, 4, 1> a, double t) {
        Matrix<float, 4, 1> t1, t2, t3;
        t1 << 1,
                t,
                pow(t, 2),
                pow(t, 3);
        t2 << 0,
                1,
                2 * t,
                3 * pow(t, 2);
        t3 << 0,
                0,
                2,
                6 * t;
        *q_r = a.dot(t1);
        *q_d_r = a.dot(t2);
        *q_dd_r = a.dot(t3);
    }

    // Function to interpolate a point on a spline
    void QuinticSplineInterpolate(float *q_r, float *q_d_r, float *q_dd_r, Matrix<float, 6, 1> a, double t) {
        Matrix<float, 6, 1> t1, t2, t3;
        t1 << 1,
                t,
                pow(t, 2),
                pow(t, 3),
                pow(t, 4),
                pow(t, 5);
        t2 << 0,
                1,
                2 * t,
                3 * pow(t, 2),
                4 * pow(t, 3),
                5 * pow(t, 4);
        t3 << 0,
                0,
                2,
                6 * t,
                12 * pow(t, 2),
                20 * pow(t, 3);
        *q_r = a.dot(t1);
        *q_d_r = a.dot(t2);
        *q_dd_r = a.dot(t3);
    }

// Compute rotation matrix
    Matrix3f ComputeRotationMatrix(AngleAxisf a) {
        Quaternionf q = Quaternionf(a);
        double q_0, q_1, q_2, q_3;
        q_0 = q.w();
        q_1 = q.x();
        q_2 = q.y();
        q_3 = q.z();
        Matrix3f R_0p;
        R_0p << 1 - 2 * (pow(q_2, 2) + pow(q_3, 2)), 2 * q_1 * q_2 - 2 * q_0 * q_3, 2 * q_0 * q_2 + 2 * q_1 * q_3,
                2 * q_1 * q_2 + 2 * q_0 * q_3, 1 - 2 * (pow(q_1, 2) + pow(q_3, 2)), -2 * q_0 * q_1 + 2 * q_2 * q_3,
                -2 * q_0 * q_2 + 2 * q_1 * q_3, 2 * q_0 * q_1 + 2 * q_2 * q_3, 1 - 2 * (pow(q_1, 2) + pow(q_2, 2));
        return R_0p;
    }

// Compute a rotation matrix for the derivative
    Matrix3f ComputeRotationMatrixDeriv(AngleAxisf a, AngleAxisf a_d) {
        // Convert axis angle representation to quaternions
        Quaternionf q = Quaternionf(a);
        Quaternionf q_d = Quaternionf((-a_d.angle() / 2) * sin(a.angle() / 2),
                                      (a.axis())(0) * (a_d.angle() / 2) * cos(a.angle() / 2),
                                      (a.axis())(1) * (a_d.angle() / 2) * cos(a.angle() / 2),
                                      (a.axis())(2) * (a_d.angle() / 2) * cos(a.angle() / 2));
        // Convert quaternion to rotation matrix
        double q_0, q_1, q_2, q_3, q0_d, q1_d, q2_d, q3_d;
        q_0 = q.w();
        q_1 = q.x();
        q_2 = q.y();
        q_3 = q.z();
        q0_d = q_d.w();
        q1_d = q_d.x();
        q2_d = q_d.y();
        q3_d = q_d.z();
        Matrix3f R_0p_d;
        R_0p_d << -4 * q2_d * q_2 - 4 * q3_d * q_3, 2 * q1_d * q_2 + 2 * q_1 * q2_d - 2 * q0_d * q_3 - 2 * q_0 * q3_d,
                2 * q0_d * q_2 + 2 * q_0 * q2_d + 2 * q1_d * q_3 + 2 * q_1 * q3_d,
                2 * q1_d * q_2 + 2 * q_1 * q2_d + 2 * q0_d * q_3 + 2 * q_0 * q3_d, -4 * q1_d * q_1 - 4 * q3_d * q_3,
                -2 * q0_d * q_1 - 2 * q_0 * q1_d + 2 * q2_d * q_3 + 2 * q_2 * q3_d,
                -2 * q0_d * q_2 - 2 * q_0 * q2_d + 2 * q1_d * q_3 + 2 * q_1 * q3_d, 2 * q0_d * q_1 + 2 * q_0 * q1_d +
                                                                                    2 * q2_d * q_3 + 2 * q_2 * q3_d,
                -4 * q1_d * q_1 - 4 * q2_d * q_2;
        return R_0p_d;
    }

// Compute a rotation matrix for the double derivative
    Matrix3f ComputeRotationMatrixDoubleDeriv(AngleAxisf a, AngleAxisf a_d, AngleAxisf a_dd) {
        // Convert axis angle representation to quaternions
        Quaternionf q = Quaternionf(a);
        Quaternionf q_d = Quaternionf(-a_d.angle() / 2 * sin(a.angle() / 2),
                                      (a.axis())(0) * (a_d.angle() / 2) * cos(a.angle() / 2),
                                      (a.axis())(1) * (a_d.angle() / 2) * cos(a.angle() / 2),
                                      (a.axis())(2) * (a_d.angle() / 2) * cos(a.angle() / 2));
        Quaternionf q_dd = Quaternionf(
                -a_dd.angle() / 2 * sin(a.angle() / 2) - pow(a_d.angle(), 2) / 4 * cos(a.angle() / 2),
                (a.axis())(0) * (a_dd.angle() / 2 * cos(a.angle() / 2) - pow(a_d.angle(), 2) / 4 * sin(a.angle() / 2)),
                (a.axis())(1) * (a_dd.angle() / 2 * cos(a.angle() / 2) - pow(a_d.angle(), 2) / 4 * sin(a.angle() / 2)),
                (a.axis())(2) * (a_dd.angle() / 2 * cos(a.angle() / 2) - pow(a_d.angle(), 2) / 4 * sin(a.angle() / 2)));
        double q_0, q_1, q_2, q_3, q0_d, q1_d, q2_d, q3_d, q0_dd, q1_dd, q2_dd, q3_dd;
        q_0 = q.w();
        q_1 = q.x();
        q_2 = q.y();
        q_3 = q.z();
        q0_d = q_d.w();
        q1_d = q_d.x();
        q2_d = q_d.y();
        q3_d = q_d.z();
        q0_dd = q_dd.w();
        q1_dd = q_dd.x();
        q2_dd = q_dd.y();
        q3_dd = q_dd.z();
        Matrix3f R_0p_dd;
        R_0p_dd << -4 * q2_dd * q_2 - 4 * pow(q2_d, 2) - 4 * q3_dd * q_3 - 4 * pow(q3_d, 2), 2 * q1_dd * q_2 +
                                                                                             4 * q1_d * q2_d +
                                                                                             2 * q_1 * q2_dd -
                                                                                             2 * q0_dd * q_3 -
                                                                                             4 * q0_d * q3_d -
                                                                                             2 * q_0 * q3_dd,
                2 * q0_dd * q_2 + 4 * q0_d * q2_d + 2 * q_0 * q2_dd + 2 * q1_dd * q_3 + 4 * q1_d * q3_d +
                2 * q_1 * q3_dd,
                2 * q1_dd * q2_d + 4 * q1_d * q2_d + 2 * q_1 * q2_dd + 2 * q0_dd * q_3 + 4 * q0_d * q3_d +
                2 * q_0 * q3_dd, -4 * q1_dd * q_1 - 4 * pow(q1_d, 2) - 4 * q3_dd * q_3 - 4 * pow(q3_d, 2),
                -2 * q0_dd * q_1 - 4 * q0_d * q1_d - 2 * q_0 * q1_dd + 2 * q2_dd * q_3 + 4 * q2_d * q3_d +
                2 * q_2 * q3_dd,
                -2 * q0_dd * q_2 - 4 * q0_d * q2_d - 2 * q_0 * q2_dd + 2 * q1_dd * q_3 + 4 * q1_d * q3_d +
                2 * q_1 * q3_dd, 2 * q0_dd * q_1 + 4 * q0_d * q1_d + 2 * q_0 * q1_dd + 2 * q2_dd * q_3 +
                                 4 * q2_d * q3_d + 2 * q_2 * q3_dd, -4 * q1_dd * q_1 - 4 * pow(q1_d, 2) -
                                                                    4 * q2_dd * q_2 - 4 * pow(q2_d, 2);
        return R_0p_dd;
    }

    MatrixXf Pinv(MatrixXf A) {

        int n_rows = A.rows();
        int n_cols = A.cols();
        //MatrixXf A_pinv = MatrixXf::Zero(n_cols,n_rows);
        JacobiSVD<MatrixXf> svd(A, ComputeFullU | ComputeFullV);
        if (n_rows < n_cols) {
            VectorXf S = svd.singularValues();
            MatrixXf S_pinv = MatrixXf::Zero(n_cols, n_rows);
            for (int i = 0; i < n_rows; i++) {
                if (S(i) > 1e-6) {
                    S_pinv(i, i) = 1 / S(i);
                }
            }
            return svd.matrixV() * S_pinv * svd.matrixU().transpose();
        } else {
            VectorXf S = svd.singularValues();
            MatrixXf S_pinv = MatrixXf::Zero(n_cols, n_rows);
            for (int i = 0; i < n_cols; i++) {
                if (S(i) > 1e-6) {
                    S_pinv(i, i) = 1 / S(i);
                }
            }
            return svd.matrixV() * S_pinv * svd.matrixU().transpose();
        }
    }

    MatrixXd Pinv(MatrixXd A) {

        int n_rows = A.rows();
        int n_cols = A.cols();
        //MatrixXf A_pinv = MatrixXf::Zero(n_cols,n_rows);
        JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
        if (n_rows < n_cols) {
            VectorXd S = svd.singularValues();
            MatrixXd S_pinv = MatrixXd::Zero(n_cols, n_rows);
            for (int i = 0; i < n_rows; i++) {
                if (S(i) > 1e-6) {
                    S_pinv(i, i) = 1 / S(i);
                }
            }
            return svd.matrixV() * S_pinv * svd.matrixU().transpose();
        } else {
            VectorXd S = svd.singularValues();
            MatrixXd S_pinv = MatrixXd::Zero(n_cols, n_rows);
            for (int i = 0; i < n_cols; i++) {
                if (S(i) > 1e-6) {
                    S_pinv(i, i) = 1 / S(i);
                }
            }
            return svd.matrixV() * S_pinv * svd.matrixU().transpose();
        }
    }
}
