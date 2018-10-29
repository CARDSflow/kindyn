/* A namespace that extends Eigen to contain other necessary functions 
 *
 * Author        : Jonathan EDEN
 * Created       : 2016
 * Description   : A namepsace that extends the Eigen library
*/

#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

using namespace Eigen;
// This is a namespace to add some necessary functions into.
namespace EigenExtension{
  /**
   * Skew symmetric matrix for cross product computation
   * @param v 3d Vector
   * @return skew symmetric matrix
   */
  Matrix3f SkewSymmetric(Vector3f v);
  /**
   * Skew symmetric matrix for cross product computation
   * @param v 3d Vector
   * @return skew symmetric matrix
   */
  Matrix3d SkewSymmetric2(Vector3d v);
  // Get the spline coefficients.
  Matrix<float,2,1> GetLinearSplineCoefficients(float q_r_i, float q_r_ip1, double t);
  Matrix<float,4,1> GetCubicSplineCoefficients(float q_r_i, float q_d_r_i,
                                          float q_r_ip1, float q_d_r_ip1, double t);
  Matrix<float,6,1> GetQuinticSplineCoefficients(float q_r_i, float q_d_r_i, float q_dd_r_i,
                                          float q_r_ip1, float q_d_r_ip1, float q_dd_r_ip1, double t);
  // Interpolate the spline
  void LinearSplineInterpolate(float* q_r, float* q_d_r, float* q_dd_r, Matrix<float,2,1> a, double t);
  void CubicSplineInterpolate(float* q_r, float* q_d_r, float* q_dd_r, Matrix<float,4,1> a, double t);                                        
  void QuinticSplineInterpolate(float* q_r, float* q_d_r, float* q_dd_r, Matrix<float,6,1> a, double t);
  // Compute the rotation matrix
  Matrix3f ComputeRotationMatrix(AngleAxisf a);
  // Compute the derivative of a rotation matrix
  Matrix3f ComputeRotationMatrixDeriv(AngleAxisf a, AngleAxisf a_d);
  // Compute the double derivative of a rotation matrix
  Matrix3f ComputeRotationMatrixDoubleDeriv(AngleAxisf a, AngleAxisf a_d, AngleAxisf a_dd);
  /**
   *  Pseudo inverse matrix
   * @param A
   * @return Pseudo inverse matrix
   */
  MatrixXf Pinv(MatrixXf A);
  /**
   *  Pseudo inverse matrix
   * @param A
   * @return Pseudo inverse matrix
   */
  MatrixXd Pinv(MatrixXd A);

  //MatrixXf LeftPseudoInverse(MatrixXf A);
  //MatrixXf RightMatrixPseudoInverse(MatrixXf A);
}