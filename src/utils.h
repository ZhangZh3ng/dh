/*
 * @Author: your name
 * @Date: 2021-08-21 11:07:33
 * @LastEditTime: 2021-09-15 15:54:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/utils.hpp
 */

#ifndef DH_UTILS_H
#define DH_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "constant.h"

namespace dh{
  /**
  * @brief Return the anti-skew matrix corresponding to the input 3x1 vector
  * 
  * @param vec A 3x1 vector
  * @return Eigen::Matrix<double, 3, 3> 
  */
  inline Eigen::Matrix<double, 3, 3> askew(const Eigen::Matrix<double, 3, 1> &vec)
  {
      Eigen::Matrix<double, 3, 3> mat;
      mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(2), vec(0), 0;
      return mat;
  }

  /**
  * @brief Return 3d vector cross product i.e. vxr.
  * 
  * @param v Vector in the left.
  * @param r Vector in the rigth.
  * @return Eigen::Matrix<double, 3, 1> 
  */
  inline Eigen::Matrix<double, 3, 1> cross_product(const Eigen::Matrix<double, 3, 1> v,
                                                   const Eigen::Matrix<double, 3, 1> r)
  {
      Eigen::Matrix<double, 3, 1> vo;
      vo(0) = v(1) * r(2) - v(2) * r(1);
      vo(1) = v(2) * r(0) - v(0) * r(2);
      vo(2) = v(0) * r(1) - v(1) * r(0);
      return vo;
  }

  /** 
  * @brief Return the dot product of two vector, i.e. v1*v2 
  * 
  * @param v1 
  * @param v2 
  * @return double 
  */
  inline double dot_product(const Eigen::Matrix<double, Eigen::Dynamic, 1> v1, const Eigen::Matrix<double, Eigen::Dynamic, 1> v2)
  {
      int row1, row2;
      row1 = v1.rows();
      row2 = v2.rows();
      if (row1 != row2)
      {
          throw 1;
      }

      double result = 0;
      for (int i = 0; i < row1; i++)
      {
          result = result + v1(i) * v2(i);
      }
      return result;
  }

  template<int N>
  inline Eigen::Matrix<double, N, N> std_to_covariance(const Eigen::Matrix<double, N, 1>& std){
    Eigen::Matrix<double, N, N> covariance;   
    covariance.setZero();
    for(int i = 0; i<N; ++i)
        covariance(i, i) = std(i) * std(i);
    return covariance;
  }

  /**
   * @brief return TRUE when abs(a-b) is enouth small.
   */
  inline bool AlmostEqual(const double &a, const double &b){
    // return (abs(a - b) < IF_EQUAL_THRESHHOLD);
    return (abs(a - b) < 1e-8);
  }
  
  /**
   * @brief return TRUE when (a-b) is enouth large positive number.
   */
  inline bool DefinitelyGreater(const double& a, const double &b){
    return (a > b) && !(AlmostEqual(a, b));
  }

  /**
   * @brief return TRUE when a>b or (b-a) is enough small
   */
  inline bool GreaterOrAlmostEqual(const double& a, const double& b){
    return (a > b) || AlmostEqual(a, b);
  }

} // namespace dh
#endif // DH_UTILS_H