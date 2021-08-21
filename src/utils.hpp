/*
 * @Author: your name
 * @Date: 2021-08-21 11:07:33
 * @LastEditTime: 2021-08-21 11:12:54
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/utils.hpp
 */

#ifndef DH_UTILS_H
#define DH_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dh
{
    namespace utils
    {
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
        inline Eigen::Matrix<double, 3, 1> cross_product(const Eigen::Matrix<double, 3, 1> v, const Eigen::Matrix<double, 3, 1> r)
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

    } // namespace utils

} // namespace dh

#endif