/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-05 20:01:04
 * @LastEditTime: 2021-08-06 20:57:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/include/core.hpp
 */

#ifndef DH_CORE_H
#define DH_CORE_H

#include <string>
#include <iostream>
#include <Eigen/Core>
#include <math.h>

namespace dh
{
    namespace core
    {
        /**
         * @brief Say hello
         * 
         */
        void hello()
        {
            std::cout << "Welcome using this navigation toolbox: DH!" << std::endl;
        }

        /**
         * @brief Quaternion(Hamilton) multiply 3x1 vector, using to convert the coordinate of a vector.
         * 
         * @param q Attitude quaternion, and scalar part at former.
         * @param vec 3x1 vector
         * @return Eigen::Matrix<double, 3, 1> The coordinate of input vector in another frame
         */
        inline Eigen::Matrix<double, 3, 1> quat_multiply_vec(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 3, 1> &vec)
        {

            Eigen::Matrix<double, 3, 1> vecout;

            vecout(0) = (q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3)) * vec(0) + (2 * q(1) * q(2) - 2 * q(0) * q(3)) * vec(1) + (2 * q(1) * q(3) + 2 * q(0) * q(2)) * vec(2);

            vecout(1) = (2 * q(1) * q(2) + 2 * q(0) * q(3)) * vec(0) + (q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3)) * vec(1) + (2 * q(2) * q(3) - 2 * q(0) * q(1)) * vec(2);

            vecout(2) = (2 * q(1) * q(3) - 2 * q(0) * q(2)) * vec(0) + (2 * q(2) * q(3) - 2 * q(0) * q(1)) * vec(2) + (q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3)) * vec(2);

            return vecout;
        }

        /**
         * @brief Quaternion(Hamilton) multiply quaternion
         * 
         * @param ql Left quaternion
         * @param qr Right quaternion
         * @return Eigen::Matrix<double, 4, 1> 
         */
        inline Eigen::Matrix<double, 4, 1> quat_multiply_quat(const Eigen::Matrix<double, 4, 1> &ql, const Eigen::Matrix<double, 4, 1> &qr)
        {
            Eigen::Matrix<double, 4, 1> qout;
            qout(0) = ql(0) * qr(0) - ql(1) * qr(1) - ql(2) * qr(2) - ql(3) * qr(3);
            qout(1) = ql(0) * qr(1) + ql(1) * qr(0) + ql(2) * qr(3) - ql(3) * qr(2);
            qout(2) = ql(0) * qr(2) + ql(2) * qr(0) + ql(3) * qr(1) - ql(1) * qr(3);
            qout(3) = ql(0) * qr(3) + ql(3) * qr(0) + ql(1) * qr(2) - ql(2) * qr(1);
            return qout;
        }

        /**
         * @brief Return the conjugate of a quaternion
         * 
         * @param q A quaternion
         * @return Eigen::Matrix<double, 4, 1> 
         */
        inline Eigen::Matrix<double, 4, 1> quat_conjugate(const Eigen::Matrix<double, 4, 1> &q)
        {
            Eigen::Matrix<double, 4, 1> qout;
            qout(0) = q(0);
            qout(1) = -q(1);
            qout(2) = -q(2);
            qout(3) = -q(3);
            return qout;
        }

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
         * @brief Return the rotation vector of the input anti-skew matrix.
         * 
         * @param mat An anti-skew matrix.
         * @return Eigen::Matrix<double, 3, 1> 
         */
        inline Eigen::Matrix<double, 3, 1> vee(const Eigen::Matrix<double, 3, 3> &mat){
            Eigen::Matrix<double, 3, 1> vec;
            vec << mat(2, 1), mat(0, 2), mat(1, 0);
            return vec;
        }

        /**
         * @brief Return the new quaternion(Halmition) after the input quaternion add a rotation.
         * 
         * @param q A quaternion
         * @param rot 3x1 rotation vector which indicate a rotaion.
         * @return Eigen::Matrix<double, 4, 1> 
         */
        inline Eigen::Matrix<double, 4, 1> quat_add_rot(const Eigen::Matrix<double, 4, 1> & q, const Eigen::Matrix<double, 3, 1> & rot){
            Eigen::Matrix<double, 4, 1> dq;
            const double norm_rot = rot.norm();
            const double sin_half_norm_rot = std::sin(norm_rot/2);
            dq(0) = std::cos(norm_rot/2);
            dq(1) = rot(0)/norm_rot*sin_half_norm_rot;
            dq(2) = rot(1)/norm_rot*sin_half_norm_rot;
            dq(3) = rot(2)/norm_rot*sin_half_norm_rot;
            return dh::core::quat_multiply_quat(q, dq);
        }

        
        
    }
}

#endif