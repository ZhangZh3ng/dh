/*
 * @Author: your name
 * @Date: 2021-08-21 08:05:59
 * @LastEditTime: 2021-08-21 11:30:51
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/geometry.hpp
 */

#ifndef DH_GEOMETRY_H
#define DH_GEOMETRY_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "type.hpp"
#include <cmath>

namespace dh
{
    /**
     * @brief This modual extend eigen's geometry modual, add some interface.
     * 
     */
    namespace geometry
    {
        /**
         * @brief Convert three euler angle to direction cosine matrix(dcm).
         * 
         * @param r1 1st angle, unit is radian.
         * @param r2 2nd angle
         * @param r3 3rd angle
         * @param axis1 1st axis, type is dh::type::RotationAxis.
         * @param axis2 2nd axis
         * @param axis3 3rd axis
         * @return Eigen::Matrix<double, 3, 3> 
         */
        inline Eigen::Matrix<double, 3, 3> euler_angle_to_dcm(const double r1, const double r2, const double r3, dh::type::RotationAxis axis1, dh::type::RotationAxis axis2, dh::type::RotationAxis axis3)
        {
            if (axis1 == axis2 || axis2 == axis3)
            {
                throw 1;
            }

            Eigen::Matrix<double, 3, 3> m1, m2, m3, dcm;

            switch (axis1)
            {
            case dh::type::RotationAxis::X:
                m1 = Eigen::AngleAxisd(r1, Eigen::Vector3d::UnitX());
                break;
            case dh::type::RotationAxis::Y:
                m1 = Eigen::AngleAxisd(r1, Eigen::Vector3d::UnitY());
                break;
            case dh::type::RotationAxis::Z:
                m1 = Eigen::AngleAxisd(r1, Eigen::Vector3d::UnitZ());
                break;
            }

            switch (axis2)
            {
            case dh::type::RotationAxis::X:
                m2 = Eigen::AngleAxisd(r2, Eigen::Vector3d::UnitX());
                break;
            case dh::type::RotationAxis::Y:
                m2 = Eigen::AngleAxisd(r2, Eigen::Vector3d::UnitY());
                break;
            case dh::type::RotationAxis::Z:
                m2 = Eigen::AngleAxisd(r2, Eigen::Vector3d::UnitZ());
                break;
            }

            switch (axis3)
            {
            case dh::type::RotationAxis::X:
                m3 = Eigen::AngleAxisd(r3, Eigen::Vector3d::UnitX());
                break;
            case dh::type::RotationAxis::Y:
                m3 = Eigen::AngleAxisd(r3, Eigen::Vector3d::UnitY());
                break;
            case dh::type::RotationAxis::Z:
                m3 = Eigen::AngleAxisd(r3, Eigen::Vector3d::UnitZ());
                break;
            }

            dcm = m1 * m2 * m3;
            return dcm;
        }

        /**
         * @brief convert direction cosine matrix(dcm) to quaternion
         * 
         * @param dcm 
         * @return Eigen::Quaterniond 
         */
        inline Eigen::Quaterniond dcm_to_quat(const Eigen::Matrix<double, 3, 3> dcm)
        {
            Eigen::Quaterniond q = Eigen::Quaterniond(dcm);
            return q;
        }

        /***************************************************************************
         *                 zyx eluer angle -> quaternion, dcm                       *
         ***************************************************************************/

        /**
         * @brief convert zxy euler angle to direction cosine matrix(dcm)
         * 
         * @param rxyz 3x1 vector, in order of rx, ry, rz. angle unit is radian.
         * @return Eigen::Matrix<double, 3, 3> 
         */
        Eigen::Matrix<double, 3, 3> zxy_euler_angle_to_dcm(const Eigen::Matrix<double, 3, 1> rxyz)
        {
            Eigen::Matrix<double, 3, 3> dcm;
            dcm = dh::geometry::euler_angle_to_dcm(rxyz(2), rxyz(0), rxyz(1), dh::type::RotationAxis::Z, dh::type::RotationAxis::X, dh::type::RotationAxis::Y);
            return dcm;
        }

        /**
         * @brief convert zxy euler angle to direction cosine matrix(dcm)
         * 
         * @param rx rotation around x axis, angle unit is radian.
         * @param ry rotation around y axis, angle unit is radian.
         * @param rz rotation around z axis, angle unit is radian.
         * @return Eigen::Matrix<double, 3, 3> 
         */
        inline Eigen::Matrix<double, 3, 3> zxy_euler_angle_to_dcm(const double rx, const double ry, const double rz)
        {
            Eigen::Matrix<double, 3, 3> dcm;
            dcm = dh::geometry::euler_angle_to_dcm(rz, rx, ry, dh::type::RotationAxis::Z, dh::type::RotationAxis::X, dh::type::RotationAxis::Y);
            return dcm;
        }

        /**
         * @brief convert zxy euler angle to quaternion.
         * 
         * @param rxyz 3x1 vector, in order of rx, ry, rz. angle unit is radian.
         * @return Eigen::Quaterniond 
         */
        inline Eigen::Quaterniond zxy_euler_angle_to_quat(const Eigen::Matrix<double, 3, 1> rxyz)
        {
            Eigen::Quaterniond q = Eigen::Quaterniond(dh::geometry::zxy_euler_angle_to_dcm(rxyz));
            return q;
        }

        /**
         * @brief convert zxy euler angle to quaternion.
         * 
         * @param rx rotation around x axis, angle unit is radian.
         * @param ry rotation around y axis, angle unit is radian.
         * @param rz rotation around z axis, angle unit is radian.
         * @return Eigen::Quaterniond 
         */
        inline Eigen::Quaterniond zxy_euler_angle_to_quat(const double rx, const double ry, const double rz)
        {
            Eigen::Quaterniond q = Eigen::Quaterniond(dh::geometry::zxy_euler_angle_to_dcm(rx, ry, rz));
            return q;
        }

        /***************************************************************************
         *                 zyx eluer angle -> quaternion, dcm                       *
         ***************************************************************************/

        /**
         * @brief convert zyx euler angle to direction cosine matrix(dcm)
         * 
         * @param rxyz 3x1 vector, in order of rx, ry, rz. angle unit is radian.
         * @return Eigen::Matrix<double, 3, 3> 
         */
        Eigen::Matrix<double, 3, 3> zyx_euler_angle_to_dcm(const Eigen::Matrix<double, 3, 1> rxyz)
        {
            Eigen::Matrix<double, 3, 3> dcm;
            dcm = dh::geometry::euler_angle_to_dcm(rxyz(2), rxyz(1), rxyz(0), dh::type::RotationAxis::Z, dh::type::RotationAxis::Y, dh::type::RotationAxis::X);
            return dcm;
        }

        /**
         * @brief convert zyx euler angle to direction cosine matrix(dcm)
         * 
         * @param rx rotation around x axis, angle unit is radian.
         * @param ry rotation around y axis, angle unit is radian.
         * @param rz rotation around z axis, angle unit is radian.
         * @return Eigen::Matrix<double, 3, 3> 
         */
        inline Eigen::Matrix<double, 3, 3> zyx_euler_angle_to_dcm(const double rx, const double ry, const double rz)
        {
            Eigen::Matrix<double, 3, 3> dcm;
            dcm = dh::geometry::euler_angle_to_dcm(rz, ry, rx, dh::type::RotationAxis::Z, dh::type::RotationAxis::Y, dh::type::RotationAxis::X);
            return dcm;
        }

        /**
         * @brief convert zyx euler angle to quaternion.
         * 
         * @param rxyz 3x1 vector, in order of rx, ry, rz. angle unit is radian.
         * @return Eigen::Quaterniond 
         */
        inline Eigen::Quaterniond zyx_euler_angle_to_quat(const Eigen::Matrix<double, 3, 1> rxyz)
        {
            Eigen::Quaterniond q = Eigen::Quaterniond(dh::geometry::zyx_euler_angle_to_dcm(rxyz));
            return q;
        }

        /**
         * @brief convert zyx euler angle to quaternion.
         * 
         * @param rx rotation around x axis, angle unit is radian.
         * @param ry rotation around y axis, angle unit is radian.
         * @param rz rotation around z axis, angle unit is radian.
         * @return Eigen::Quaterniond 
         */
        inline Eigen::Quaterniond zyx_euler_angle_to_quat(const double rx, const double ry, const double rz)
        {
            Eigen::Quaterniond q = Eigen::Quaterniond(dh::geometry::zyx_euler_angle_to_dcm(rx, ry, rz));
            return q;
        }

        /***************************************************************************
         *                quaternion, dcm <--> yaw, pitch, roll                    *
         ***************************************************************************/

        /**
         * @brief convert direction cosine matrix(dcm) to (yaw pitch roll)
         * 
         * @param dcm 
         * @param type a dh::type::EulerAngleType, default is zyx
         * @return Eigen::Vector3d 
         */
        inline Eigen::Vector3d dcm_to_ypr(const Eigen::Matrix<double, 3, 3> dcm, dh::type::EulerAngleType type = dh::type::ZXY)
        {
            Eigen::Vector3d ypr;
            double yaw, pitch, roll;
            double pitch_thresh_hold = 1 - 1e-6;
            switch (type)
            {
            case dh::type::ZYX:
                if (dcm(2, 0) < pitch_thresh_hold)
                {
                    pitch = -std::asin(dcm(2, 0));
                    roll = std::atan2(dcm(2, 1), dcm(2, 2));
                    yaw = std::atan2(dcm(0, 1), dcm(0, 0));
                }
                else
                {
                    pitch = -std::asin(dcm(2, 0));
                    roll = -std::atan2(dcm(0, 1), dcm(1, 1));
                    yaw = 0;
                }
                break;
            case dh::type::ZXY:
                if (dcm(2, 1) < pitch_thresh_hold)
                {
                    pitch = std::asin(dcm(2, 1));
                    roll = -std::atan2(dcm(2, 0), dcm(2, 2));
                    yaw = -std::atan2(dcm(0, 1), dcm(1, 1));
                }
                else
                {
                    pitch = std::asin(dcm(2, 1));
                    roll = std::atan2(dcm(0, 2), dcm(0, 0));
                    yaw = 0;
                }
                break;
            default:
                throw 1;
            }
            ypr << yaw, pitch, roll;
            return ypr;
        }

        /**
         * @brief convert quaternion to yaw pitch roll
         * 
         * @param q eigen quaternion
         * @param type a dh::type::EulerAngleType, default is zyx
         * @return Eigen::Vector3d 
         */
        inline Eigen::Vector3d quat_to_ypr(const Eigen::Quaterniond q, dh::type::EulerAngleType type = dh::type::ZXY)
        {
            Eigen::Vector3d ypr = dh::geometry::dcm_to_ypr(q.toRotationMatrix(), type);
            return ypr;
        }

        /**
         * @brief convert ypr angle to direction cosine matrix(dcm).
         * 
         * @param ypr 
         * @param type a dh::type::EulerAngleType type value, default value is ZXY
         * @return Eigen::Matrix<double, 3,  3> 
         */
        inline Eigen::Matrix<double, 3, 3> ypr_to_dcm(const Eigen::Vector3d ypr, const dh::type::EulerAngleType type = dh::type::EulerAngleType::ZXY)
        {
            Eigen::Matrix<double, 3, 3> dcm;
            switch (type)
            {
            case dh::type::EulerAngleType::ZXY:
                dcm = dh::geometry::zxy_euler_angle_to_dcm(ypr(1), ypr(2), ypr(0));
                break;
            case dh::type::EulerAngleType::ZYX:
                dcm = dh::geometry::zyx_euler_angle_to_dcm(ypr(2), ypr(1), ypr(0));
            default:
                throw 1;
            }
            return dcm;
        }

        /**
         * @brief convert ypr angle to quaternion
         * 
         * @param ypr 
         * @param type a dh::type::EulerAngleType type value, default value is ZXY
         * @return Eigen::Quaterniond 
         */
        inline Eigen::Quaterniond ypr_to_quat(const Eigen::Vector3d ypr, const dh::type::EulerAngleType type = dh::type::EulerAngleType::ZXY)
        {
            Eigen::Quaterniond q = Eigen::Quaterniond(dh::geometry::ypr_to_dcm(ypr, type));
            return q;
        }

        /***************************************************************************
         *                          quaternion update    todo                       *
         ***************************************************************************/

    } // namespace geometry
} // namespace dh

#endif