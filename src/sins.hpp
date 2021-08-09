/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-07 10:02:48
 * @LastEditTime: 2021-08-09 20:08:38
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/sins.hpp
 */

#ifndef DH_SINS_H
#define DH_SINS_H

#include "core.hpp"
#include "wgs84.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

namespace dh
{
    namespace sins
    {
        /**
         * @brief Return the radii of earth at latitude = lat and altitude = alt.
         * 
         * @param lat Latitude in redian.
         * @return Eigen::Matrix<double, 2, 1> The former is north-sounth radius, the latter is east-west radius.
         */
        inline Eigen::Matrix<double, 2, 1> earth_radii(const double lat)
        {
            Eigen::Matrix<double, 2, 1> R;
            R(0) = dh::wgs84::Re * (1 - dh::wgs84::e2) / std::pow((1 - dh::wgs84::e2 * std::sin(lat) * std::sin(lat)), 1.5);
            R(1) = dh::wgs84::Re / (pow(1 - dh::wgs84::e2 * std::sin(lat) * std::sin(lat), 0.5));
            return R;
        }

        /**
         * @brief Return the gravity projected in East-North-Up frame.
         * 
         * @param lat Latitude in radian.
         * @param alt Altitude in meter.
         * @return Eigen::Matrix<double, 3, 1> 
         */
        inline Eigen::Matrix<double, 3, 1> gravity_in_ENU(const double lat, const double alt)
        {
            Eigen::Matrix<double, 3, 1> g;
            g(0) = 0;
            g(1) = -8.08e-9 * alt * std::sin(2 * lat);
            g(2) = -9.7803253359 * (1 + 0.0053024 * std::pow(std::sin(lat), 2) - 0.00000582 * std::pow(std::sin(2 * lat), 2)) - 3.08e-6 * alt;
            return g;
        }

        class InertialState
        {
        public:
            double lat;   // latitude in radian.
            double lon;   // longitude in radian.
            double alt;   // altitude in meter.
            double pitch; // pitch in radian.
            double roll;  // roll in radian
            double yaw;   // yaw in radian.
            double ve;    // east velocity in m/s.
            double vn;    // north velocity in m/s.
            double vu;    // up velocity in m/s.
            Eigen::Quaterniond q_body_to_reference;
        };

        dh::sins::InertialState &sins_update_in_ENU(dh::sins::InertialState &sins_state, const double time_length, const Eigen::Matrix<double, 3, 1> &w_ib_b, const Eigen::Matrix<double, 3, 1> &f_ib_b)
        {
            const double wie = dh::wgs84::wie;
            Eigen::Matrix<double, 3, 1> alpha = w_ib_b * time_length;
            const double mag_alpha = alpha.norm();
            Eigen::Matrix<double, 3, 1> w_ie_n;
            w_ie_n << 0, std::cos(sins_state.lat), std::sin(sins_state.lat);
            Eigen::Matrix<double, 2, 1> Rmn = dh::sins::earth_radii(sins_state.lat);
            Eigen::Matrix<double, 3, 1> w_en_n;
            w_en_n(0) = -sins_state.vn / (Rmn(0) + sins_state.alt);
            w_en_n(1) = sins_state.ve / (Rmn(1) + sins_state.alt);
            w_en_n(2) = sins_state.ve / (Rmn(1) + sins_state.alt);

            return sins_state;
        }
    }
}

#endif