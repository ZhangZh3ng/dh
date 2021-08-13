/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-07 10:02:48
 * @LastEditTime: 2021-08-13 17:15:49
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
        inline Eigen::Matrix<double, 3, 1> gravity_in_enu(const double lat, const double alt)
        {
            Eigen::Matrix<double, 3, 1> g;
            g(0) = 0;
            g(1) = -8.08e-9 * alt * std::sin(2 * lat);
            g(2) = -9.7803253359 * (1 + 0.0053024 * std::pow(std::sin(lat), 2) - 0.00000582 * std::pow(std::sin(2 * lat), 2)) - 3.08e-6 * alt;
            return g;
        }

        /**
         * @brief Return the projection of earth rate in East-North-Up frame.
         * 
         * @param lat Latitude in radian.
         * @return Eigen::Matrix<double, 3, 1> 
         */
        inline Eigen::Matrix<double, 3, 1> wie_in_enu(const double lat)
        {
            const double wie = dh::wgs84::wie;
            Eigen::Matrix<double, 3, 1> w_ie_n;
            w_ie_n(0) = 0;
            w_ie_n(1) = wie * std::cos(lat);
            w_ie_n(2) = wie * std::sin(lat);
            return w_ie_n;
        }

        /**
         * @brief Return the rotation rate caused by vehicle movement on the earth.
         * 
         * @param lat Latitude in radian.
         * @param alt Altitude in meter.
         * @param ve East velocity in m/s.
         * @param vn North velocity in m/s.
         * @return Eigen::Matrix<double, 3, 1> 
         */
        inline Eigen::Matrix<double, 3, 1> wen_in_enu(const double lat, const double alt, const double ve, const double vn)
        {
            Eigen::Matrix<double, 3, 1> w_en_n;
            Eigen::Matrix<double, 2, 1> Rmn;
            Rmn = dh::sins::earth_radii(lat);
            w_en_n(0) = -vn / (Rmn(0) + alt);
            w_en_n(1) = ve / (Rmn(1) + alt);
            w_en_n(2) = ve / (Rmn(1) + alt) * std::tan(lat);
            return w_en_n;
        }

        /**
         * @brief Return the rotation rate of navigation frame with respect to inertial frame.
         * 
         * @param lat Latitude in radian.
         * @param alt Altitude in meter.
         * @param ve East velocity in m/s.
         * @param vn North velocity in m/s.
         * @return Eigen::Matrix<double, 3, 1> 
         */
        inline Eigen::Matrix<double, 3, 1> win_in_enu(const double lat, const double alt, const double ve, const double vn)
        {
            Eigen::Matrix<double, 3, 1> w_in_n;
            w_in_n = dh::sins::wie_in_enu(lat) + dh::sins::wen_in_enu(lat, alt, ve, vn);
            return w_in_n;
        }

        /**
         * @brief Return sins Mpv matrix, which indicate the derivatives of position w.r.t velocity.
         * 
         * @param lat Latitude in radian.
         * @param alt Altitude in meter.
         * @return Eigen::Matrix<double, 3 ,3> 
         */
        inline Eigen::Matrix<double, 3, 3> sins_Mpv(const double lat, const double alt)
        {
            Eigen::Matrix<double, 3, 3> mat;
            Eigen::Matrix<double, 2, 1> Rmn = dh::sins::earth_radii(lat);
            mat << 0, 1 / (Rmn(0) + alt), 0, 1 / std::cos(lat) / (Rmn(1) + alt), 0, 0, 0, 0, 1;
            return mat;
        }

        class NavigationStateInENUFrame
        {
        public:
            double latitude;                    // latitude in radian.
            double longitude;                   // longitude in radian.
            double altitude;                    // altitude in meter.
            double ve;                          // east velocity in m/s.
            double vn;                          // north velocity in m/s.
            double vu;                          // up velocity in m/s.
            Eigen::Quaternion<double> q_b_to_n; // quaternion from body frame to navigation frame.
        };

        /**
         * @brief Strap-down intertial navigation update, reference frame is East-North-Up.
         * 
         * @param ins Navigation state.
         * @param time_length Time interval between start and end time.
         * @param w_ib_b Anguler rate measured by three-axis gyroscope, in rad/s.
         * @param f_ib_b Linear acceleration measured by three-axis accelerometer, in m/s^2;
         * @return dh::sins::InertialState& 
         */
        inline dh::sins::NavigationStateInENUFrame &sins_update_in_enu(dh::sins::NavigationStateInENUFrame &ins, const double time_length, const Eigen::Matrix<double, 3, 1> &w_ib_b, const Eigen::Matrix<double, 3, 1> &f_ib_b)
        {
            dh::sins::NavigationStateInENUFrame ins0 = ins;
            // caculate angular incrementation.
            Eigen::Matrix<double, 3, 1> alpha = w_ib_b * time_length;
            Eigen::Quaterniond q_bnew_to_bold = dh::core::angle_increment_2_quat(alpha);
            Eigen::Matrix<double, 3, 1> w_ie_n = dh::sins::wie_in_enu(ins.latitude);
            Eigen::Matrix<double, 3, 1> w_en_n = dh::sins::wen_in_enu(ins.latitude, ins.altitude, ins.ve, ins.vn);
            Eigen::Matrix<double, 3, 1> w_in_n = dh::sins::win_in_enu(ins.latitude, ins.altitude, ins.ve, ins.vn);
            Eigen::Quaterniond q_nnew_to_nold = dh::core::angle_increment_2_quat(w_in_n * time_length);
            ins.q_b_to_n = q_nnew_to_nold.conjugate() * ins.q_b_to_n * q_bnew_to_bold;

            Eigen::Matrix<double, 3, 1> a_nb_n;
            Eigen::Matrix<double, 3, 1> vel0, vel1, vel_avg;
            vel0 << ins.ve, ins.vn, ins.vu;

            a_nb_n = (ins.q_b_to_n * f_ib_b + ins0.q_b_to_n * f_ib_b) * time_length / 2 - dh::core::cross_product((2 * w_ie_n + w_en_n), vel0) * time_length + dh::sins::gravity_in_enu(ins.latitude, ins.altitude) * time_length;

            vel1 = vel0 + a_nb_n * time_length;
            vel_avg = (vel0 + vel1) / 2;
            Eigen::Matrix<double, 3, 3> mpv = dh::sins::sins_Mpv(ins.latitude, ins.altitude);
            Eigen::Matrix<double, 3, 1> pos0, pos1;
            pos0 << ins.latitude, ins.longitude, ins.altitude;
            pos1 = pos0 + mpv * vel_avg * time_length;

            ins.latitude = pos1(0);
            ins.longitude = pos1(1);
            ins.altitude = pos1(2);
            ins.ve = vel1(0);
            ins.vn = vel1(1);
            ins.vu = vel1(2);

            return ins;
        }
    }
}

#endif