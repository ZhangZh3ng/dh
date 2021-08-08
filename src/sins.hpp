/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-07 10:02:48
 * @LastEditTime: 2021-08-08 09:15:53
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/sins.hpp
 */

#ifndef DH_SINS_H
#define DH_SINS_H

#include "core.hpp"
#include "wgs84.hpp"
#include <math.h>

namespace dh{
    namespace sins{
        /**
         * @brief Return the radii of earth at latitude = lat and altitude = alt.
         * 
         * @param lat Latitude in redian.
         * @return Eigen::Matrix<double, 2, 1> The former is north-sounth radius, the latter is east-west radius.
         */
        inline Eigen::Matrix<double, 2, 1> earth_radii(const double lat){
            Eigen::Matrix<double, 2, 1> R;
            R(0) = dh::wgs84::Re * (1 - dh::wgs84::e2)/std::pow((1-dh::wgs84::e2 * std::sin(lat)*std::sin(lat)),1.5);
            R(1) = dh::wgs84::Re / (pow(1-dh::wgs84::e2*std::sin(lat)*std::sin(lat),0.5));
            return R;
        }
    }
}

#endif