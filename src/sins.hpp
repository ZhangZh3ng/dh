/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-07 10:02:48
 * @LastEditTime: 2021-08-07 11:35:06
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/sins.hpp
 */

#ifndef DH_SINS_H
#define DH_SINS_H

#include "core.hpp"
#include <math.h>

namespace dh{
    namespace sins{
        /**
         * @brief Return the radii of earth at latitude = lat and altitude = alt.
         * 
         * @param lat Latitude in redian.
         * @param alt Altitude in m.
         * @return Eigen::Matrix<double, 2, 1> 
         */
        Eigen::Matrix<double, 2, 1> earth_radii(const double lat, const double alt){
            Eigen::Matrix<double, 2, 1> R;
            return R;
        }
    }
}

#endif