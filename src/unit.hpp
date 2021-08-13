/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-07 09:28:30
 * @LastEditTime: 2021-08-07 09:57:25
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/unit.hpp
 */

#ifndef DH_UNIT_H
#define DH_UNIT_H

#include<cmath>

namespace dh{
    /**
     * @brief Unit constant value.
     * 
     */
    namespace unit{
        const double DEGREE = M_PI/180;     // angle degree
        const double ANGLE_MINUTE = M_PI/180/60;
        const double ANGLE_SECOND = M_PI/180/3600;
        const double cm = 0.01;
        const double DAY = 86400;
        const double DEGREE_PER_HOUR = M_PI/180/3600;
        const double DEGREE_PER_SECOND = M_PI/180;
        const double DEGREE_PER_SqHz = M_PI/180/3600;
        const double DEGREE_PER_SqHOUR = M_PI/180/3600/60;
        const double g = 9.80556;
        const double Gal = 0.01;
        const double HOUR = 3600;
        const double km = 1000;
        const double km_PER_HOUR = 1/3.6;
        const double mg = 9.80556e-3;
        const double ug = 9.80556e-6;
        const double mGal = 1e-5;
        const double MINUTE = 60;
        const double mm = 1e-3;
        const double m_PER_s2_PER_SqHOUR = 1/60;
        const double nm = 1853;
        const double ppm = 1e-6;
        const double ug_PER_SqHOUR = 9.80556e-6/60;
        const double ug_PER_SqHz = 9.80556e-6;
    }
}

#endif