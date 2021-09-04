/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-07 09:28:30
 * @LastEditTime: 2021-09-04 15:48:15
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/unit.hpp
 */

#ifndef DH_UNIT_OLD_H
#define DH_UNIT_OLD_H

#include<cmath>

namespace dh{
    /**
     * @brief Unit constant value.
     * 
     */
    namespace unit{
        const double DEGREE = M_PI/180;     
        const double degree = M_PI/180;     
        const double angle_minute = M_PI/180/60;
        const double angle_second = M_PI/180/3600;
        const double cm = 0.01;
        const double day = 86400;
        const double degree_per_hour = M_PI/180/3600;
        const double degree_per_second = M_PI/180;
        const double degree_per_SqHz = M_PI/180/3600;
        const double degree_per_Sqhour = M_PI/180/3600/60;
        const double g = 9.80556;
        const double Gal = 0.01;
        const double hour = 3600;
        const double km = 1000;
        const double km_per_hour = 1/3.6;
        const double mg = 9.80556e-3;
        const double ug = 9.80556e-6;
        const double mGal = 1e-5;
        const double time_minute = 60;
        const double mm = 1e-3;
        const double m_per_s2_per_Sqhour = 1/60;
        const double nm = 1853;
        const double ppm = 1e-6;
        const double ug_per_Sqhour = 9.80556e-6/60;
        const double ug_per_SqHz = 9.80556e-6;
    }
}

#endif