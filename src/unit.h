/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-07 09:28:30
 * @LastEditTime: 2021-09-13 10:02:49
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/unit.hpp
 */

#ifndef DH_UNIT_H
#define DH_UNIT_H

#include<cmath>

namespace dh{
    const double C_degree = M_PI / 180;
    const double C_angle_minute = M_PI / 180 / 60;
    const double C_angle_second = M_PI / 180 / 3600;
    const double C_cm = 0.01;
    const double C_day = 86400;
    const double C_degree_per_hour = M_PI / 180 / 3600;
    const double C_degree_per_second = M_PI / 180;
    const double C_degree_per_SqHz = M_PI / 180 / 3600;
    const double C_degree_per_Sqhour = M_PI / 180 / 3600 / 60;
    const double C_g = 9.80556;
    const double C_Gal = 0.01;
    const double C_hour = 3600;
    const double C_km = 1000;
    const double C_km_per_hour = 1 / 3.6;
    const double C_mg = 9.80556e-3;
    const double C_ug = 9.80556e-6;
    const double C_mGal = 1e-5;
    const double C_time_minute = 60;
    const double C_mm = 1e-3;
    const double C_m_per_s2_per_Sqhour = 1 / 60;
    const double C_nm = 1853;
    const double C_ppm = 1e-6;
    const double C_ug_per_Sqhour = 9.80556e-6 / 60;
    const double C_ug_per_SqHz = 9.80556e-6;
} // namespace dh
#endif // DH_UNIT_H