/*
 * @Author: your name
 * @Date: 2021-09-15 14:32:01
 * @LastEditTime: 2021-09-15 14:42:38
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/constant.h
 */

#ifndef DH_CONSTANT_H
#define DH_CONSTANT_H

#include<cmath>

namespace dh{
  /***************************************************************************
  *                                 Unit                                     *
  ***************************************************************************/

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


  /***************************************************************************
  *                            WGS84 Parameter                               *
  ***************************************************************************/

  const double C_e = 0.0818191908425;
  const double C_e2 = 0.006694379990121;
  const double C_f = 0.003352810664747;
  const double C_Re = 6378137;
  const double C_Rp = 6356752;
  const double C_wie = 7.292115e-5;
  const double C_ge = 9.7803253359;


  /***************************************************************************
  *                                 Others                                    *
  ***************************************************************************/
  
  const double GREATER_THRESHHOLD = 1e-8;
  const double IF_EQUAL_THRESHHOLD = 1e-8;

}

#endif