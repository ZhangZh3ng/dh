/*
 * @Author: your name
 * @Date: 2021-08-07 11:29:12
 * @LastEditTime: 2021-09-13 10:08:20
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/wgs84.hpp
 */

#ifndef DH_WGS84_H
#define DH_WGS84_H

namespace dh{
    const double C_e = 0.0818191908425;
    const double C_e2 = 0.006694379990121;
    const double C_f = 0.003352810664747;
    const double C_Re = 6378137;
    const double C_Rp = 6356752;
    const double C_wie = 7.292115e-5;
    const double C_ge = 9.7803253359;
} // namespace dh

#endif // DH_WGS84_H