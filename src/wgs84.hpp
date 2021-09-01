/*
 * @Author: your name
 * @Date: 2021-08-07 11:29:12
 * @LastEditTime: 2021-09-01 20:07:47
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/wgs84.hpp
 */

#ifndef DH_WGS84_H
#define DH_WGS84_H

namespace dh{
    namespace wgs84{
        const double e = 0.0818191908425;
        const double e2 = 0.006694379990121;
        const double f = 0.003352810664747;
        const double Re = 6378137;
        const double Rp = 6356752;
        const double wie = 7.292115e-5;
        const double ge = 9.7803253359;
    }
}

#endif