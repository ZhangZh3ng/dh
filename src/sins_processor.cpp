/*
 * @Author: your name
 * @Date: 2021-09-17 10:54:21
 * @LastEditTime: 2021-09-17 15:34:52
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/sins_processor.cpp
 */

#include "sins_processor.h"
#include "geometry.h"

namespace dh{
  void SimpleLocalSinsProcessor::propagate(
      const ImuMeasurement6d &imu,
      const double &dt){
    Eigen::Vector3d gw(0, 0, -9.8); // gravity in world frame.
    Eigen::Vector3d ab = imu.a + nav_param.q() * gw;
    nav_param.propagate(imu.w, ab, dt);
  }
}