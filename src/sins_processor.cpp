/*
 * @Author: your name
 * @Date: 2021-09-17 10:54:21
 * @LastEditTime: 2021-09-17 14:36:42
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/sins_processor.cpp
 */

#include "sins_processor.h"
#include "geometry.h"

namespace dh{
  void SimpleLocalSinsProcessor::propogate(const ImuMeasurement6d& imu,
  const double& dt){
    if (dt < 0)
      throw 1;
    Eigen::Vector3d gn(0, 0, -9.8);
    Eigen::Vector3d ab = imu.a + np.q() * gn;
    np.propogate(imu.w, ab, dt);
  }
}