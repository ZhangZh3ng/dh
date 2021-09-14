/*
 * @Author: your name
 * @Date: 2021-09-06 15:33:52
 * @LastEditTime: 2021-09-14 14:14:34
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/imu.h
 */

#ifndef DH_IMU_H
#define DH_IMU_H

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "parameter.h"


namespace dh{

  class Imu6d{
  public:
    bool generateMeasurement();
    
    double sample_rate = 100;
    Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d ng = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d na = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d rwg = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d rwa = Eigen::Vector3d(0, 0, 0);
  };

} // dh
#endif // DH_IMU_H