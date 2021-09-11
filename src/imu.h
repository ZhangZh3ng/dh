/*
 * @Author: your name
 * @Date: 2021-09-06 15:33:52
 * @LastEditTime: 2021-09-06 16:16:32
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


using namespace dh::parameter;

namespace dh{
namespace sensor{

  class Imu6d{
  public:
    bool generateMeasurement();
    
    double sample_rate = 100;
    Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d ng = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d na = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d nbg = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d nba = Eigen::Vector3d(0, 0, 0);
  };

}
}







#endif // DH_IMU_H