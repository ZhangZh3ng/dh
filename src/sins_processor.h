/*
 * @Author: your name
 * @Date: 2021-09-17 10:47:52
 * @LastEditTime: 2021-09-17 15:34:44
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/sins_processor.h
 */

#ifndef DH_SINS_PROCESSOR_H
#define DH_SINS_PROCESSOR_H

#include "sins.h"
#include "imu_parameter.h"
#include "navigation_parameter.h"

namespace dh{
  class SimpleLocalSinsProcessor{
  public:
    SimpleLocalSinsProcessor(const LocalNavigationParameter &_nav_param)
        : nav_param(_nav_param){}
    void propagate(const ImuMeasurement6d& imu, const double& dt);
    LocalNavigationParameter nav_param;
  };
}

#endif