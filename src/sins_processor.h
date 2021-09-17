/*
 * @Author: your name
 * @Date: 2021-09-17 10:47:52
 * @LastEditTime: 2021-09-17 14:33:27
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
    SimpleLocalSinsProcessor(const LocalNavigationParameter &_np)
        : np(_np), time_stamp(_np.time_stamp) {}
    void propogate(const ImuMeasurement6d& imu, const double& dt);
    LocalNavigationParameter np;
    double time_stamp = 0;
  };
}

#endif