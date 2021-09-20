/*
 * @Author: your name
 * @Date: 2021-09-20 09:44:25
 * @LastEditTime: 2021-09-20 10:36:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/format_transform.h
 */

#ifndef DH_FORMATE_TRANSFORM_H  
#define DH_FORMATE_TRANSFORM_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "imu_parameter.h"
#include "navigation_parameter.h"

namespace dh{

  inline void set_preintegration_parameter(
      double (&pose)[7],
      double (&speedBias)[9],
      const LocalNavigationParameter &np,
      const Eigen::Vector3d &ba,
      const Eigen::Vector3d &bg)
  {
    pose[0] = np.px;
    pose[1] = np.py;
    pose[2] = np.pz;
    pose[3] = np.q().x();
    pose[4] = np.q().y();
    pose[5] = np.q().z();
    pose[6] = np.q().w();
    speedBias[0] = np.vx;
    speedBias[1] = np.vy;
    speedBias[2] = np.vz;
    speedBias[3] = ba(0);
    speedBias[4] = ba(1);
    speedBias[5] = ba(2);
    speedBias[6] = bg(0);
    speedBias[7] = bg(1);
    speedBias[8] = bg(2);
  }

  inline void set_preintegration_parameter(
      double (&pose)[7],
      double (&speedBias)[9],
      const LocalNavigationParameter &np,
      const ImuErrorParameter &imuerr)
  {
    set_preintegration_parameter(pose, speedBias, np, imuerr.ba, imuerr.bg);
  }
}

#endif // DH_FORMATE_TRANSFORM_H