/*
 * @Author: your name
 * @Date: 2021-09-04 15:21:23
 * @LastEditTime: 2021-09-04 16:23:36
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/parameter.cpp
 */

#include "parameter.h"
#include "geometry.h"

using namespace dh::geometry;

namespace dh{
namespace parameter{

  void NavigationParameter3d::update(const double wy, const double wp, const double wr,
                                     const double ax, const double ay, const double az,
                                     const double t){
    Eigen::Matrix3d C_b2w_0 = ypr_to_dcm(yaw, pitch, roll, this->euler_angle_type);
    Eigen::Vector3d vb_0 = Eigen::Vector3d(vx, vy, vz);
    Eigen::Vector3d vw_0 = C_b2w_0 * vb_0;

    this->yaw += wy * t;
    this->pitch += wp * t;
    this->roll += wr * t;
    this->vx += ax * t;
    this->vy += ay * t;
    this->vz += az * t;

    Eigen::Matrix3d C_b2w = ypr_to_dcm(yaw, pitch, roll, this->euler_angle_type);
    Eigen::Vector3d vb = Eigen::Vector3d(vx, vy, vz);
    Eigen::Vector3d vw = C_b2w * vb;

    Eigen::Vector3d delata_p = (vw + vw_0)/2 * t;
    this->px += delata_p[0];
    this->py += delata_p[1];
    this->pz += delata_p[2];
  }

  void NavigationParameter3d::update(const Eigen::Vector3d w,
                                     const Eigen::Vector3d a,
                                     const double t){
    this->update(w(0), w(1), w(2), a(0), a(1), a(2), t);
  }

} // namespace parameter
} // namespace dh