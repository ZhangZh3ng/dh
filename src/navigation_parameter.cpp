/*
 * @Author: your name
 * @Date: 2021-09-04 15:21:23
 * @LastEditTime: 2021-09-16 21:25:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/parameter.cpp
 */
#include <iostream>

// #include <opencv2/core/core.hpp>

#include "navigation_parameter.h"
#include "geometry.h"

using namespace dh;

namespace dh{

  void LocalNavigationParameter::update(const double &wy, const double &wp, const double &wr,
                                     const double &ax, const double &ay, const double &az,
                                     const double &dt)
  {
    Eigen::Matrix3d C_b2w_0 = ypr_to_dcm(yaw, pitch, roll, this->euler_angle_type);
    Eigen::Vector3d vb_0 = Eigen::Vector3d(vx, vy, vz);
    Eigen::Vector3d vw_0 = C_b2w_0 * vb_0;

    this->yaw += wy * dt;
    this->pitch += wp * dt;
    this->roll += wr * dt;
    this->vx += ax * dt;
    this->vy += ay * dt;
    this->vz += az * dt;

    Eigen::Matrix3d C_b2w = ypr_to_dcm(yaw, pitch, roll, this->euler_angle_type);
    Eigen::Vector3d vb = Eigen::Vector3d(vx, vy, vz);
    Eigen::Vector3d vw = C_b2w * vb;

    Eigen::Vector3d delata_p = (vw + vw_0)/2 * dt;
    this->px += delata_p[0];
    this->py += delata_p[1];
    this->pz += delata_p[2];

    this->time_stamp += dt;
  }

  void LocalNavigationParameter::update(const Eigen::Vector3d &w,
                                     const Eigen::Vector3d &a,
                                     const double &dt)
  {
    this->update(w(0), w(1), w(2), a(0), a(1), a(2), dt);
  }

  void PoseQPV::updateByAcc(const Eigen::Vector3d &w, const Eigen::Vector3d&a, const double &dt){
    const Eigen::Quaterniond q0 = this->q;
    const Eigen::Vector3d p0 = this->p;
    const Eigen::Vector3d v0 = this->v;
    this->q = quat_add_rot(this->q, w * dt);
    this->v = v0 + q0 * a * dt;
    this->p = p + (v0 + this->v) / 2 * dt;
    this->time_stamp += dt;
  }

  void np_to_pose(const LocalNavigationParameter &np,
                  Pose3d &pose)
  {
    pose.p(0) = np.px;
    pose.p(1) = np.py;
    pose.p(2) = np.pz;
    pose.q = ypr_to_quat(np.yaw, np.pitch, np.roll,
                         np.euler_angle_type);
  }

  void np_to_pose(const LocalNavigationParameter &np,
                  PoseQPV &pose){
    pose.p(0) = np.px;
    pose.p(1) = np.py;
    pose.p(2) = np.pz;
    pose.v(0) = np.vx;
    pose.v(1) = np.vy;
    pose.v(2) = np.vz;
    pose.time_stamp = np.time_stamp;
    pose.q = ypr_to_quat(np.yaw, np.pitch, np.roll,
                         np.euler_angle_type);                
  }

} // namespace dh