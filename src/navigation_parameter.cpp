/*
 * @Author: your name
 * @Date: 2021-09-04 15:21:23
 * @LastEditTime: 2021-09-17 15:34:41
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/parameter.cpp
 */
#include <iostream>

#include "navigation_parameter.h"
#include "geometry.h"

using namespace dh;

namespace dh{

  void LocalNavigationParameter::propagate(
      const double &wy, const double &wp, const double &wr,
      const double &ax, const double &ay, const double &az,
      const double &dt)
  {
    Eigen::Matrix3d C_b2w_0 = ypr_to_dcm(yaw, pitch, roll,
                                         this->euler_angle_type);
    Eigen::Vector3d vb_0 = Eigen::Vector3d(vx, vy, vz);
    Eigen::Vector3d vw_0 = C_b2w_0 * vb_0;

    yaw += wy * dt;
    pitch += wp * dt;
    roll += wr * dt;
    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;

    Eigen::Matrix3d C_b2w = ypr_to_dcm(yaw, pitch, roll,
                                       this->euler_angle_type);
    Eigen::Vector3d vb = Eigen::Vector3d(vx, vy, vz);
    Eigen::Vector3d vw = C_b2w * vb;

    Eigen::Vector3d delata_p = (vw + vw_0)/2 * dt;
    px += delata_p[0];
    py += delata_p[1];
    pz += delata_p[2];

    this->time_stamp += dt;
  }

  void LocalNavigationParameter::propagate(const Eigen::Vector3d &w,
                                           const Eigen::Vector3d &a,
                                           const double &dt)
  {
    this->propagate(w(0), w(1), w(2), a(0), a(1), a(2), dt);
  }

  void PoseQPV::propagate(const Eigen::Vector3d &w,
                               const Eigen::Vector3d &a,
                               const double &dt)
  {
    const Eigen::Quaterniond q0 = q;
    const Eigen::Vector3d p0 = p;
    const Eigen::Vector3d v0 = v;
    q = quat_add_rot(q, w * dt);
    v = v0 + q0 * a * dt;
    p = p + (v0 + v) / 2 * dt;
    time_stamp += dt;
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