/*
 * @Author: your name
 * @Date: 2021-09-04 15:21:23
 * @LastEditTime: 2021-09-10 15:15:14
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/parameter.cpp
 */

#include "parameter.h"
#include "geometry.h"

using namespace dh::geometry;

namespace dh{
namespace parameter{

  /***************************************************************************
  *                         NavigationParameter3d                            *
  ***************************************************************************/

  void NavigationParameter3d::update(const double &wy, const double &wp, const double &wr,
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

  void NavigationParameter3d::update(const Eigen::Vector3d &w,
                                     const Eigen::Vector3d &a,
                                     const double &dt)
  {
    this->update(w(0), w(1), w(2), a(0), a(1), a(2), dt);
  }

  bool posePropogate(PoseQPV &pose, const Eigen::Vector3d &w, const Eigen::Vector3d &a,
                     const double &dt)
  {
    const PoseQPV pose0 = pose;
    quat_update_by_rot(pose.q, w * dt);
    pose.v = pose0.v + pose0.q * a * dt;
    pose.p = pose0.p + (pose.v + pose0.v) / 2 * dt;
    return true;
  }

  void np_to_pose(const NavigationParameter3d &np,
                  Pose3d &pose)
  {
    pose.p(0) = np.px;
    pose.p(1) = np.py;
    pose.p(2) = np.pz;
    pose.q = ypr_to_quat(np.yaw, np.pitch, np.roll,
                         np.euler_angle_type);
  }

} // namespace parameter
} // namespace dh