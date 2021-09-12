/*
 * @Author: your name
 * @Date: 2021-09-04 15:21:23
 * @LastEditTime: 2021-09-12 11:15:15
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/parameter.cpp
 */
#include <iostream>
#include "parameter.h"
#include "geometry.h"

using namespace dh::geometry;

namespace dh{
namespace parameter{

  /***************************************************************************
  *                         LocalNavigationParameter                            *
  ***************************************************************************/

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

  void np_to_imu(const LocalNavigationParameter &np_begin,
                 const LocalNavigationParameter &np_end,
                 ImuMeasurement6d &vimu){
                  
  }

  void np_to_imu(const std::vector<LocalNavigationParameter>& vnp,
                 std::vector<ImuMeasurement6d>& vimu){
    PoseQPV pose;
    std::vector<PoseQPV> vpose;
    for(std::vector<LocalNavigationParameter>::const_iterator it = vnp.begin();
        it != vnp.end();
        ++it){
          np_to_pose(*it, pose);
          vpose.push_back(pose);
    }
    pose_to_imu(vpose, vimu);
  }

  void pose_to_imu(const PoseQPV &p_begin, const PoseQPV &p_end,
                   ImuMeasurement6d &imu)
  {
    const double dt = p_end.time_stamp - p_begin.time_stamp;
    imu.time_stamp = p_begin.time_stamp;
    // Qe = Qb * dq(w*dt)
    imu.w = quat_increment_to_rot(p_begin.q, p_end.q) / dt;
    // imu.w = quat_increment_to_rot(p_begin.q, p_end.q);
    // std::cout << p_begin.q.coeffs()<< std::endl;
    // std::cout << p_end.q.coeffs()<< std::endl;
    // imu.w << 0, 0, 0;
    // imu.w << p_begin.q.w(), p_begin.q.y(), p_begin.q.z();

    // Ve = Vb + R(Qb)*(a*dt)
    Eigen::Vector3d gn;
    gn << 0, 0, -9.8;
    imu.a = p_begin.q.inverse() * ((p_end.v - p_begin.v) / dt - gn);
  }

  void pose_to_imu(const std::vector<PoseQPV> &pose,
                   std::vector<ImuMeasurement6d> &imu)
  {
    imu.clear();
    ImuMeasurement6d imu_now = ImuMeasurement6d::Zero();
    PoseQPV p_begin, p_end;
    for (std::vector<PoseQPV>::const_iterator it_pose = pose.begin();
         it_pose+1 != pose.end();
         ++it_pose)
    {
      p_begin = *it_pose;
      p_end = *(it_pose+1);
      pose_to_imu(p_begin, p_end, imu_now);
      imu.push_back(imu_now);
    }
  }

} // namespace parameter
} // namespace dh