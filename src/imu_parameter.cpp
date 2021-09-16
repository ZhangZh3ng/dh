/*
 * @Author: your name
 * @Date: 2021-09-16 21:17:52
 * @LastEditTime: 2021-09-16 21:20:05
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/imu_parameter.cpp
 */

#include <opencv2/core/core.hpp>

#include "imu_parameter.h"
#include "navigation_parameter.h"

namespace dh{
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
         ++it_pose){
      p_begin = *it_pose;
      p_end = *(it_pose+1);
      pose_to_imu(p_begin, p_end, imu_now);
      imu.push_back(imu_now);
    }
  }

  void imu_add_error(std::vector<ImuMeasurement6d> &vimu,
                     const ImuErrorParameter &err){
    cv::RNG rng;
    Eigen::Vector3d rw_bg, rw_ba;
    Eigen::Vector3d bg_now, ba_now;
    Eigen::Vector3d noise_error_g, noise_error_a;
    rw_bg << 0, 0, 0;
    rw_ba << 0, 0, 0;
    const double dt = 1 / err.sample_rate;
    const double sqdt = sqrt(dt);
    for (std::vector<ImuMeasurement6d>::iterator it = vimu.begin();
         it != vimu.end();
         ++it)
    {
      rw_bg(0) += rng.gaussian(err.rwg(0)) * sqdt;
      rw_bg(1) += rng.gaussian(err.rwg(1)) * sqdt;
      rw_bg(2) += rng.gaussian(err.rwg(2)) * sqdt;
      rw_ba(0) += rng.gaussian(err.rwa(0)) * sqdt;
      rw_ba(1) += rng.gaussian(err.rwa(1)) * sqdt;
      rw_ba(2) += rng.gaussian(err.rwa(2)) * sqdt;
      bg_now = err.bg + rw_bg;
      ba_now = err.ba + rw_ba;

      noise_error_g(0) = rng.gaussian(err.ng(0)) / sqdt;
      noise_error_g(1) = rng.gaussian(err.ng(1)) / sqdt;
      noise_error_g(2) = rng.gaussian(err.ng(2)) / sqdt;
      noise_error_a(0) = rng.gaussian(err.na(0)) / sqdt;
      noise_error_a(1) = rng.gaussian(err.na(1)) / sqdt;
      noise_error_a(2) = rng.gaussian(err.na(2)) / sqdt;

      (*it).w += (bg_now + noise_error_g);
      (*it).a += (ba_now + noise_error_a);
    }
  }

} // namespace dh