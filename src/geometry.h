/*
 * @Author: your name
 * @Date: 2021-09-02 20:13:13
 * @LastEditTime: 2021-09-04 16:24:36
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/geometry.h
 */

#ifndef DH_GEOMETRY_H
#define DH_GEOMETRY_H

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "type.h"

using namespace dh::type;

namespace dh{
namespace geometry{

  inline Eigen::Matrix3d rmat(const double r, RotationAxis axis){
    Eigen::Matrix3d mat;
    switch (axis)
    {
    case X:
      mat = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
      break;
    case Y:
      mat = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitY());
      break;
    case Z:
      mat = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitZ());
      break;
    }
    return mat;
  }
  
  inline Eigen::Matrix3d euler_angle_to_dcm(const double r1,
                                            const double r2, 
                                            const double r3, 
                                            RotationAxis axis1, 
                                            RotationAxis axis2, 
                                            RotationAxis axis3){
    if (axis1 == axis2 || axis2 == axis3)
    {
      throw 1;
    }
    Eigen::Matrix3d dcm;
    dcm = rmat(r1, axis1) * rmat(r2, axis2) * rmat(r3, axis3);
    return dcm;
  }

  inline Eigen::Quaterniond dcm_to_quat(const Eigen::Matrix3d dcm){
    Eigen::Quaterniond q = Eigen::Quaterniond(dcm);
    return q;
  }

  /***************************************************************************
  *                 zyx eluer angle -> quaternion, dcm                       *
  ***************************************************************************/

  Eigen::Matrix<double, 3, 3> zxy_euler_angle_to_dcm(const Eigen::Vector3d rxyz){
    Eigen::Matrix3d dcm;
    dcm = euler_angle_to_dcm(rxyz(2), rxyz(0), rxyz(1), Z, X, Y);
    return dcm;
  }

  inline Eigen::Matrix3d zxy_euler_angle_to_dcm(const double rx,
                                                const double ry, 
                                                const double rz){
    Eigen::Matrix3d dcm;
    dcm = euler_angle_to_dcm(rz, rx, ry, Z, X, Y);
    return dcm;
  }

  inline Eigen::Quaterniond zxy_euler_angle_to_quat(const Eigen::Vector3d rxyz){
    Eigen::Quaterniond q = Eigen::Quaterniond(zxy_euler_angle_to_dcm(rxyz));
    return q;
  }

  inline Eigen::Quaterniond zxy_euler_angle_to_quat(const double rx,
                                                    const double ry, 
                                                    const double rz){
    Eigen::Quaterniond q = Eigen::Quaterniond(zxy_euler_angle_to_dcm(rx, ry, rz));
    return q;
  }

  /***************************************************************************
  *                 zyx eluer angle -> quaternion, dcm                       *
  ***************************************************************************/

  Eigen::Matrix3d zyx_euler_angle_to_dcm(const Eigen::Vector3d rxyz){
    Eigen::Matrix3d dcm;
    dcm = euler_angle_to_dcm(rxyz(2), rxyz(1), rxyz(0), Z, Y, X);
    return dcm;
  }

  inline Eigen::Matrix3d zyx_euler_angle_to_dcm(const double rx,
                                                const double ry, 
                                                const double rz){
    Eigen::Matrix3d dcm;
    dcm = euler_angle_to_dcm(rz, ry, rx, Z, Y, X);
    return dcm;
  }

  inline Eigen::Quaterniond zyx_euler_angle_to_quat(const Eigen::Vector3d rxyz){
    Eigen::Quaterniond q = Eigen::Quaterniond(zyx_euler_angle_to_dcm(rxyz));
    return q;
  }

  inline Eigen::Quaterniond zyx_euler_angle_to_quat(const double rx,
                                                    const double ry, 
                                                    const double rz){
    Eigen::Quaterniond q = Eigen::Quaterniond(zyx_euler_angle_to_dcm(rx, ry, rz));
    return q;
  }

  /***************************************************************************
  *                quaternion, dcm <--> yaw, pitch, roll                    *
  ***************************************************************************/

  inline Eigen::Vector3d dcm_to_ypr(const Eigen::Matrix3d dcm,
                                    EulerAngleType type = ZXY){
    Eigen::Vector3d ypr;
    double yaw, pitch, roll;
    double pitch_thresh_hold = 1 - 1e-6;
    switch (type)
    {
    case ZYX:
      if (dcm(2, 0) < pitch_thresh_hold)
      {
        pitch = -std::asin(dcm(2, 0));
        roll = std::atan2(dcm(2, 1), dcm(2, 2));
        yaw = std::atan2(dcm(0, 1), dcm(0, 0));
      }
      else
      {
        pitch = -std::asin(dcm(2, 0));
        roll = -std::atan2(dcm(0, 1), dcm(1, 1));
        yaw = 0;
      }
      break;
    case ZXY:
      if (dcm(2, 1) < pitch_thresh_hold)
      {
        pitch = std::asin(dcm(2, 1));
        roll = -std::atan2(dcm(2, 0), dcm(2, 2));
        yaw = -std::atan2(dcm(0, 1), dcm(1, 1));
      }
      else
      {
        pitch = std::asin(dcm(2, 1));
        roll = std::atan2(dcm(0, 2), dcm(0, 0));
        yaw = 0;
      }
      break;
    default:
      throw 1;
    }
    ypr << yaw, pitch, roll;
    return ypr;
  }

  inline Eigen::Vector3d quat_to_ypr(const Eigen::Quaterniond q,
                                     EulerAngleType type = ZXY){
    Eigen::Vector3d ypr = dcm_to_ypr(q.toRotationMatrix(), type);
    return ypr;
  }

  inline Eigen::Matrix<double, 3, 3> ypr_to_dcm(const Eigen::Vector3d ypr,
                                                const EulerAngleType type = ZXY){
    Eigen::Matrix3d dcm;
    switch (type)
    {
    case ZXY:
      dcm = zxy_euler_angle_to_dcm(ypr(1), ypr(2), ypr(0));
      break;
    case ZYX:
      dcm = zyx_euler_angle_to_dcm(ypr(2), ypr(1), ypr(0));
    default:
      throw 1;
    }
    return dcm;
  }

  inline Eigen::Matrix3d ypr_to_dcm(const double yaw,
                                    const double pitch,
                                    const double roll, 
                                    const EulerAngleType type = ZXY){
    Eigen::Matrix3d dcm;
    switch (type)
    {
    case ZXY:
      dcm = zxy_euler_angle_to_dcm(yaw, pitch, roll);
      break;
    case ZYX:
      dcm = zyx_euler_angle_to_dcm(yaw, pitch, roll);
    default:
      throw 1;
    }
    return dcm;
  }

  inline Eigen::Quaterniond ypr_to_quat(const Eigen::Vector3d ypr,
                                        const EulerAngleType type = ZXY){
    Eigen::Quaterniond q = Eigen::Quaterniond(ypr_to_dcm(ypr, type));
    return q;
  }

  inline Eigen::Quaterniond ypr_to_quat(const double yaw,
                                        const double pitch,
                                        const double roll, 
                                        const EulerAngleType type = ZXY){
    Eigen::Quaterniond q = Eigen::Quaterniond(ypr_to_dcm(yaw, pitch, roll, type));
    return q;
  }

  /***************************************************************************
  *                          quaternion update                               *
  ***************************************************************************/

  /**
   * @brief convert rotation to quaternion.
   * q = |    cos(r/2)    | 
   *     |rx/r * sin(r/2) |
   *     |ry/r * sin(r/2) |
   *     |rz/r * sin(r/2) |
   */
  inline Eigen::Quaterniond rot_to_quat(const Eigen::Vector3d rot){
    double w, x, y, z;
    const double norm_rot = rot.norm();
    const double sin_half_norm_rot = std::sin(norm_rot / 2);
    w = std::cos(norm_rot / 2);
    x = rot(0) / norm_rot * sin_half_norm_rot;
    y = rot(1) / norm_rot * sin_half_norm_rot;
    z = rot(2) / norm_rot * sin_half_norm_rot;
    Eigen::Quaterniond q = Eigen::Quaterniond(w, x, y, z);
    return q;
  }

  inline const Eigen::Quaterniond &quat_add_rot(Eigen::Quaterniond &q,
                                                const Eigen::Vector3d &rot){
    const double norm_rot = rot.norm();
    const double sin_half_norm_rot = std::sin(norm_rot / 2);
    double w, x, y, z;
    w = std::cos(norm_rot / 2);
    x = rot(0) / norm_rot * sin_half_norm_rot;
    y = rot(1) / norm_rot * sin_half_norm_rot;
    z = rot(2) / norm_rot * sin_half_norm_rot;
    Eigen::Quaterniond dq = Eigen::Quaterniond(w, x, y, z);
    q = q * dq;
    return q;
  }

  inline const Eigen::Quaterniond &quat_add_rot(Eigen::Quaterniond &q, 
                                                const double &rotx,
                                                const double &roty,
                                                const double &rotz){
    const double norm_rot = std::pow(rotx * rotx + roty * roty + rotz * rotz, 0.5);
    const double sin_half_norm_rot = std::sin(norm_rot / 2);
    double w, x, y, z;
    w = std::cos(norm_rot / 2);
    x = rotx / norm_rot * sin_half_norm_rot;
    y = roty / norm_rot * sin_half_norm_rot;
    z = rotz / norm_rot * sin_half_norm_rot;
    Eigen::Quaterniond dq = Eigen::Quaterniond(w, x, y, z);
    q = q * dq;
    return q;
  }

  inline bool ypr_standerlize(double &yaw, double &pitch, double &roll){
    bool has_change = false;
    while (yaw < 0 || yaw > 2*M_PI)
    {
      has_change = true;
      if (yaw < 0)
        yaw += 2*M_PI;
      else
        yaw -= 2*M_PI;
    }

    while (pitch > M_PI_2 || pitch < -M_PI_2)
    {
      has_change = true;
      if (pitch > M_PI_2)
        pitch = M_PI - pitch;
      else
        pitch = M_PI + pitch;
    }

    while (roll > M_PI || roll < -M_PI)
    {
      if (roll > M_PI)
        roll -= 2 * M_PI;
      else
        roll += 2 * M_PI;
    }
    return has_change;
  }

} // namespace geometry
} // namespace dh
#endif