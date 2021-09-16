/*
 * @Author: your name
 * @Date: 2021-09-02 20:13:13
 * @LastEditTime: 2021-09-16 19:18:04
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/geometry.h
 */

#ifndef DH_GEOMETRY_H_
#define DH_GEOMETRY_H_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "type.h"


namespace dh{

  inline Eigen::Matrix3d rmat(const double r, RotationAxis axis){
    Eigen::Matrix3d mat;
    switch (axis)
    {
    case RotationAxis::X:
      mat = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
      break;
    case RotationAxis::Y:
      mat = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitY());
      break;
    case RotationAxis::Z:
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

  inline Eigen::Quaterniond euler_angle_to_quat(const double r1,
                                                const double r2,
                                                const double r3, RotationAxis axis1,
                                                RotationAxis axis2,
                                                RotationAxis axis3)
  {
    Eigen::Quaterniond q = dcm_to_quat(euler_angle_to_dcm(r1, r2, r3, axis1, axis2, axis3));
    return q;
  }

  /***************************************************************************
  *                 zyx eluer angle -> quaternion, dcm                       *
  ***************************************************************************/

  inline Eigen::Matrix<double, 3, 3> zxy_euler_angle_to_dcm(const Eigen::Vector3d rxyz){
    Eigen::Matrix3d dcm;
    dcm = euler_angle_to_dcm(rxyz(2), rxyz(0), rxyz(1),
                             RotationAxis::Z, 
                             RotationAxis::X, 
                             RotationAxis::Y);
    return dcm;
  }

  inline Eigen::Matrix3d zxy_euler_angle_to_dcm(const double rx,
                                                const double ry, 
                                                const double rz){
    Eigen::Matrix3d dcm;
    dcm = euler_angle_to_dcm(rz, rx, ry,
                             RotationAxis::Z,
                             RotationAxis::X,
                             RotationAxis::Y);
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

  inline Eigen::Matrix3d zyx_euler_angle_to_dcm(const Eigen::Vector3d rxyz){
    Eigen::Matrix3d dcm;
    dcm = euler_angle_to_dcm(rxyz(2), rxyz(1), rxyz(0),
                             RotationAxis::Z,
                             RotationAxis::Y,
                             RotationAxis::X);
    return dcm;
  }

  inline Eigen::Matrix3d zyx_euler_angle_to_dcm(const double rx,
                                                const double ry, 
                                                const double rz){
    Eigen::Matrix3d dcm;
    dcm = euler_angle_to_dcm(rz, ry, rx,
                             RotationAxis::Z,
                             RotationAxis::Y,
                             RotationAxis::X);
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
                                    EulerAngleType type = EulerAngleType::ZXY){
    Eigen::Vector3d ypr;
    double yaw, pitch, roll;
    double pitch_thresh_hold = 1 - 1e-6;
    switch (type)
    {
    case EulerAngleType::ZYX:
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
    case EulerAngleType::ZXY:
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
                                     EulerAngleType type = EulerAngleType::ZXY){
    Eigen::Vector3d ypr = dcm_to_ypr(q.toRotationMatrix(), type);
    return ypr;
  }

  inline Eigen::Matrix<double, 3, 3> ypr_to_dcm(const Eigen::Vector3d ypr,
                                                const EulerAngleType type = EulerAngleType::ZXY){
    Eigen::Matrix3d dcm;
    switch (type)
    {
    case EulerAngleType::ZXY:
      dcm = zxy_euler_angle_to_dcm(ypr(1), ypr(2), ypr(0));
      break;
    case EulerAngleType::ZYX:
      dcm = zyx_euler_angle_to_dcm(ypr(2), ypr(1), ypr(0));
    default:
      throw 1;
    }
    return dcm;
  }

  inline Eigen::Matrix3d ypr_to_dcm(const double yaw,
                                    const double pitch,
                                    const double roll,
                                    const EulerAngleType type = EulerAngleType::ZXY)
  {
    Eigen::Matrix3d dcm;
    switch (type)
    {
    case EulerAngleType::ZXY:
      dcm = zxy_euler_angle_to_dcm(yaw, pitch, roll);
      break;
    case EulerAngleType::ZYX:
      dcm = zyx_euler_angle_to_dcm(yaw, pitch, roll);
    default:
      throw 1;
    }
    return dcm;
  }

  inline Eigen::Quaterniond ypr_to_quat(const Eigen::Vector3d ypr,
                                        const EulerAngleType type = EulerAngleType::ZXY){
    Eigen::Quaterniond q = Eigen::Quaterniond(ypr_to_dcm(ypr, type));
    return q;
  }

  inline Eigen::Quaterniond ypr_to_quat(const double yaw,
                                        const double pitch,
                                        const double roll, 
                                        const EulerAngleType type = EulerAngleType::ZXY){
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
    const double sin_half_norm_rot = sin(norm_rot / 2);
    w = cos(norm_rot / 2);
    x = rot(0) / norm_rot * sin_half_norm_rot;
    y = rot(1) / norm_rot * sin_half_norm_rot;
    z = rot(2) / norm_rot * sin_half_norm_rot;
    return Eigen::Quaterniond(w, x, y, z);
  }

  inline Eigen::Quaterniond quat_add_rot(const Eigen::Quaterniond &q,
                                         const Eigen::Vector3d &rot)
  {
    Eigen::Quaterniond qout;
    const double norm_rot = rot.norm();
    const double sin_half_norm_rot = sin(norm_rot / 2);
    double w, x, y, z;
    w = cos(norm_rot / 2);
    x = rot(0) / norm_rot * sin_half_norm_rot;
    y = rot(1) / norm_rot * sin_half_norm_rot;
    z = rot(2) / norm_rot * sin_half_norm_rot;
    Eigen::Quaterniond dq = Eigen::Quaterniond(w, x, y, z);
    qout = q * dq;
    return qout;
  }

  inline Eigen::Quaterniond quat_add_rot(Eigen::Quaterniond &q,
                                         const double &rotx,
                                         const double &roty,
                                         const double &rotz)
  {
    Eigen::Vector3d rot(rotx, roty, rotz);
    return quat_add_rot(q, rot);
  }

  inline Eigen::Vector3d quat_increment_to_rot(const Eigen::Quaterniond &q_begin,
                                               const Eigen::Quaterniond &q_end)
  {
    Eigen::Vector3d rot;
    const Eigen::Quaterniond dq = q_begin.inverse() * q_end;
    const double norm_rot = 2*acos(dq.w());
    if (norm_rot < 1e-8){
      rot << 0, 0, 0;
    }
    else{
      const double rot_sin_half_norm_rot = norm_rot / sin(norm_rot / 2);
      rot(0) = dq.x() * rot_sin_half_norm_rot;
      rot(1) = dq.y() * rot_sin_half_norm_rot;
      rot(2) = dq.z() * rot_sin_half_norm_rot;
    }
    return rot;
  }

  /***************************************************************************
  *                                 others                                   *
  ***************************************************************************/
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

} // namespace dh
#endif // DH_GEOMETRY_H_