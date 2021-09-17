/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-07 10:02:48
 * @LastEditTime: 2021-09-17 15:32:02
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/sins.hpp
 */

#ifndef DH_SINS_H
#define DH_SINS_H

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "constant.h"
#include "geometry.h"

using namespace dh;

namespace dh{

/**
 * @brief Return the radii of earth at lat.
 * 
 * @param lat Latitude in redian.
 * @return Eigen::Vector2d The former is north-sounth radius, the latter is east-west radius.
 */
  inline Eigen::Matrix<double, 2, 1> earth_radii(const double &lat)
  {
    Eigen::Matrix<double, 2, 1> R;
    R(0) = C_Re * (1 - C_e2) / pow((1 - C_e2 * sin(lat) * sin(lat)), 1.5);
    R(1) = C_Re / (pow(1 - C_e2 * sin(lat) * sin(lat), 0.5));
    return R;
  }

  /**
 * @brief Return the gravity projected in East-North-Up frame.
 * 
 * @param lat Latitude in radian.
 * @param alt Altitude in meter.
 */
  inline Eigen::Matrix<double, 3, 1> gravity_in_enu(const double &lat, const double &alt)
  {
      Eigen::Matrix<double, 3, 1> g;
      g(0) = 0;
      g(1) = -8.08e-9 * alt * sin(2 * lat);
      g(2) = -9.7803253359 * (1 + 0.0053024 * pow(sin(lat), 2) - 0.00000582 * pow(sin(2 * lat), 2)) - 3.08e-6 * alt;
      return g;
  }

  /**
 * @brief Return the projection of earth rate in East-North-Up frame.
 * 
 * @param lat Latitude in radian.
 */
  inline Eigen::Matrix<double, 3, 1> wie_in_enu(const double &lat)
  {
      Eigen::Matrix<double, 3, 1> w_ie_n;
      w_ie_n(0) = 0;
      w_ie_n(1) = C_wie * cos(lat);
      w_ie_n(2) = C_wie * sin(lat);
      return w_ie_n;
  }

  /**
 * @brief Return the rotation rate caused by vehicle movement on the earth.
 * 
 * @param lat Latitude in radian.
 * @param alt Altitude in meter.
 * @param ve East velocity in m/s.
 * @param vn North velocity in m/s.
 */
  inline Eigen::Matrix<double, 3, 1> wen_in_enu(const double &lat, const double &alt, const double &ve, const double &vn)
  {
      Eigen::Matrix<double, 3, 1> w_en_n;
      Eigen::Matrix<double, 2, 1> Rmn;
      Rmn = earth_radii(lat);
      w_en_n(0) = -vn / (Rmn(0) + alt);
      w_en_n(1) = ve / (Rmn(1) + alt);
      w_en_n(2) = ve / (Rmn(1) + alt) * tan(lat);
      return w_en_n;
  }

  /**
 * @brief Return the rotation rate of navigation frame with respect to inertial frame.
 * 
 * @param lat Latitude in radian.
 * @param alt Altitude in meter.
 * @param ve East velocity in m/s.
 * @param vn North velocity in m/s.
 */
  inline Eigen::Matrix<double, 3, 1> win_in_enu(const double &lat, const double &alt, const double &ve, const double &vn)
  {
      Eigen::Matrix<double, 3, 1> w_in_n;
      w_in_n = wie_in_enu(lat) + wen_in_enu(lat, alt, ve, vn);
      return w_in_n;
  }

  /**
   * @brief Return sins Mpv matrix, which indicate the derivatives of position w.r.t velocity.
   * 
   * @param lat Latitude in radian.
   * @param alt Altitude in meter.
   */
  inline Eigen::Matrix<double, 3, 3> sins_Mpv_enu(const double &lat, const double &alt)
  {
    Eigen::Matrix<double, 3, 3> mat;
    Eigen::Matrix<double, 2, 1> Rmn = earth_radii(lat);
    mat << 0, 1 / (Rmn(0) + alt), 0, 1 / cos(lat) / (Rmn(1) + alt), 0, 0, 0, 0, 1;
    return mat;
  }

  class SinsInEnuReferenceFrame
  {
  public:
    // quaternion s.t. q*vb = vn.
    Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
    // coordinate of ground velocity in navigation frame.
    Eigen::Vector3d vn_ = Eigen::Vector3d::Zero();
    // latitude(rad), longitude(rad), altitude(m)
    Eigen::Vector3d lla_ = Eigen::Vector3d::Zero();
    // imu output time interval.
    double time_length_ = 0.01;

    SinsInEnuReferenceFrame(const Eigen::Quaterniond _q, const Eigen::Vector3d _vn, const Eigen::Vector3d _lla, const double _time_length = 0.01) : q_(_q), vn_(_vn), lla_(_lla), time_length_(_time_length) {}

    /**
      * @brief update with angular velocity and acceleration.
      * @param w_ib_b angular velocity measured by gyroscope, unit is rad/s.
      * @param f_ib_b acceleration measured by accelerometer, unit is m/s^2.
      */
    void update(const Eigen::Vector3d& w_ib_b, const Eigen::Vector3d& f_ib_b)
    {
      const double dt = this->time_length_;
      const Eigen::Quaterniond q_b_n_0 = this->q_;
      const Eigen::Vector3d v_eb_n_0 = this->vn_;
      const double lat0 = this->lla_(0);
      const double lon0 = this->lla_(1);
      const double alt0 = this->lla_(2);
      const Eigen::Vector3d lla0 = this->lla_;

      // attitude update
      Eigen::Quaterniond q_bt_b0 = rot_to_quat(w_ib_b * dt);
      Eigen::Vector3d w_in_n;
      w_in_n << 0, 0, 0;
      Eigen::Quaterniond q_n0_nt = rot_to_quat(w_in_n * dt);
      Eigen::Quaterniond q_b_n = q_n0_nt * q_b_n_0 * q_bt_b0;
      this->q_ = q_b_n;

      // velocity update
      Eigen::Vector3d f_ib_n = (q_b_n_0 * f_ib_b + q_b_n * f_ib_b) * dt / 2;
      Eigen::Vector3d v_eb_n = v_eb_n_0 + (f_ib_n - gravity_in_enu(lat0, alt0)) * dt;
      this->vn_ = v_eb_n;

      // position update
      Eigen::Vector3d lla = lla0 + sins_Mpv_enu(lat0, alt0) * (v_eb_n + v_eb_n_0) * dt / 2;
      this->lla_ = lla;
    }
  };

  /**
   * @brief convert ecef coordinate to latitude longitude altitude.
   * 
   * @param ecef ecef x,y,z coordinate.
   */
  inline Eigen::Vector3d ecef_to_lla(const Eigen::Vector3d& ecef){
    double x, y, z, lat, lon, alt;
    x = ecef(0);
    y = ecef(1);
    z = ecef(2);
    // longitude:
    lon = atan2(y, x);
    // latitude:
    double t0 = 0, t = 0;
    int iter_times = 0;
    const int MAX_ITER_TIME = 6;
    while (iter_times < MAX_ITER_TIME)
    {
      ++iter_times;
      t = 1 / sqrt(x * x + y * y) * (z + C_Re * C_e2 * t0 / sqrt(1 + (1 - C_e2) * t0 * t0));
      t0 = t;
    }
    lat = atan(t);
    // altitude:
    double Rn = C_Re / sqrt(1 - C_e2 * pow(sin(lat), 2));
    alt = sqrt(x * x + y * y) / cos(lat) - Rn;
    return Eigen::Vector3d(lat, lon, alt);
  }

  /**
   * @brief convert latitude, longitude, altitude to ecef coordinate.
   * 
   * @param lla latitude(rad) longitude(rad) altitude(m)
   */
  inline Eigen::Vector3d lla_to_ecef(const Eigen::Vector3d& lla){
    double lat, lon, alt, x, y, z;
    lat = lla(0);
    lon = lla(1);
    alt = lla(2);
    const double Rn = C_Re / sqrt(1 - C_e2 * pow(sin(lat), 2));
    x = (Rn + alt) * cos(lat) * cos(lon);
    y = (Rn + alt) * cos(lat) * sin(lon);
    z = (Rn * (1 - C_e2) + alt) * sin(lat);
    return Eigen::Vector3d(x, y, z);
  }

} // namespace dh
#endif // DH_SINS_H