/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-07 10:02:48
 * @LastEditTime: 2021-09-11 16:02:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/sins.hpp
 */

#ifndef DH_SINS_H
#define DH_SINS_H

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// #include "core.hpp"
#include "wgs84.h"
#include "geometry.h"

using namespace dh::geometry;

namespace dh{
namespace sins{

/**
 * @brief Return the radii of earth at latitude = lat and altitude = alt.
 * 
 * @param lat Latitude in redian.
 * @return Eigen::Matrix<double, 2, 1> The former is north-sounth radius, the latter is east-west radius.
 */
  inline Eigen::Matrix<double, 2, 1> earth_radii(const double &lat)
  {
    Eigen::Matrix<double, 2, 1> R;
    R(0) = dh::wgs84::Re * (1 - dh::wgs84::e2) / std::pow((1 - dh::wgs84::e2 * std::sin(lat) * std::sin(lat)), 1.5);
    R(1) = dh::wgs84::Re / (pow(1 - dh::wgs84::e2 * std::sin(lat) * std::sin(lat), 0.5));
    return R;
  }

  /**
 * @brief Return the gravity projected in East-North-Up frame.
 * 
 * @param lat Latitude in radian.
 * @param alt Altitude in meter.
 * @return Eigen::Matrix<double, 3, 1> 
 */
  inline Eigen::Matrix<double, 3, 1> gravity_in_enu(const double &lat, const double &alt)
  {
      Eigen::Matrix<double, 3, 1> g;
      g(0) = 0;
      g(1) = -8.08e-9 * alt * std::sin(2 * lat);
      g(2) = -9.7803253359 * (1 + 0.0053024 * std::pow(std::sin(lat), 2) - 0.00000582 * std::pow(std::sin(2 * lat), 2)) - 3.08e-6 * alt;
      return g;
  }

  /**
 * @brief Return the projection of earth rate in East-North-Up frame.
 * 
 * @param lat Latitude in radian.
 * @return Eigen::Matrix<double, 3, 1> 
 */
  inline Eigen::Matrix<double, 3, 1> wie_in_enu(const double &lat)
  {
      const double wie = dh::wgs84::wie;
      Eigen::Matrix<double, 3, 1> w_ie_n;
      w_ie_n(0) = 0;
      w_ie_n(1) = wie * std::cos(lat);
      w_ie_n(2) = wie * std::sin(lat);
      return w_ie_n;
  }

  /**
 * @brief Return the rotation rate caused by vehicle movement on the earth.
 * 
 * @param lat Latitude in radian.
 * @param alt Altitude in meter.
 * @param ve East velocity in m/s.
 * @param vn North velocity in m/s.
 * @return Eigen::Matrix<double, 3, 1> 
 */
  inline Eigen::Matrix<double, 3, 1> wen_in_enu(const double &lat, const double &alt, const double &ve, const double &vn)
  {
      Eigen::Matrix<double, 3, 1> w_en_n;
      Eigen::Matrix<double, 2, 1> Rmn;
      Rmn = dh::sins::earth_radii(lat);
      w_en_n(0) = -vn / (Rmn(0) + alt);
      w_en_n(1) = ve / (Rmn(1) + alt);
      w_en_n(2) = ve / (Rmn(1) + alt) * std::tan(lat);
      return w_en_n;
  }

  /**
 * @brief Return the rotation rate of navigation frame with respect to inertial frame.
 * 
 * @param lat Latitude in radian.
 * @param alt Altitude in meter.
 * @param ve East velocity in m/s.
 * @param vn North velocity in m/s.
 * @return Eigen::Matrix<double, 3, 1> 
 */
  inline Eigen::Matrix<double, 3, 1> win_in_enu(const double &lat, const double &alt, const double &ve, const double &vn)
  {
      Eigen::Matrix<double, 3, 1> w_in_n;
      w_in_n = dh::sins::wie_in_enu(lat) + dh::sins::wen_in_enu(lat, alt, ve, vn);
      return w_in_n;
  }

  /**
   * @brief Return sins Mpv matrix, which indicate the derivatives of position w.r.t velocity.
   * 
   * @param lat Latitude in radian.
   * @param alt Altitude in meter.
   * @return Eigen::Matrix<double, 3 ,3> 
   */
  inline Eigen::Matrix<double, 3, 3> sins_Mpv_enu(const double &lat, const double &alt)
  {
    Eigen::Matrix<double, 3, 3> mat;
    Eigen::Matrix<double, 2, 1> Rmn = dh::sins::earth_radii(lat);
    mat << 0, 1 / (Rmn(0) + alt), 0, 1 / std::cos(lat) / (Rmn(1) + alt), 0, 0, 0, 0, 1;
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

    SinsInEnuReferenceFrame(const Eigen::Quaterniond q, const Eigen::Vector3d vn, const Eigen::Vector3d lla, const double time_length = 0.01) : q_(q), vn_(vn), lla_(lla), time_length_(time_length) {}

    /**
      * @brief update with angular velocity and acceleration.
      * @param w_ib_b angular velocity measured by gyroscope, unit is rad/s.
      * @param f_ib_b acceleration measured by accelerometer, unit is m/s^2.
      */
    void update(const Eigen::Vector3d w_ib_b, const Eigen::Vector3d f_ib_b)
    {
      const double t = this->time_length_;
      const Eigen::Quaterniond q_b_n_0 = this->q_;
      const Eigen::Vector3d v_eb_n_0 = this->vn_;
      const double lat0 = this->lla_(0);
      const double lon0 = this->lla_(1);
      const double alt0 = this->lla_(2);
      const Eigen::Vector3d lla0 = this->lla_;

      // attitude update
      Eigen::Quaterniond q_bt_b0 = dh::geometry::rot_to_quat(w_ib_b * t);
      Eigen::Vector3d w_in_n;
      w_in_n << 0, 0, 0;
      Eigen::Quaterniond q_n0_nt = dh::geometry::rot_to_quat(w_in_n * t);
      Eigen::Quaterniond q_b_n = q_n0_nt * q_b_n_0 * q_bt_b0;
      this->q_ = q_b_n;

      // velocity update
      Eigen::Vector3d f_ib_n = (q_b_n_0 * f_ib_b + q_b_n * f_ib_b) * t / 2;
      Eigen::Vector3d v_eb_n = v_eb_n_0 + (f_ib_n - dh::sins::gravity_in_enu(lat0, alt0)) * t;
      this->vn_ = v_eb_n;

      // position update
      Eigen::Vector3d lla = lla0 + dh::sins::sins_Mpv_enu(lat0, alt0) * (v_eb_n + v_eb_n_0) * t / 2;
      this->lla_ = lla;
    }
  };

  inline Eigen::Vector3d ecef_to_lla(const Eigen::Vector3d& ecef){
    Eigen::Vector3d lla;
    const double Re = dh::wgs84::Re;
    const double e2 = dh::wgs84::e2;
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
      t = 1 / sqrt(x * x + y * y) * (z + Re * e2 * t0 / sqrt(1 + (1 - e2) * t0 * t0));
      t0 = t;
    }
    lat = atan(t);
    // altitude:
    double Rn = Re / sqrt(1 - e2 * pow(sin(lat), 2));
    alt = sqrt(x * x + y * y) / cos(lat) - Rn;
    lla << lat, lon, alt;
    return lla;
  }

  inline Eigen::Vector3d lla_to_ecef(const Eigen::Vector3d& lla){
    Eigen::Vector3d ecef;
    double lat, lon, alt, x, y, z;
    lat = lla(0);
    lon = lla(1);
    alt = lla(2);
    const double Re = dh::wgs84::Re;
    const double e2 = dh::wgs84::e2;
    const double Rn = Re / sqrt(1 - e2 * pow(sin(lat), 2));
    x = (Rn + alt) * cos(lat) * cos(lon);
    y = (Rn + alt) * cos(lat) * sin(lon);
    z = (Rn * (1 - e2) + alt) * sin(lat);
    ecef << x, y, z;
    return ecef;
  }

} // namespace sins
} // namespace dh

#endif // DH_SINS_H