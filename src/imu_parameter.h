/*
 * @Author: your name
 * @Date: 2021-09-06 15:33:52
 * @LastEditTime: 2021-09-20 09:02:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/imu.h
 */

#ifndef DH_IMU_H
#define DH_IMU_H

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "navigation_parameter.h"


namespace dh{

struct ImuMeasurement6d{
  public:
    ImuMeasurement6d(const Eigen::Vector3d &_w,
                     const Eigen::Vector3d &_a,
                     const double& _time_stamp)
        : w(_w), a(_a), time_stamp(_time_stamp) {}

    ImuMeasurement6d(const double &wx, const double &wy, const double &wz,
                     const double &ax, const double &ay, const double &az,
                     const double &v_time_stamp)
    {
      this->w << wx, wy, wz;
      this->a << ax, ay, az;
      this->time_stamp = v_time_stamp;
    }

    static ImuMeasurement6d Zero(){
        return ImuMeasurement6d(Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(), 0);}

    Eigen::Vector3d w;
    Eigen::Vector3d a;
    double time_stamp;
  };

  inline ImuMeasurement6d operator-(const ImuMeasurement6d &a,
                                    const ImuMeasurement6d &b)
  {
    double time_stamp = 0;
    if (abs(a.time_stamp - b.time_stamp) < 1e-8)
      time_stamp = a.time_stamp;
    return ImuMeasurement6d(a.w - b.w, a.a - b.a, time_stamp);
  }

  inline std::ostream &operator<<(std::ostream &os, const ImuMeasurement6d &imu){
    os << imu.time_stamp << " " << imu.w(0) << " " << imu.w(1)
       << " " << imu.w(2) << " " << imu.a(0) << " " << imu.a(1)
       << " " << imu.a(2);
    return os;
  }

  typedef std::vector<ImuMeasurement6d> ImuVector;

  inline ImuVector operator-(const ImuVector &v1,
                             const ImuVector &v2){
    ImuVector vout;
    vout.clear();
    if(v1.size() != v2.size())
      return vout;
    
    for (ImuVector::const_iterator it1 = v1.begin(), it2 = v2.begin();
         it1 != v1.end();
         ++it1, ++it2)
      vout.push_back((*it1) - (*it2));
    return vout;
  }

  void np_to_imu(const LocalNavigationParameter &np_begin,
                 const LocalNavigationParameter &np_end,
                 ImuMeasurement6d &imu);

  void np_to_imu(const std::vector<LocalNavigationParameter>& vnp,
                 std::vector<ImuMeasurement6d>& vimu);

  void pose_to_imu(const PoseQPV &p_begin,
                   const PoseQPV &p_end,
                   ImuMeasurement6d &imu);

  void pose_to_imu(const std::vector<PoseQPV> &pose,
                   std::vector<ImuMeasurement6d> &imu);
  
  /***************************************************************************
  *                             ImuErrorParameter                            *
  ***************************************************************************/

  class ImuErrorParameter{
  public:

    Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);  // gyroscope bias, rad/s
    Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);  // accelerometer bias, m/s^2
    Eigen::Vector3d ng = Eigen::Vector3d(0, 0, 0);  // gyroscope noise root PSD, rad/s^0.5
    Eigen::Vector3d na = Eigen::Vector3d(0, 0, 0);  // accelerometer noise root PSD, m/s^1.5
    Eigen::Vector3d rwg = Eigen::Vector3d(0, 0, 0); // gyroscope random walk root PSD, rad/s^1.5
    Eigen::Vector3d rwa = Eigen::Vector3d(0, 0, 0); // accelerometer random walk root PSD, m/s^2.5
    double sample_rate = 100;                       // in Hz

    static ImuErrorParameter Zero(){
      return ImuErrorParameter();
    }
  };

  inline std::ostream &operator<<(std::ostream &os, const ImuErrorParameter &err)
  {
    os << "sample rate = " << err.sample_rate << " (HZ)" << std::endl;
    
    os << "gyroscope bias = " << err.bg(0) << " " << err.bg(1) 
    << " " << err.bg(2) << " (rad/s)" << std::endl;
    os << "gyroscope noise = " << err.ng(0) << " " << err.ng(1) 
    << " " << err.ng(2) << " (rad/s^0.5)" << std::endl;
    os << "gyroscope bias random walk = " << err.rwg(0) << " " << err.rwg(1) 
    << " " << err.rwg(2) << " (rad/s^1.5)" << std::endl;

    os << "accelerometer bias = " << err.ba(0) << " " << err.ba(1) 
    << " " << err.ba(2) << " (m/s^2)" << std::endl;
    os << "accelerometer noise = " << err.na(0) << " " << err.na(1) 
    << " " << err.na(2) << " (m/s^1.5)" << std::endl;
    os << "accelerometer bias random walk = " << err.rwa(0) << " " << err.rwa(1) 
    << " " << err.rwa(2) << " (m/s^2.5)";
    return os;
  }

  void imu_add_error(std::vector<ImuMeasurement6d> &imu,
                     const ImuErrorParameter &err);

} // dh
#endif // DH_IMU_PARAMETER_H