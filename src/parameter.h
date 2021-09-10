/*
 * @Author: your name
 * @Date: 2021-09-04 15:08:03
 * @LastEditTime: 2021-09-10 14:29:33
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/parameter.h
 */


#ifndef DH_PARAMETER_H
#define DH_PARAMETER_H

#include<memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slam/pose_graph_3d/types.h"

#include "type.h"


using namespace dh::type;
using namespace ceres::examples;

namespace dh{
namespace parameter{

  /***************************************************************************
  *                         NavigationParameter3d                             *
  ***************************************************************************/ 

  struct NavigationParameter3d{
  public:
    NavigationParameter3d(const double v_yaw,
                          const double v_pitch,
                          const double v_roll,
                          const double v_vx,
                          const double v_vy,
                          const double v_vz,
                          const double v_px,
                          const double v_py,
                          const double v_pz,
                          const EulerAngleType euler_type = EulerAngleType::ZXY)
        : yaw(v_yaw), pitch(v_pitch), roll(v_roll),
          vx(v_vx), vy(v_vy), vz(v_vz),
          px(v_px), py(v_py), pz(v_pz),
          euler_angle_type(euler_type) {}

    NavigationParameter3d(const Eigen::Vector3d &ypr,
                          const Eigen::Vector3d &vxyz,
                          const Eigen::Vector3d &pxyz,
                          const EulerAngleType euler_type = EulerAngleType::ZXY)
        : yaw(ypr(0)), pitch(ypr(1)), roll(ypr(2)),
          vx(vxyz(0)), vy(vxyz(1)), vz(vxyz(2)),
          px(pxyz(0)), py(pxyz(1)), pz(pxyz(2)),
          euler_angle_type(euler_type) {}
    
    virtual void update(const double wy, const double wp, const double wr,
                const double ax, const double ay, const double az,
                const double t) = 0;
                
    virtual void update(const Eigen::Vector3d w,
                const Eigen::Vector3d a,
                const double t) = 0;

    virtual std::string name() = 0;

    double time_stamp = 0;
    double yaw;
    double pitch;
    double roll;
    double vx;
    double vy;
    double vz;
    double px;
    double py;
    double pz;
    EulerAngleType euler_angle_type;
  };

  inline std::ostream &operator<<(std::ostream &os, const NavigationParameter3d& np){
    os << np.time_stamp
       << " " << np.yaw << " " << np.pitch << " " << np.roll
       << " " << np.vx << " " << np.vy << " " << np.vz
       << " " << np.px << " " << np.py << " " << np.pz;
    return os;
  }

  struct XyzNavigationParameter : public NavigationParameter3d
  {
  public:
    XyzNavigationParameter(const double v_yaw,
                           const double v_pitch,
                           const double v_roll,
                           const double v_vx,
                           const double v_vy,
                           const double v_vz,
                           const double v_px,
                           const double v_py,
                           const double v_pz,
                           const EulerAngleType euler_type = EulerAngleType::ZXY)
        : NavigationParameter3d(v_yaw, v_pitch, v_roll,
                                v_vx, v_vy, v_vz, v_px,
                                v_py, v_pz, euler_type)
    {
    }

    void update(const double wy, const double wp, const double wr,
                const double ax, const double ay, const double az,
                const double t);

    void update(const Eigen::Vector3d w,
                const Eigen::Vector3d a,
                const double t);

    std::string name() { return "NP_XYZ"; }
  };

  struct EnuNavigationParameter : public NavigationParameter3d
  {
  public:
    EnuNavigationParameter(const double v_yaw,
                           const double v_pitch,
                           const double v_roll,
                           const double v_vx,
                           const double v_vy,
                           const double v_vz,
                           const double v_px,
                           const double v_py,
                           const double v_pz,
                           const EulerAngleType euler_type = EulerAngleType::ZXY)
        : NavigationParameter3d(v_yaw, v_pitch, v_roll,
                                v_vx, v_vy, v_vz, v_px,
                                v_py, v_pz, euler_type)
    {
    }
  
    void update(const double wy, const double wp, const double wr,
                const double ax, const double ay, const double az,
                const double t);

    void update(const Eigen::Vector3d w,
                const Eigen::Vector3d a,
                const double t);

    std::string name() { return "NP_ENU"; }
  };

  struct PoseQPV{
  public:
    PoseQPV(const Eigen::Quaterniond &v_q, const Eigen::Vector3d &v_p,
            const Eigen::Vector3d &v_v, const double &v_time_stamp = 0) : q(v_q), p(v_p), v(v_v), time_stamp(v_time_stamp) {}

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d v = Eigen::Vector3d(0, 0, 0);
    double time_stamp = 0;
  };

  bool posePropogate(PoseQPV &pose, const Eigen::Vector3d &w, const Eigen::Vector3d &a,
                     const double &dt)
  {
    const PoseQPV pose0 = pose;
    quat_update_by_rot(pose.q, w * dt);
    pose.v = pose0.v + pose0.q * a * dt;
    pose.p = pose0.p + (pose.v + pose0.v) / 2 * dt;
    return true;
  }

      /***************************************************************************
  *                            Format conversion                             *
  ***************************************************************************/

      void np_to_pose(const XyzNavigationParameter &np,
                      Pose3d &pose);

  void np_to_pose(const EnuNavigationParameter &np,
                  Pose3d &pose);

  template<class NpType>
  void np_to_pose(std::vector<NpType>& vnp,
                  std::vector<Pose3d>& vpose){
    vpose.clear();
    Pose3d pose;
    for(typename std::vector<NpType>::iterator it = vnp.begin();
        it != vnp.end();
        ++it){
      np_to_pose(*it, pose);
      vpose.push_back(pose);
    }               
  }

  /***************************************************************************
  *                             ImuMeasurement6d                             *
  ***************************************************************************/                                      
  struct ImuMeasurement6d{
  public:
    ImuMeasurement6d(const Eigen::Vector3d &v_w,
                     const Eigen::Vector3d &v_a,
                     const double v_time_stamp)
        : w(v_w), a(v_a), time_stamp(v_time_stamp) {}

    ImuMeasurement6d(const double wx, const double wy, const double wz,
                     const double ax, const double ay, const double az,
                     const double v_time_stamp){
      this->w << wx, wy, wz;
      this->a << ax, ay, az;
      this->time_stamp = v_time_stamp;
    }

    Eigen::Vector3d w;
    Eigen::Vector3d a;
    double time_stamp;
  };

  inline std::ostream &operator<<(std::ostream &os, ImuMeasurement6d &imu){
    os << imu.time_stamp << " " << imu.w(0) << " " << imu.w(1)
       << " " << imu.w(2) << " " << imu.a(0) << " " << imu.a(1)
       << " " << imu.a(2);
    return os;
  }


  /***************************************************************************
  *                             For ceres                                     *
  ***************************************************************************/

    inline Pose3d operator-(Pose3d &pe, Pose3d &pb)
  {
    Pose3d pbe;
    pbe.q = pb.q.conjugate() * pe.q;
    pbe.p = pb.q * (pe.p - pb.p);
    return pbe;
  }

} // namespace parameter
} // namespace dh

#endif