/*
 * @Author: your name
 * @Date: 2021-09-04 15:08:03
 * @LastEditTime: 2021-09-11 11:10:23
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

using namespace ceres::examples;
using namespace dh::type;

namespace dh{
namespace parameter{

  /***************************************************************************
  *                         NavigationParameter3d                             *
  ***************************************************************************/ 

  struct NavigationParameter3d{
  public:
    NavigationParameter3d(const double &v_yaw,
                          const double &v_pitch,
                          const double &v_roll,
                          const double &v_vx,
                          const double &v_vy,
                          const double &v_vz,
                          const double &v_px,
                          const double &v_py,
                          const double &v_pz,
                          const double &v_time_stamp = 0,
                          const EulerAngleType &euler_type = EulerAngleType::ZXY)
        : yaw(v_yaw), pitch(v_pitch), roll(v_roll),
          vx(v_vx), vy(v_vy), vz(v_vz),
          px(v_px), py(v_py), pz(v_pz), time_stamp(v_time_stamp),
          euler_angle_type(euler_type) {}

    NavigationParameter3d(const Eigen::Vector3d &ypr,
                          const Eigen::Vector3d &vxyz,
                          const Eigen::Vector3d &pxyz,
                          const double &v_time_stamp = 0,
                          const EulerAngleType &euler_type = EulerAngleType::ZXY)
        : yaw(ypr(0)), pitch(ypr(1)), roll(ypr(2)),
          vx(vxyz(0)), vy(vxyz(1)), vz(vxyz(2)),
          px(pxyz(0)), py(pxyz(1)), pz(pxyz(2)), time_stamp(v_time_stamp),
          euler_angle_type(euler_type) {}

    virtual void update(const double &wy, const double &wp, const double &wr,
                        const double &ax, const double &ay, const double &az,
                        const double &dt);

    virtual void update(const Eigen::Vector3d &w,
                        const Eigen::Vector3d &a,
                        const double &dt);

    virtual std::string name(){ return "Np"; }

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

  void np_to_pose(const NavigationParameter3d &np,
                  Pose3d &pose);

  /***************************************************************************
  *                                 PoseQPV                                  *
  ***************************************************************************/ 

  struct PoseQPV{
  public:
    PoseQPV(){}
    PoseQPV(const Eigen::Quaterniond &v_q, const Eigen::Vector3d &v_p,
            const Eigen::Vector3d &v_v, const double &v_time_stamp = 0) : q(v_q), p(v_p), v(v_v), time_stamp(v_time_stamp) {}

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d v = Eigen::Vector3d(0, 0, 0);
    double time_stamp = 0;
  };

  inline bool poseUpdate(PoseQPV &pose, const Eigen::Vector3d &w, const Eigen::Vector3d &a,
                         const double &dt);

  /***************************************************************************
  *                            Format conversion                             *
  ***************************************************************************/
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
                     const double& v_time_stamp)
        : w(v_w), a(v_a), time_stamp(v_time_stamp) {}

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

  inline std::ostream &operator<<(std::ostream &os, ImuMeasurement6d &imu){
    os << imu.time_stamp << " " << imu.w(0) << " " << imu.w(1)
       << " " << imu.w(2) << " " << imu.a(0) << " " << imu.a(1)
       << " " << imu.a(2);
    return os;
  }

  void pose_to_imu(const PoseQPV &p_begin, const PoseQPV &p_end,
                   ImuMeasurement6d &imu);

  void pose_to_imu(const std::vector<PoseQPV> &pose,
                   std::vector<ImuMeasurement6d> &imu);

  /***************************************************************************
  *                             For ceres                                     *
  ***************************************************************************/

  inline Pose3d operator-(Pose3d &p_end, Pose3d &p_begin)
  {
    Pose3d pbe;
    pbe.q = p_begin.q.conjugate() * p_end.q;
    pbe.p = p_begin.q * (p_end.p - p_begin.p);
    return pbe;
  }

} // namespace parameter
} // namespace dh

#endif