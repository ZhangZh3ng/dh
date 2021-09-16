/*
 * @Author: your name
 * @Date: 2021-09-04 15:08:03
 * @LastEditTime: 2021-09-16 21:25:04
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/parameter.h
 */


#ifndef DH_NAVIGATION_PARAMETER_H
#define DH_NAVIGATION_PARAMETER_H

#include<memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ceres_example_slam/pose_graph_3d/types.h"
#include "type.h"
#include "geometry.h"


using namespace ceres::examples;

namespace dh{

  /***************************************************************************
  *                         NavigationParameter3d                             *
  ***************************************************************************/ 

  struct LocalNavigationParameter{
  public:
    LocalNavigationParameter(
        const double &_yaw, const double &_pitch,
        const double &_roll, const double &_vx,
        const double &_vy, const double &_vz,
        const double &_px, const double &_py,
        const double &_pz, const double &_time_stamp = 0,
        const EulerAngleType &euler_type = EulerAngleType::ZXY)
        : yaw(_yaw), pitch(_pitch), roll(_roll),
          vx(_vx), vy(_vy), vz(_vz),
          px(_px), py(_py), pz(_pz), time_stamp(_time_stamp),
          euler_angle_type(euler_type) {}

    LocalNavigationParameter(
        const Eigen::Vector3d &ypr,
        const Eigen::Vector3d &vxyz,
        const Eigen::Vector3d &pxyz,
        const double &_time_stamp = 0,
        const EulerAngleType &euler_type = EulerAngleType::ZXY)
        : yaw(ypr(0)), pitch(ypr(1)), roll(ypr(2)),
          vx(vxyz(0)), vy(vxyz(1)), vz(vxyz(2)),
          px(pxyz(0)), py(pxyz(1)), pz(pxyz(2)), time_stamp(_time_stamp),
          euler_angle_type(euler_type) {}

    virtual void update(const double &wy, const double &wp, const double &wr,
                        const double &ax, const double &ay, const double &az,
                        const double &dt);

    virtual void update(const Eigen::Vector3d &w,
                        const Eigen::Vector3d &a,
                        const double &dt);

    Eigen::Quaterniond q() const {return ypr_to_quat(yaw, pitch, roll, euler_angle_type);}
    Eigen::Vector3d v() const {return Eigen::Vector3d(vx, vy, vz);}
    Eigen::Vector3d p() const {return Eigen::Vector3d(px, py, pz);}
    Eigen::Vector3d ypr() const {return Eigen::Vector3d(yaw, pitch, roll);}
    Eigen::Matrix3d dcm() const {return ypr_to_dcm(yaw, pitch, roll, euler_angle_type);}

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

  inline std::ostream &operator<<(std::ostream &os, const LocalNavigationParameter& np){
    os << np.time_stamp
       << " " << np.yaw << " " << np.pitch << " " << np.roll
       << " " << np.vx << " " << np.vy << " " << np.vz
       << " " << np.px << " " << np.py << " " << np.pz;
    return os;
  }

  /***************************************************************************
  *                                 PoseQPV                                  *
  ***************************************************************************/ 

 /**
  * @brief add velocity and time stamp to ceres::Poses3d.
  */
  struct PoseQPV{
  public:
    PoseQPV(){}
    PoseQPV(const Eigen::Quaterniond &_q, const Eigen::Vector3d &_p,
            const Eigen::Vector3d &_v, const double &_time_stamp = 0) : q(_q), p(_p), v(_v), time_stamp(_time_stamp) {}

    void updateByAcc(const Eigen::Vector3d &w, const Eigen::Vector3d&a, const double &dt);
    std::string name() const {return "PoseQPV";}

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d v = Eigen::Vector3d(0, 0, 0);
    double time_stamp = 0;
  };

  inline std::ostream &operator<<(std::ostream &os, const PoseQPV &pose)
  {
    os << pose.time_stamp << " "
       << pose.q.w() << " " << pose.q.x() << " " << pose.q.y() << " " << pose.q.z() << " "
       << pose.p(0) << " " << pose.p(1) << " " << pose.p(2) << " "
       << pose.v(0) << " " << pose.v(1) << " " << pose.v(2);
    return os;
  }

  /***************************************************************************
  *                            Format conversion                             *
  ***************************************************************************/
  void np_to_pose(const LocalNavigationParameter &np,
                  Pose3d &pose);

  void np_to_pose(const LocalNavigationParameter &np,
                  PoseQPV &pose);

  template<class NpType, class PoseType>
  void np_to_pose(std::vector<NpType>& vnp,
                  std::vector<PoseType>& vpose){
    vpose.clear();
    PoseType pose;
    for(typename std::vector<NpType>::iterator it = vnp.begin();
        it != vnp.end();
        ++it){
      np_to_pose(*it, pose);
      vpose.push_back(pose);
    }               
  }

} // namespace dh

#endif