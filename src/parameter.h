/*
 * @Author: your name
 * @Date: 2021-09-04 15:08:03
 * @LastEditTime: 2021-09-05 09:59:33
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/parameter.h
 */


#ifndef DH_PARAMETER_H
#define DH_PARAMETER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slam/pose_graph_3d/types.h"

#include "type.h"


using namespace dh::type;
using namespace ceres::examples;

namespace dh{
namespace parameter{

  struct NavigationParameter3d{
  public:
    NavigationParameter3d(const double v_yaw, const double v_pitch, const double v_roll,
                          const double v_vx, const double v_vy, const double v_vz,
                          const double v_px, const double v_py, const double v_pz,
                          const EulerAngleType euler_type = ZXY)
        : yaw(v_yaw), pitch(v_pitch), roll(v_roll),
          vx(v_vx), vy(v_vy), vz(v_vz),
          px(v_px), py(v_py), pz(v_pz),
          euler_angle_type(euler_type) {}

    NavigationParameter3d(const Eigen::Vector3d &ypr, const Eigen::Vector3d &vxyz,
                          const Eigen::Vector3d &pxyz, const EulerAngleType euler_type = ZXY)
        : yaw(ypr(0)), pitch(ypr(1)), roll(ypr(2)),
          vx(vxyz(0)), vy(vxyz(1)), vz(vxyz(2)),
          px(pxyz(0)), py(pxyz(1)), pz(pxyz(2)),
          euler_angle_type(euler_type) {}

    void update(const double wy, const double wp, const double wr,
                const double ax, const double ay, const double az,
                const double t);

    void update(const Eigen::Vector3d w,
                const Eigen::Vector3d a,
                const double t);

    static const NavigationParameter3d zeroParameter(const EulerAngleType euler_type = ZXY)
    {
      return NavigationParameter3d(0, 0, 0, 0, 0, 0, 0, 0, 0, euler_type);
    }

    static std::string name() { return "NavigationParameter3d"; }

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

    void navigation_parameter_to_g2o_pose(const NavigationParameter3d &np,
                                          Pose3d &pose);

} // namespace parameter
} // namespace dh

#endif