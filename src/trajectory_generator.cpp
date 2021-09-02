/*
 * @Author: your name
 * @Date: 2021-09-01 19:57:19
 * @LastEditTime: 2021-09-02 21:23:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/trajectory_generator.cpp
 */

#include "trajectory_generator.h"

namespace dh{
namespace tg{
  
  void Trajectory3d::briefReport() {
    std::cout << std::left;
    std::cout << "********************************************" <<
    "**************************" << std::endl;
    std::cout << "Total time: " << this->total_time << "s" <<std::endl;
    std::cout << "********************************************" <<
    "**************************" << std::endl;
    std::cout << "UNIT";
    std::cout << "       Time:s" << 
    "      Angular velocity:rad/s"<<
    "   Acceleration:m/s2" << std::endl;
    std::cout << "Index\t" << "Begin\t" << "End\t" << "Wy\t" << "Wp\t" 
    << "Wr\t" << "Ax\t" << "Ay\t" << "Az\t" << std::endl;

    int index = 1;
    for (std::vector<Motion3d>::iterator it = this->motions.begin();
         it != this->motions.end(); it++){
      std::cout << index++ << "\t";
      std::cout << *it << std::endl;
    }
  }

  void NavigationParameter3d::update(const double wy, const double wp, const double wr,
                                     const double ax, const double ay, const double az,
                                     const double t){
    Eigen::Matrix3d C_b2w_0 = ypr_to_dcm(yaw, pitch, roll, this->euler_angle_type);
    Eigen::Vector3d vb_0 = Eigen::Vector3d(vx, vy, vz);
    Eigen::Vector3d vw_0 = C_b2w_0 * vb_0;

    this->yaw += wy * t;
    this->pitch += wp * t;
    this->roll += wr * t;
    this->vx += ax * t;
    this->vy += ay * t;
    this->vz += az * t;

    Eigen::Matrix3d C_b2w = ypr_to_dcm(yaw, pitch, roll, this->euler_angle_type);
    Eigen::Vector3d vb = Eigen::Vector3d(vx, vy, vz);
    Eigen::Vector3d vw = C_b2w * vb;

    Eigen::Vector3d delata_p = (vw + vw_0)/2 * t;
    this->px += delata_p[0];
    this->py += delata_p[1];
    this->pz += delata_p[2];
  }


  bool TrajectoryGenerator3d::generate(const std::string& filename,
                                       const Trajectory3d& traj){
    this->trajectory = traj;
    for(std::vector<Motion3d>::iterator it = this->trajectory.motions.begin();
        it != this->trajectory.motions.end(); it++)

    return true;
  }

}   // namespace tg  
}   // namespace dh