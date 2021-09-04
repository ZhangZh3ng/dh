/*
 * @Author: your name
 * @Date: 2021-09-01 19:57:19
 * @LastEditTime: 2021-09-04 16:19:53
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
    std::cout << "********************************************" <<
    "**************************" << std::endl;
  }

  bool Trajectory3d::getAngleVelocityAndAcceleration(Eigen::Vector3d &w,
                                                     Eigen::Vector3d &a,
                                                     const double time_stamp){
    if(time_stamp >= this->total_time)                                                 
      return false;
    
    w << 0, 0, 0;
    a << 0, 0, 0;
    for(std::vector<Motion3d>::iterator it = this->motions.begin();
        it != this->motions.end();
        it++)
    {
      if (time_stamp >= (*it).begin_time && time_stamp < (*it).end_time)
      {
        w(0) += (*it).wy;
        w(1) += (*it).wp;
        w(2) += (*it).wr;
        a(0) += (*it).ax;
        a(1) += (*it).ay;
        a(2) += (*it).az;
      }
    }
    return true;
  }

  bool TrajectoryGenerator::generate(const std::string& filename,
                                     Trajectory3d& trajectory){
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
    std::cout << "Error opening the file: " << filename;
    return false;
  }
    
    double time_stamp = 0;
    Eigen::Vector3d w, a;
    NavigationParameter3d np = trajectory.init_parameter;
    np.euler_angle_type = this->euler_angle_type;

    while(time_stamp < trajectory.total_time){
      trajectory.getAngleVelocityAndAcceleration(w, a, time_stamp);
      np.update(w, a, this->step_time);
      time_stamp += this->step_time;
      
    }
    

    return true;
  }

}   // namespace tg  
}   // namespace dh