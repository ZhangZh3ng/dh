/*
 * @Author: your name
 * @Date: 2021-09-01 19:57:19
 * @LastEditTime: 2021-09-15 15:45:16
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/trajectory_generator.cpp
 */

#include "trajectory_generator.h"
#include "mywrite.h"

using namespace dh;

namespace dh{

  /***************************************************************************
  *                             Trajectory3d                                 *
  ***************************************************************************/

  void Trajectory3d::briefReport() const {
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
    for (std::vector<Motion3d>::const_iterator it = this->motions.begin();
         it != this->motions.end();
         it++){
      std::cout << index++ << "\t";
      std::cout << *it << std::endl;
    }
    std::cout << "********************************************" <<
    "**************************" << std::endl;
  }

  bool Trajectory3d::getAngleVelocityAndAcceleration(Eigen::Vector3d &w,
                                                     Eigen::Vector3d &a,
                                                     const double time_stamp) const {
    if(time_stamp >= this->total_time)                                                 
      return false;
    
    w << 0, 0, 0;
    a << 0, 0, 0;
    for(std::vector<Motion3d>::const_iterator it = this->motions.begin();
        it != this->motions.end();
        it++){
      if ( GreaterOrAlmostEqual(time_stamp, (*it).begin_time)
          && DefinitelyGreater((*it).end_time, time_stamp))
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

}   // namespace dh