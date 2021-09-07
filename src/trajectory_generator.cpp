/*
 * @Author: your name
 * @Date: 2021-09-01 19:57:19
 * @LastEditTime: 2021-09-07 20:35:07
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/trajectory_generator.cpp
 */

#include "trajectory_generator.h"
#include "write.h"

using namespace dh::write;

namespace dh{
namespace tg{

  /***************************************************************************
  *                             Trajectory3d                                 *
  ***************************************************************************/

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

    double threshhold = 1e-8;
    for(std::vector<Motion3d>::iterator it = this->motions.begin();
        it != this->motions.end();
        it++)
    {
      if ( ((*it).begin_time - time_stamp) <  threshhold
           && ((*it).end_time - time_stamp) > threshhold )
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

   /***************************************************************************
  *                          TrajectoryGenerator                               *
  ***************************************************************************/

  bool TrajectoryGenerator::generate(VectorOfNavigationParameter3d &vnp,
                                     Trajectory3d& trajectory){
    vnp.clear();
    Eigen::Vector3d w, a;

    np = *trajectory.initial_parameter;

    np.euler_angle_type = this->euler_angle_type;
    vnp.push_back(np);

    while(np.time_stamp < trajectory.total_time){
      trajectory.getAngleVelocityAndAcceleration(w, a, np.time_stamp);
      np.update(w, a, this->step_time);
      vnp.push_back(np);
    }
    return true;
  }

  bool TrajectoryGenerator::generateNavigationParameter(const std::string &filename,
                                                        Trajectory3d &trajectory)
  {
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
    std::cout << "Error opening the file: " << filename;
    return false;
    }

    VectorOfNavigationParameter3d vnp;
    this->generate(vnp, trajectory);
    for (std::vector<NavigationParameter3d>::iterator it = vnp.begin();
         it != vnp.end();
         ++it){
      writeNavigationParameters(outfile, *it);
    }

    return true;
  };

  bool TrajectoryGenerator::generateG2o(const std::string &filename,
                                        Trajectory3d &trajectory){
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
    std::cout << "Error opening the file: " << filename;
    return false;
    }

    VectorOfNavigationParameter3d vnp;
    this->generate(vnp, trajectory);

    Pose3d pose;   
    int pose_id = 0;
    for (std::vector<NavigationParameter3d>::iterator it = vnp.begin();
         it != vnp.end();
         ++it)
    {
      np_to_pose(*it, pose);
      writeG2oVertexSE3(outfile, pose_id, pose);
      ++pose_id;
    }

    return true;                                      
  }

  bool TrajectoryGenerator::generatePose(const std::string &filename,
                                         Trajectory3d &trajectory)
  {
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
    std::cout << "Error opening the file: " << filename;
    return false;
    }

    VectorOfNavigationParameter3d vnp;
    this->generate(vnp, trajectory);

    Pose3d pose;   
    int pose_id = 0;
    for (std::vector<NavigationParameter3d>::iterator it = vnp.begin();
         it != vnp.end();
         ++it)
    {
      np_to_pose(*it, pose);
      writeCeresPose3d(outfile, pose_id, pose);
      ++pose_id;
    }
    return true;
  }


}   // namespace tg  
}   // namespace dh