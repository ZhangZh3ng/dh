/*
 * @Author: your name
 * @Date: 2021-09-01 19:53:13
 * @LastEditTime: 2021-09-10 11:02:22
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/trajectory_generator.h
 */

#ifndef DH_TRAJECTORY_GENERATOR_H
#define DH_TRAJECTORY_GENERATOR_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "type.h"
#include "parameter.h"

using namespace dh::type;
using namespace dh::parameter;

namespace dh{
namespace tg{
  
  struct Motion3d
  {
  public:
      friend std::ostream &operator<<(std::ostream &os, Motion3d &motion);
      friend class Trajectory3d;

      Motion3d(const double val_begin_time, const double val_end_time,
               const double val_wy, const double val_wp, const double val_wr,
               const double val_ax, const double val_ay, const double val_az)
          : begin_time(val_begin_time), end_time(val_end_time),
            wy(val_wy), wp(val_wp), wr(val_wr),
            ax(val_ax), ay(val_ay), az(val_az) {}

      static int size() { return 8; }
      double getEndTime() const { return this->end_time; }

  private:
      double begin_time; // begin time of this motion
      double end_time;   // end time of this motion
      double wy;         // yaw velocity, unit is rad/s
      double wp;         // pitch velocity, unit is rad/s
      double wr;         // roll velocity, unit is rad/s
      double ax;         // acceleration in body-x axis, unit is m/s^2
      double ay;         // acceleration in body-y axis, unit is m/s^2
      double az;         // acceleration in body-z axis, unit is m/s^2
  };

  inline std::ostream &operator<<(std::ostream &os, Motion3d &motion)
  {
      os << motion.begin_time << "\t" << motion.end_time << "\t"
         << motion.wy << "\t" << motion.wp << "\t" << motion.wr << "\t"
         << motion.ax << "\t" << motion.ay << "\t" << motion.az;
      return os;
  }

  class Trajectory3d
  {
  public:
      friend class TrajectoryGenerator;

      template <class NpType>
      friend bool generateTrajectory(std::vector<NpType> &vnp, const NpType &initial_np, const Trajectory3d &trajectory, const double& dt);

      // Trajectory3d(NavigationParameter3d np){
      //     this->initial_parameter = np;
      // }

      void addMotion(const Motion3d &motion)
      {
          this->motions_ptr->push_back(motion);
          this->num_motions += 1;
          if (motion.getEndTime() > this->total_time)
              this->total_time = motion.getEndTime();
      }

      void addMotion(const double lasting_time, const double wy, const double wp,
                     const double wr, const double ax, const double ay, const double az)
      {
          this->addMotion(Motion3d(this->total_time,
                                   this->total_time + lasting_time,
                                   wy, wp, wr, ax, ay, az));
      }

      bool getAngleVelocityAndAcceleration(Eigen::Vector3d &w,
                                           Eigen::Vector3d &a,
                                           const double time_stamp) const ;

      void briefReport();

  private:
      std::shared_ptr<std::vector<Motion3d>> motions_ptr;
      int num_motions = 0;
      double total_time = 0;
      // NavigationParameter3d initial_parameter;
  };

  template <class NpType>
  bool generateTrajectory(std::vector<NpType> &vnp,
                          const NpType &initial_np,
                          const Trajectory3d &trajectory,
                          const double& dt)
  {
    NpType np = initial_np;
    vnp.clear();
    Eigen::Vector3d w, a;
    vnp.push_back(np);

    while(np.time_stamp < trajectory.total_time){
      trajectory.getAngleVelocityAndAcceleration(w, a, np.time_stamp);
      np.update(w, a, dt);
      vnp.push_back(np);
    }
    return true;
  }

  class TrajectoryGenerator
  {
  public:
    TrajectoryGenerator(){};

    template <class NpType>
    bool generate(std::vector<NpType> &vnp,
                  NpType &np,
                  Trajectory3d &trajectory)
    {
        vnp.clear();
        Eigen::Vector3d w, a;

        np.euler_angle_type = this->euler_angle_type;
        vnp.push_back(np);

        while (np.time_stamp < trajectory.total_time)
        {
            trajectory.getAngleVelocityAndAcceleration(w, a, np.time_stamp);
            np.update(w, a, this->step_time);
            vnp.push_back(np);
        }
        return true;
    }

    template<class NpType>
    void output(const std::string &filename,
                Trajectory3d &trajectory)
    {
        switch (this->output_data_format)
        {
        case DataFormat::G2O_VERTEX_SE3:
            
        case DataFormat::XYZ_NavigationParameter:

        case DataFormat::CERES_Pose3d:

        default:
            std::cout << "Unsupported data format" << std::endl;
        }
    }

    double step_time = 0.01;
    EulerAngleType euler_angle_type = EulerAngleType::ZXY;
    DataFormat output_data_format = DataFormat::XYZ_NavigationParameter;

  private:
    // // output NavigationParameter format :
    // // 1.time_stamp 2.yaw 3.pitch 4.roll 5.vx 6.vy 7.vz 8.px 9.py 10.pz
    // bool generateNavigationParameter(const std::string &filename,
    //                                  Trajectory3d &trajectory);

    // // output g2o VERTEX_SE3 format:
    // // VERTEX_SE3:QUAT pose_id px py pz qx qy qz qw
    // bool generateG2o(const std::string &filename,
    //                  Trajectory3d &trajectory);

    // // output ceres Pose3d format:
    // // pose_id px py pz qx qy qz qw
    // bool generatePose(const std::string &filename,
    //                   Trajectory3d &trajectory);
  };

 

}   // namespace tg   
}   // namespace dh

# endif