/*
 * @Author: your name
 * @Date: 2021-09-01 19:53:13
 * @LastEditTime: 2021-09-17 11:00:54
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
#include "navigation_parameter.h"
#include "imu_parameter.h"
#include "utils.h"

namespace dh{
  
  struct Motion3d
  {
  public:
    friend std::ostream &operator<<(std::ostream &os, const Motion3d &motion);
    friend class Trajectory3d;

    Motion3d(const double _begin_time, const double _end_time,
             const double _wy, const double _wp, const double _wr,
             const double _ax, const double _ay, const double _az)
        : begin_time(_begin_time), end_time(_end_time),
          wy(_wy), wp(_wp), wr(_wr),
          ax(_ax), ay(_ay), az(_az) {}

    static int size() { return 8; }
    double getEndTime() const { return this->end_time; }

    double begin_time; // begin time of this motion
    double end_time;   // end time of this motion
    double wy;         // yaw velocity, unit is rad/s
    double wp;         // pitch velocity, unit is rad/s
    double wr;         // roll velocity, unit is rad/s
    double ax;         // acceleration in body-x axis, unit is m/s^2
    double ay;         // acceleration in body-y axis, unit is m/s^2
    double az;         // acceleration in body-z axis, unit is m/s^2
  };

  inline std::ostream &operator<<(std::ostream &os, const Motion3d &motion)
  {
    os << motion.begin_time << "\t" << motion.end_time << "\t"
       << motion.wy << "\t" << motion.wp << "\t" << motion.wr << "\t"
       << motion.ax << "\t" << motion.ay << "\t" << motion.az;
    return os;
  }

  class Trajectory3d
  {
  public:
    template <class NpType>
    friend bool generateTrajectory(std::vector<NpType> &vnp, const NpType &initial_np, const Trajectory3d &trajectory, const double &dt);

    void addMotion(const Motion3d &motion)
    {
      this->motions.push_back(motion);
      this->num_motions += 1;
      if (motion.end_time > this->total_time)
        this->total_time = motion.end_time;
    }

    void addMotion(const double &lasting_time, const double &wy, const double &wp,
                   const double &wr, const double &ax, const double &ay, const double &az)
    {
      this->addMotion(Motion3d(this->total_time,
                               this->total_time + lasting_time,
                               wy, wp, wr, ax, ay, az));
    }

    bool getAngleVelocityAndAcceleration(Eigen::Vector3d &w,
                                         Eigen::Vector3d &a,
                                         const double time_stamp) const;

    void briefReport() const;

    std::vector<Motion3d> motions;
    int num_motions = 0;
    double total_time = 0;
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

    while(!DefinitelyGreater(np.time_stamp, trajectory.total_time)){
      trajectory.getAngleVelocityAndAcceleration(w, a, np.time_stamp);
      np.propogate(w, a, dt);
      vnp.push_back(np);
    }
    return true;
  }

}   // namespace dh

# endif