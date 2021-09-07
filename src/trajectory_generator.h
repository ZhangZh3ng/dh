/*
 * @Author: your name
 * @Date: 2021-09-01 19:53:13
 * @LastEditTime: 2021-09-07 20:46:05
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
        friend std::ostream &operator<<(std::ostream &os, Motion3d & motion);
        friend class Trajectory3d;

        Motion3d(const double val_begin_time, const double val_end_time,
                 const double val_wy, const double val_wp, const double val_wr,
                 const double val_ax, const double val_ay, const double val_az)
            : begin_time(val_begin_time), end_time(val_end_time),
              wy(val_wy), wp(val_wp), wr(val_wr),
              ax(val_ax), ay(val_ay), az(val_az) {}

        static int size() { return 8; }
        double getEndTime() const {return this->end_time; }

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

    inline std::ostream &operator<<(std::ostream &os, Motion3d & motion){
        os << motion.begin_time << "\t" << motion.end_time  << "\t"
        << motion.wy << "\t" << motion.wp << "\t" << motion.wr << "\t"
        << motion.ax << "\t" << motion.ay << "\t" << motion.az;
        return os;
    }

    class Trajectory3d{
    public:
        friend class TrajectoryGenerator;

        Trajectory3d(NavigationParameter3d np){
            this->initial_parameter = np;
        }

        void addMotion(const Motion3d& motion){
            this->motions.push_back(motion);
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
                                             const double time_stamp);

        void briefReport();
        
    private:
        std::vector<Motion3d> motions;
        int num_motions = 0;
        double total_time = 0;
        NavigationParameter3d initial_parameter;
    };



    class TrajectoryGenerator{
    public:
        TrajectoryGenerator(){};

        bool generate(VectorOfNavigationParameter3d &vnp,
                      Trajectory3d &Trajectory3d);

        bool generate(const std::string &filename,
                      Trajectory3d &trajectory){
            switch (this->data_format){
            case G2O_VERTEX_SE3:
                return this->generateG2o(filename, trajectory);
            case DH_NavigationParameter3d:
                return this->generateNavigationParameter(filename, trajectory);
            case CERES_Pose3d:
                return this->generatePose(filename, trajectory);
            default:
                std::cout << "Unsupported data format" << std::endl;
                return false;
            }
        }

        double step_time = 0.01;
        EulerAngleType euler_angle_type = ZXY;
        DataFormat data_format = DH_NavigationParameter3d;

    private:
        // output NavigationParameter format :
        // 1.time_stamp 2.yaw 3.pitch 4.roll 5.vx 6.vy 7.vz 8.px 9.py 10.pz
        bool generateNavigationParameter(const std::string &filename,
                                         Trajectory3d &trajectory);
                                         
        // output g2o VERTEX_SE3 format:
        // VERTEX_SE3:QUAT pose_id px py pz qx qy qz qw
        bool generateG2o(const std::string &filename,
                         Trajectory3d &trajectory);

        // output ceres Pose3d format:
        // pose_id px py pz qx qy qz qw
        bool generatePose(const std::string &filename,
                          Trajectory3d &trajectory);
    };
    
}   // namespace tg   
}   // namespace dh

# endif