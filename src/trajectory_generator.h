/*
 * @Author: your name
 * @Date: 2021-09-01 19:53:13
 * @LastEditTime: 2021-09-04 14:07:33
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/trajectory_generator.h
 */

#ifndef DH_TRAJECTORY_GENERATOR_H
#define DH_TRAJECTORY_GENERATOR_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "type.h"
#include "geometry.h"


using namespace dh::type;
using namespace dh::geometry;

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

        static std::string name() { return "Motion3d"; }
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

    inline std::ostream &operator<<(std::ostream &os, Motion3d & motion) {
        os << motion.begin_time << "\t" << motion.end_time  << "\t"
        << motion.wy << "\t" << motion.wp << "\t" << motion.wr << "\t"
        << motion.ax << "\t" << motion.ay << "\t" << motion.az;
        return os;
    }

    struct NavigationParameter3d{
    public:
        NavigationParameter3d(const double v_yaw, const double v_pitch,const double v_roll,
                              const double v_vx, const double v_vy, const double v_vz,
                              const double v_px, const double v_py, const double v_pz,
                              const EulerAngleType euler_type = ZXY)
                              : yaw(v_yaw), pitch(v_pitch), roll(v_roll),
                                vx(v_vx), vy(v_vy), vz(v_vz), 
                                px(v_px), py(v_py), pz(v_pz),
                                euler_angle_type(euler_type) {}
        
        void update(const double wy, const double wp, const double wr,
                    const double ax, const double ay, const double az,
                    const double t);

        void update(const Eigen::Vector3d w, 
                    const Eigen::Vector3d a, 
                    const double t);

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

    class Trajectory3d{
    public:
        friend class TrajectoryGenerator;

        Trajectory3d(const double v_init_yaw, const double v_init_pitch,
                     const double v_init_roll, const double v_init_vx,
                     const double v_init_vy, const double v_init_vz,
                     const double v_init_px, const double v_init_py,
                     const double v_init_pz)
            : init_yaw(v_init_yaw), init_pitch(v_init_pitch),
              init_roll(v_init_roll), init_vx(v_init_vx),
              init_vy(v_init_vy), init_vz(v_init_vz),
              init_px(v_init_px), init_py(v_init_py),
              init_pz(v_init_pz) {}

        Trajectory3d(const Eigen::Vector3d& ypr, const Eigen::Vector3d & vxyz,
                        const Eigen::Vector3d& pxyz) 
                        : init_yaw(ypr(0)), init_pitch(ypr(1)), init_roll(ypr(2)),
                        init_vx(vxyz(0)), init_vy(vxyz(1)), init_vz(vxyz(2)),
                        init_px(pxyz(0)), init_py(pxyz(1)), init_pz(pxyz(2)) {}

        static std::string name() { return "Trajectory3d"; }

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
        int num_motions;
        double total_time = 0;
        NavigationParameter3d initial_navigation_parameter;
        double init_yaw;
        double init_pitch;
        double init_roll;
        double init_vx;
        double init_vy;
        double init_vz;
        double init_px;
        double init_py;
        double init_pz;
    };

    class TrajectoryGenerator{
    public:
        TrajectoryGenerator(){};
        bool generate(const std::string& filename,
                      Trajectory3d& trajectroy);
                
        double step_time = 0.01;
        EulerAngleType euler_angle_type = ZXY;
    };
    
}   // namespace tg   
}   // namespace dh

# endif