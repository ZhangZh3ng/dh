/*
 * @Author: your name
 * @Date: 2021-09-01 19:53:13
 * @LastEditTime: 2021-09-01 21:19:37
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

namespace dh{
    
    struct Motion3d
    {
    public:
        friend std::ostream &operator<<(std::ostream &os, Motion3d & motion);

        Motion3d(const double val_begin_time, const double val_end_time,
                 const double val_wy, const double val_wp, const double val_wr,
                 const double val_ax, const double val_ay, const double val_az)
                  : begin_time(val_begin_time), end_time(val_end_time),
                    wy(val_wy), wp(val_wp), wr(val_wr),
                    ax(val_ax), ay(val_ay), az(val_az) {}

        static std::string name() { return "Motion3d"; }
        static int size() { return 8; }

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
        os << motion.begin_time << motion.end_time
        << motion.wy << motion.wp << motion.wr
        << motion.ax << motion.ay << motion.az;
        return os;
    }

    class Trajectory3d{
    public:
        friend class TrajectoryGenerator3d;

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

        void addMotion(const Motion3d& motion);
        
    private:
        std::vector<dh::Motion3d> motion;
        int num_motions;
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

    class TrajectoryGenerator3d{
    public:
        dh::Trajectory3d& trajectory;
        bool generate(const std::string& filename);

    private:
        
    };
}   // namespace dh



# endif