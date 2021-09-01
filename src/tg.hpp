/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-14 10:25:36
 * @LastEditTime: 2021-08-31 21:28:20
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/ts.hpp
 */

#ifndef DH_TS_H
#define DH_TS_H

#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include "unit.hpp"
#include "type.hpp"
#include <vector>
#include "geometry.hpp"
#include <memory>
// #include "ceres/ceres.h"
// #include "ceres/types.h"
// #include "pose_graph_3d/types.h"

namespace dh{
namespace tg{

    class Motion3d
    {
    public:
        friend std::ostream &operator<<(std::ostream &output, dh::tg::Motion3d & motion);

        Motion3d(const double val_begin_time, const double val_end_time, const double val_wy, const double val_wp, const double val_wr, const double val_ax, const double val_ay, const double val_az) : begin_time(val_begin_time), end_time(val_end_time), wy(val_wy), wp(val_wp), wr(val_wr), ax(val_ax), ay(val_ay), az(val_az) {}

        static std::string name() { return "Motion3d"; }

    public:
        double begin_time; // begin time of this motion
        double end_time;   // end time of this motion
        double wy;         // yaw velocity, unit is rad/s
        double wp;         // pitch velocity, unit is rad/s
        double wr;         // roll velocity, unit is rad/s
        double ax;         // acceleration in body-x axis direction, unit is m/s^2
        double ay;         // acceleration in body-y axis direction, unit is m/s^2
        double az;         // acceleration in body-z axis direction, unit is m/s^2
    };
    
    inline std::ostream &operator<<(std::ostream &output, const dh::tg::Motion3d &motion)
    {
        output<<motion.begin_time<<motion.end_time<<motion.wy<<motion.wp<<motion.wr<< motion.ax<<motion.ay<<motion.az;
        return output;
    }

    class Trajectory3d
    {
    public:
        /**
        * @brief constructs and initializes the trajectory. 
        * 
        * @param yaw initial yaw angle, unit is radian
        * @param pitch initial pitch
        * @param roll initial roll
        * @param vx initial velocity w.r.t body x axis, unit is m/s.
        * @param vy initial velocity w.r.t body y axis, unit is m/s.
        * @param vz initial velocity w.r.t body z axis, unit is m/s.
        * @param px initial x position, unit is m.
        * @param py initial y position, unit is m.
         @param pz initial z position, unit is m.
        */
        Trajectory3d(const double val_yaw, const double val_pitch, const double val_roll, const double val_vx, const double val_vy, const double val_vz, const double val_px, const double val_py, const double val_pz) : init_yaw(val_yaw), init_pitch(val_pitch), init_roll(val_roll), init_vx(val_vx), init_vy(val_vy), init_vz(val_vz), init_px(val_px), init_py(val_py), init_pz(val_pz) {}

        /**
        * @brief constructs and initializes the trajectory. 
        * 
        * @param ypr 
        * @param vxyz 
        * @param pxyz 
       */
        Trajectory3d(const Eigen::Vector3d ypr, const Eigen::Vector3d vxyz, const Eigen::Vector3d pxyz)
        {
            this->init_yaw = ypr(0);
            this->init_pitch = ypr(1);
            this->init_roll = ypr(2);
            this->init_vx = vxyz(0);
            this->init_vy = vxyz(1);
            this->init_vz = vxyz(2);
            this->init_px = pxyz(0);
            this->init_py = pxyz(1);
            this->init_pz = pxyz(2);
        }

        /**
         * @brief add a motion using client given parameters.
         * 
         * @param lasting_time unit is s
         * @param wy yaw rate, unit is rad/s.
         * @param wp pitch rate
         * @param wr roll rate
         * @param ax acceleration in direction of BODY x axis, unit is m/s^2
         * @param ay acceleration in direction of BODY y axis
         * @param az acceleration in direction of BODY z axis
         * @return true 
         * @return false 
        */
        bool addMotion(double lasting_time, double wy, double wp, double wr, double ax, double ay, double az)
        {
            double end_time = this->total_time + lasting_time;
            this->motion_list.push_back(++this->motion_list_row);
            this->motion_list.push_back(this->total_time);
            this->motion_list.push_back(end_time);
            this->motion_list.push_back(wy);
            this->motion_list.push_back(wp);
            this->motion_list.push_back(wr);
            this->motion_list.push_back(ax);
            this->motion_list.push_back(ay);
            this->motion_list.push_back(az);
            this->total_time = end_time;

            return true;
        }
        // private: // why I can't access the private variable in Trajectory3dGenerator?
        // friend class dh::tg::Trajectory3dGenerator;

        // initial navigation parameters.
        double init_pitch = 0;
        double init_roll = 0;
        double init_yaw = 0;
        double init_vx = 0;
        double init_vy = 0;
        double init_vz = 0;
        double init_px = 0;
        double init_py = 0;
        double init_pz = 0;

        // the summation of all motion's lasting time.
        double total_time = 0;

        // motion number.
        int motion_list_row = 0;

        //
        int motion_list_col = 9;

        /**
         * a vector restoring all motion infomation, it format:
         * 1.index  2.start time 3.end time 4.wy 5.wp 6.wr 7.ax 8.ay 9.az
         * 10.index 11.start time ...
         * 
         */
        std::vector<double> motion_list;
    };

    class NavigationParameter3d
    {
    public:
        Eigen::Quaterniond q;
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        double time_stamp;
        NavigationParameter3d(const Eigen::Quaterniond val_q, const Eigen::Vector3d val_p, const Eigen::Vector3d val_v, const double val_time_stamp) : q(val_q), p(val_p), v(val_v), time_stamp(val_time_stamp) {}
    };

    class TrajectoryGenerator3d
    {
    public:
        double step_time = 0.01;
        dh::type::EulerAngleType euler_angle_type = dh::type::EulerAngleType::ZXY;
        /**
             * @brief 
             * 
             * @param trajectory 
             * @param data a container to receive generated data.
             * @return const int 
             */
        int generate(const dh::tg::Trajectory3d &trajectory, std::vector<double> &data) const
        {
            // total epochs, including initial value.
            unsigned int epoch = 1;

            // velocity increment and euler angle increment.
            double dvx, dvy, dvz, dyaw, dpitch, droll, time_stamp = 0;

            // [yaw pitch roll]
            Eigen::Vector3d ypr;
            ypr << trajectory.init_yaw, trajectory.init_pitch, trajectory.init_roll;
            // quaternion s.t. vn = q*vb
            Eigen::Quaterniond q = dh::geometry::ypr_to_quat(ypr, this->euler_angle_type);

            Eigen::Vector3d pos;
            pos << trajectory.init_px, trajectory.init_py, trajectory.init_pz;

            // velocity projection in body frame.
            Eigen::Vector3d vb;
            // velocity projection in navigtation(reference) frame.
            Eigen::Vector3d vn;
            // initial value of vn.
            Eigen::Vector3d vn0;
            vb << trajectory.init_vx, trajectory.init_vy, trajectory.init_vz;
            vn = q * vb;
            vn0 = vn0;

            // write down initial navigation parameters.
            this->writeLine(data, time_stamp, q, pos, vn);

            // using a shorter denotion
            const int col = trajectory.motion_list_col;

            for (int k = 0; k < trajectory.motion_list_row; k++)
            {
                // time_stamp += this->step_time;
                while (time_stamp >= trajectory.motion_list[k * col + 1] && time_stamp < trajectory.motion_list[k * col + 2])
                {
                    // update time stamp.
                    time_stamp = this->step_time * epoch;
                    epoch++;

                    // read motion information.
                    dyaw = trajectory.motion_list[k * col + 3] * this->step_time;
                    dpitch = trajectory.motion_list[k * col + 4] * this->step_time;
                    droll = trajectory.motion_list[k * col + 5] * this->step_time;
                    dvx = trajectory.motion_list[k * col + 6] * this->step_time;
                    dvy = trajectory.motion_list[k * col + 7] * this->step_time;
                    dvz = trajectory.motion_list[k * col + 8] * this->step_time;

                    // because euler angle was used in Trajectory to indicate attitude change, so directly using euler angle update is more convenient.
                    ypr(0) += dyaw;
                    ypr(1) += dpitch;
                    ypr(2) += droll;
                    q = dh::geometry::ypr_to_quat(ypr, this->euler_angle_type);

                    // update velocity in body frame.
                    vb(0) += dvx;
                    vb(1) += dvy;
                    vb(2) += dvz;

                    // new velocity in navigation frame;
                    vn = q * vb;

                    // new position
                    pos = (vn + vn0) / 2 * this->step_time + pos;

                    // update initial value in last loop.
                    vn0 = vn;

                    // record data.
                    this->writeLine(data, time_stamp, q, pos, vn);
                }
            }
            return epoch;
        }

    private:
        bool writeLine(std::vector<double> &data, double time_stamp, Eigen::Quaterniond q, Eigen::Vector3d p, Eigen::Vector3d vn) const
        {
            data.push_back(time_stamp);
            data.push_back(q.w());
            data.push_back(q.x());
            data.push_back(q.y());
            data.push_back(q.z());
            data.push_back(p(0));
            data.push_back(p(1));
            data.push_back(p(2));
            data.push_back(vn(0));
            data.push_back(vn(1));
            data.push_back(vn(2));
            return true;
        }
    };
}
}

#endif