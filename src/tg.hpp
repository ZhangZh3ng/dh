/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-14 10:25:36
 * @LastEditTime: 2021-08-25 10:43:20
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

namespace dh
{
    namespace tg
    {
        class Trajectory3D
        {
        public:
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

            // Format0 =9
            int motion_list_col = 9;

            /**
             * a vector restoring all motion infomation, it format:
             * 1.index  2.start time 3.end time 4.wy 5.wp 6.wr 7.ax 8.ay 9.az
             * 10.index 11.start time ...
             * 
             */
            std::vector<double> motion_list;

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
             * @param pz initial z position, unit is m.
             */
            Trajectory3D(const double yaw, const double pitch, const double roll, const double vx, const double vy, const double vz, const double px, const double py, const double pz) : init_yaw(yaw), init_pitch(pitch), init_roll(roll), init_vx(vx), init_vy(vy), init_vz(vz), init_px(px), init_py(py), init_pz(pz) {}

            /**
             * @brief constructs and initializes the trajectory. 
             * 
             * @param ypr 
             * @param vxyz 
             * @param pxyz 
             */
            Trajectory3D(const Eigen::Vector3d ypr, const Eigen::Vector3d vxyz, const Eigen::Vector3d pxyz)
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
            bool add_motion(double lasting_time, double wy, double wp, double wr, double ax, double ay, double az)
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
        };

        class TrajectoryGenerator
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
            const int generate(const dh::tg::Trajectory3D &trajectory, std::vector<double> &data)
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
                this->write_line(data, time_stamp, q, pos, vn);

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
                        this->write_line(data, time_stamp, q, pos, vn);
                    }
                }
                return epoch;
            }

        private:
            bool write_line(std::vector<double> &data, double time_stamp, Eigen::Quaterniond q, Eigen::Vector3d p, Eigen::Vector3d vn)
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