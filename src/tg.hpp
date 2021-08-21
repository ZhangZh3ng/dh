/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-14 10:25:36
 * @LastEditTime: 2021-08-21 17:30:25
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

namespace dh
{
    namespace tg // tg means trajectory generator.
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

            //
            int reference_frame = 0;

            double total_time = 0;

            // format of motion list.
            dh::type::TrajectoryFormat motion_list_format = dh::type::TrajectoryFormat::FORMAT_0;

            // motion number.
            int motion_list_row = 0;

            // Format0 =9
            int motion_list_col = 9;

            /**
             *  a vector to stroe all motion infomation, it can be save in different format.
             * Format0:
             * 1.index  2.start time 3.end time 4.wy 5.wp 6.wr 7.ax 8.ay 9.az
             * 
             */
            std::vector<double> motion_list;

            /**
             * @brief Construct a new Trajectory 3 D object
             * 
             * @param pitch initial pitch angle, unit is radian, so as to roll and yaw.
             * @param roll 
             * @param yaw 
             * @param vx velocity w.r.t body x axis, unnit is m/s.
             * @param vy 
             * @param vz 
             * @param px 
             * @param py 
             * @param pz 
             */
            Trajectory3D(const double pitch, const double roll, const double yaw, const double vx, const double vy, const double vz, const double px, const double py, const double pz) : init_pitch(pitch), init_roll(roll), init_yaw(yaw), init_vx(vx), init_vy(vy), init_vz(vz), init_px(px), init_py(py), init_pz(pz) {}

            /**
             * @brief Construct a new Trajectory 3 D object, we will set the initial navigation parameters while constructing object.
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
             * @brief add a motion to this trajectory
             * 
             * @param lasting_time unit is s
             * @param wy yaw rate, unit is rad/s.
             * @param wp pitch rate
             * @param wr roll rate
             * @param ax acceleration in direction of BODY x axis, unit is m/s^2
             * @param ay 
             * @param az 
             * @return true 
             * @return false 
             */
            bool add_motion(double lasting_time, double wy, double wp, double wr, double ax, double ay, double az)
            {
                switch (this->motion_list_format)
                {
                case 0:
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
                    break;
                default:
                    break;
                }
                return true;
            }
        };

        class TrajectoryGenerator
        {
            // trajectory.
            const dh::tg::Trajectory3D & trajectory;

            int output_format = 0;

            double step_time = 0.01;

            std::vector<double> &generate()
            {

                std::vector<double> data;
                unsigned int epoch = 0;
                double dvx, dvy, dvz, dwy, dwp, dwr, time_stamp = 0;

                Eigen::Vector3d ypr;
                ypr << this->trajectory.init_yaw, this->trajectory.init_pitch, this->trajectory.init_roll;

                Eigen::Quaterniond q = dh::geometry::ypr_to_quat(ypr);

                Eigen::Vector3d pos;
                pos << this->trajectory.init_px, this->trajectory.init_py, this->trajectory.init_pz;

                Eigen::Vector3d vb, vn, vb0, vn0;
                vb << this->trajectory.init_vx, this->trajectory.init_vy, this->trajectory.init_vz;
                vn = q * vb;
                vb0 = vb;
                vn0 = vn0;

                // white initial navigation parameters.
                this->write_line(data, time_stamp, q, pos, vn);

                int col = this->trajectory.motion_list_col;

                for (int k = 0; k < this->trajectory.motion_list_row; k++)
                {
                    time_stamp += this->step_time;
                    while (time_stamp >= this->trajectory.motion_list[k * col + 1] && time_stamp < this->trajectory.motion_list[k * col + 2])
                    {
                        // read motion information.
                        dwy = this->trajectory.motion_list[k * col + 3] * this->step_time;
                        dwp = this->trajectory.motion_list[k * col + 4] * this->step_time;
                        dwr = this->trajectory.motion_list[k * col + 5] * this->step_time;
                        dvx = this->trajectory.motion_list[k * col + 6] * this->step_time;
                        dvy = this->trajectory.motion_list[k * col + 7] * this->step_time;
                        dvz = this->trajectory.motion_list[k * col + 8] * this->step_time;

                        // because euler angle was used in Trajectory to indicate attitude change, so directly using euler angle update is more convenient.
                        ypr(0) += dwy;
                        ypr(1) += dwp;
                        ypr(2) += dwr;
                        q = dh::geometry::ypr_to_quat(ypr);

                        // update velocity in body frame.
                        vb(0) += dvx;
                        vb(1) += dvy;
                        vb(2) += dvz;

                        // new vn;
                        vn = q * vb;

                        // new pos
                        pos = (vn + vn0) / 2 * this->step_time + pos;

                        // update velocity in last loop.
                        vb0 = vb;
                        vn0 = vn;

                        // record data.
                        this->write_line(data, time_stamp, q, pos, vn);

                        // update time stamp.
                        time_stamp += this->step_time;
                    }
                }
                return data;
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