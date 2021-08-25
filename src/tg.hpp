/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-14 10:25:36
 * @LastEditTime: 2021-08-25 20:55:02
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
             * @param pz initial z position, unit is m.
             */
            Trajectory3d(const double yaw, const double pitch, const double roll, const double vx, const double vy, const double vz, const double px, const double py, const double pz) : init_yaw_(yaw), init_pitch_(pitch), init_roll_(roll), init_vx_(vx), init_vy_(vy), init_vz_(vz), init_px_(px), init_py_(py), init_pz_(pz) {}

            /**
             * @brief constructs and initializes the trajectory. 
             * 
             * @param ypr 
             * @param vxyz 
             * @param pxyz 
             */
            Trajectory3d(const Eigen::Vector3d ypr, const Eigen::Vector3d vxyz, const Eigen::Vector3d pxyz)
            {
                this->init_yaw_ = ypr(0);
                this->init_pitch_ = ypr(1);
                this->init_roll_ = ypr(2);
                this->init_vx_ = vxyz(0);
                this->init_vy_ = vxyz(1);
                this->init_vz_ = vxyz(2);
                this->init_px_ = pxyz(0);
                this->init_py_ = pxyz(1);
                this->init_pz_ = pxyz(2);
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
                double end_time = this->total_time_ + lasting_time;
                this->motion_list.push_back(++this->motion_list_row_);
                this->motion_list.push_back(this->total_time_);
                this->motion_list.push_back(end_time);
                this->motion_list.push_back(wy);
                this->motion_list.push_back(wp);
                this->motion_list.push_back(wr);
                this->motion_list.push_back(ax);
                this->motion_list.push_back(ay);
                this->motion_list.push_back(az);
                this->total_time_ = end_time;

                return true;
            }
            // private: // why I can't access the private variable in Trajectory3dGenerator?
            // friend class dh::tg::Trajectory3dGenerator;

            // initial navigation parameters.
            double init_pitch_ = 0;
            double init_roll_ = 0;
            double init_yaw_ = 0;
            double init_vx_ = 0;
            double init_vy_ = 0;
            double init_vz_ = 0;
            double init_px_ = 0;
            double init_py_ = 0;
            double init_pz_ = 0;

            // the summation of all motion's lasting time.
            double total_time_ = 0;

            // motion number.
            int motion_list_row_ = 0;

            // Format0 =9
            int motion_list_col_ = 9;

            /**
             * a vector restoring all motion infomation, it format:
             * 1.index  2.start time 3.end time 4.wy 5.wp 6.wr 7.ax 8.ay 9.az
             * 10.index 11.start time ...
             * 
             */
            std::vector<double> motion_list;
        };

        class Trajectory3dGenerator
        {
        public:
            double step_time_ = 0.01;

            dh::type::EulerAngleType euler_angle_type_ = dh::type::EulerAngleType::ZXY;

            /**
             * @brief 
             * 
             * @param trajectory 
             * @param data a container to receive generated data.
             * @return const int 
             */
            const int generate(const dh::tg::Trajectory3d &trajectory, std::vector<double> &data)
            {
                // total epochs, including initial value.
                unsigned int epoch = 1;

                // velocity increment and euler angle increment.
                double dvx, dvy, dvz, dyaw, dpitch, droll, time_stamp = 0;

                // [yaw pitch roll]
                Eigen::Vector3d ypr;
                ypr << trajectory.init_yaw_, trajectory.init_pitch_, trajectory.init_roll_;
                // quaternion s.t. vn = q*vb
                Eigen::Quaterniond q = dh::geometry::ypr_to_quat(ypr, this->euler_angle_type_);

                Eigen::Vector3d pos;
                pos << trajectory.init_px_, trajectory.init_py_, trajectory.init_pz_;

                // velocity projection in body frame.
                Eigen::Vector3d vb;
                // velocity projection in navigtation(reference) frame.
                Eigen::Vector3d vn;
                // initial value of vn.
                Eigen::Vector3d vn0;
                vb << trajectory.init_vx_, trajectory.init_vy_, trajectory.init_vz_;
                vn = q * vb;
                vn0 = vn0;

                // write down initial navigation parameters.
                this->write_line(data, time_stamp, q, pos, vn);

                // using a shorter denotion
                const int col = trajectory.motion_list_col_;

                for (int k = 0; k < trajectory.motion_list_row_; k++)
                {
                    // time_stamp += this->step_time;
                    while (time_stamp >= trajectory.motion_list[k * col + 1] && time_stamp < trajectory.motion_list[k * col + 2])
                    {
                        // update time stamp.
                        time_stamp = this->step_time_ * epoch;
                        epoch++;

                        // read motion information.
                        dyaw = trajectory.motion_list[k * col + 3] * this->step_time_;
                        dpitch = trajectory.motion_list[k * col + 4] * this->step_time_;
                        droll = trajectory.motion_list[k * col + 5] * this->step_time_;
                        dvx = trajectory.motion_list[k * col + 6] * this->step_time_;
                        dvy = trajectory.motion_list[k * col + 7] * this->step_time_;
                        dvz = trajectory.motion_list[k * col + 8] * this->step_time_;

                        // because euler angle was used in Trajectory to indicate attitude change, so directly using euler angle update is more convenient.
                        ypr(0) += dyaw;
                        ypr(1) += dpitch;
                        ypr(2) += droll;
                        q = dh::geometry::ypr_to_quat(ypr, this->euler_angle_type_);

                        // update velocity in body frame.
                        vb(0) += dvx;
                        vb(1) += dvy;
                        vb(2) += dvz;

                        // new velocity in navigation frame;
                        vn = q * vb;

                        // new position
                        pos = (vn + vn0) / 2 * this->step_time_ + pos;

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