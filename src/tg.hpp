// /*
//  * @Author: Zhang Zheng
//  * @Date: 2021-08-14 10:25:36
//  * @LastEditTime: 2021-08-21 11:06:46
//  * @LastEditors: Please set LastEditors
//  * @Description: In User Settings Edit
//  * @FilePath: /dh/src/ts.hpp
//  */

// #ifndef DH_TS_H
// #define DH_TS_H

// #include <string>
// #include <iostream>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <math.h>
// #include "unit.hpp"
// #include "type.hpp"
// #include <vector>
// #include "geometry.hpp"

// namespace dh
// {
//     namespace tg // tg means trajectory generator.
//     {
//         class Motion3D
//         {
//         public:
//             double total_time = 0;

//             std::vector<double> motion_vector;
            
//             double init_pitch;
//             double init_roll;
//             double init_yaw;
//             double init_vx;
//             double init_vy;
//             double init_vz;
//             double init_px;
//             double init_py;
//             double init_pz;

//             Motion3D(const double pitch, const double roll, const double yaw, const double px, const double py, const double pz, const double vx, const double vy, const double vz) : init_pitch(pitch), init_roll(roll), init_yaw(yaw), init_vx(vx), init_vy(vy), init_vz(vz), init_px(px), init_py(py), init_pz(pz) {}

//             /**
//              * @brief 
//              * 
//              * @param wx 
//              * @param wy 
//              * @param wz 
//              * @param ax 
//              * @param ay 
//              * @param az 
//              * @param lasting_time 
//              */
//             void add_motion(const double wx, const double wy, const double wz, const double ax, const double ay, const double az, const double lasting_time)
//             {
//                 this->check();
//                 this->motion_vector.push_back(++this->motion_num); // col1
//                 this->motion_vector.push_back(-1);                 // col2
//                 this->motion_vector.push_back(lasting_time);       // col3
//                 this->motion_vector.push_back(wx);                 // col4
//                 this->motion_vector.push_back(wy);                 // col5
//                 this->motion_vector.push_back(wz);                 // col6
//                 this->motion_vector.push_back(ax);                 // col7
//                 this->motion_vector.push_back(ay);                 // col8
//                 this->motion_vector.push_back(az);                 // col9
//                 this->total_time += lasting_time;
//             }
            
//         };

//         /**
//          * @brief trajectory generator in 3d space, the reference frame is a cartisian frame.
//          * 
//          */
//         class TrajectoryGeneratorIn3DCartesianFrame
//         {
//         public:
//             // time length between two adjacent time point, default value is 0.01s i.e. sample rate = 100Hz
//             double time_step_length = 0.01;

//             // control whether velocity has projection in vehicle's left-right and vertical direction, default value is true, means NO projection in these directions.
//             bool only_has_forward_motion = true;

//             bool output_hamilton_quaternion = true;

//             // initial attitude quaternion(Hamiltion), default value is identity quaternion.
//             Eigen::Quaterniond init_q = Eigen::Quaterniond::Identity();

//             // initial velocity project in reference frame, default value is (0 0 0)
//             Eigen::Matrix<double, 3, 1> init_velocity = Eigen::Matrix<double, 3, 1>(0, 0, 0);

//             // initial position in reference 3d cartesian frame, default value is (0 0 0)
//             Eigen::Matrix<double, 3, 1> init_position = Eigen::Matrix<double, 3, 1>(0, 0, 0);

//             // total motion time
//             double total_time = 0;

//             double time_stamp = 0;

//             // total epoch
//             unsigned int epoch = 0;

//             //
//             dh::tg::Motion3D const &motion;

//             double px;
//             double py;
//             double pz;
//             double vx;
//             double vy;
//             double vz;
//             double pitch;
//             double roll;
//             double yaw;

//             std::vector<double> generate()
//             {
//                 std::vector<double> traj_info;

//                 double ax, ay, az, wx, wy, wz;
//                 Eigen::Quaterniond q = Eigen::Quaterniond(dh::core::euler_angle_2_quat_zxy(this->pitch, this->roll, this->yaw));

//                 Eigen::Matrix<double, 3, 1> vn;
//                 Eigen::Matrix<double, 3 ,1> vb;
//                 Eigen::Matrix<double, 3 ,1> p;

//                 vb << this->vx, this->vy, this->vz;

//                 vn = q*vb;

                
                

//                 while (this->time_stamp < this->motion.total_time)
//                 {
//                     this->epoch++;
//                     this->time_stamp = (this->epoch) * this->time_step_length;
//                 }

//                 return traj_info;
//             }

//         private:
//             void initialize()
//             {
//                 this->pitch = this->motion.init_pitch;
//                 this->roll = this->motion.init_roll;
//                 this->yaw = this->motion.init_yaw;
//                 this->vx = this->motion.init_vx;
//                 this->vy = this->motion.init_vy;
//                 this->vz = this->motion.init_vz;
//                 this->px = this->motion.init_px;
//                 this->py = this->motion.init_py;
//                 this->pz = this->motion.init_pz;
//             }

//             void update_one_step(const double wx, const double wy, const double wz, const double ax, const double ay, const double az){
                
//             }

//             void write_nav_info_to_vector(std::vector<double> &traj_info, double time_stamp, const Eigen::Quaterniond q, const Eigen::Matrix<double, 3, 1> v, const Eigen::Matrix<double, 3, 1> p)
//             {
//                 traj_info.push_back(time_stamp);
//                 traj_info.push_back(q.w());
//                 traj_info.push_back(q.x());
//                 traj_info.push_back(q.y());
//                 traj_info.push_back(q.z());
//                 traj_info.push_back(v(0));
//                 traj_info.push_back(v(1));
//                 traj_info.push_back(v(2));
//                 traj_info.push_back(p(0));
//                 traj_info.push_back(p(1));
//                 traj_info.push_back(p(2));
//             }
//         };

//     }
// }

// #endif