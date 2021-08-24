/*
 * @Author: your name
 * @Date: 2021-08-21 09:23:17
 * @LastEditTime: 2021-08-24 14:35:08
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/main2.cpp
 */

#include "core.hpp"
#include "unit.hpp"
#include "wgs84.hpp"
#include "sins.hpp"
#include "tg.hpp"
#include "read.hpp"
#include "geometry.hpp"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <string>
#include <vector>
#include "utils.hpp"

void test_zxy_to_quat()
{
    Eigen::Vector3d v;
    v << 1 * dh::unit::degree, 2 * dh::unit::degree, 30 * dh::unit::degree;
    Eigen::Quaterniond q = dh::geometry::zxy_euler_angle_to_quat(v);
    std::cout << q.coeffs() << std::endl;

    Eigen::Matrix<double, 3, 3> dcm = q.toRotationMatrix();
    std::cout << dcm << std::endl;
    std::cout << dcm(2, 0) << std::endl;

    Eigen::Vector3d rpy = dh::geometry::quat_to_ypr(q) / dh::unit::degree;
    std::cout << rpy << std::endl;
}

void test_utils()
{
    Eigen::Vector3d v;
    v << 1, 2, 3;
    std::cout << dh::utils::askew(v) << std::endl;
}

void test_eigen_quat()
{
    Eigen::Vector3d ypr;
    ypr<< 90 * dh::unit::degree, 0, 0;
    Eigen::Quaterniond q = dh::geometry::ypr_to_quat(ypr, dh::type::ZXY);
    Eigen::Vector3d v;
    v<<0,1,0;
    std::cout << q.coeffs() << std::endl;
    std::cout << q*v << std::endl;
}

void test_tg()
{
    double yaw = 90 * dh::unit::degree;
    double pitch = 0 * dh::unit::degree;
    double roll = 0 * dh::unit::degree;
    dh::tg::Trajectory3D mt(yaw, pitch, roll, 0, 0, 0, 0, 0, 0);
    mt.add_motion(10, 0, 0, 0, 0, 5, 0);
    mt.add_motion(90, 0, 0, 0, 0, 0, 0);
    int k = 0;
    std::cout.precision(5);
    std::cout.width(6);
    for (std::vector<double>::iterator it = mt.motion_list.begin(); it != mt.motion_list.end(); it++)
    {
        std::cout << *it;
        if (++k == 9)
        {
            std::cout << std::endl;
            k = 0;
        }
        else
            std::cout << " ";
    }
    dh::tg::TrajectoryGenerator gen;
    gen.step_time = 0.1;
    std::vector<double> data;
    gen.generate(mt, data);
    k = 0;
    for (std::vector<double>::iterator itg = data.begin(); itg != data.end(); itg++)
    {
        std::cout << *itg;
        if (++k == 11)
        {
            std::cout << std::endl;
            k = 0;
        }
        else
            std::cout << " ";
    }
}

int main(int argc, char **argv)
{
    // std::string path = argv[1];
    test_eigen_quat();

    return 0;
}