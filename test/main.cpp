/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-05 20:34:22
 * @LastEditTime: 2021-08-09 19:33:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/main.cpp
 */

#include "core.hpp"
#include "unit.hpp"
#include "wgs84.hpp"
#include "sins.hpp"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>

void test()
{
    dh::core::hello();
    Eigen::Matrix<double, 4, 1> q;
    q << 0, 1, 0, 0;
    Eigen::Matrix<double, 3, 1> r;
    r << 1, 2, 3;
    std::cout << dh::core::quat_multiply_vec(q, r) << std::endl;
    Eigen::Matrix<double, 3, 3> mat;
    mat = dh::core::askew(r);
    std::cout << mat(1, 2) << std::endl;
    std::cout << dh::core::quat_add_rot(q, r) << std::endl;
    std::cout << dh::core::vee(mat) << std::endl;
    std::cout << dh::unit::DEGREE << std::endl;
    std::cout << dh::unit::ug << std::endl;
}

void test_earth_radii()
{
    double lat = 30 * dh::unit::DEGREE;
    double alt = 100;
    std::cout << dh::sins::earth_radii(lat) << std::endl;
}

void test_zxy_angle_2_quat()
{
    double rx = 10 * dh::unit::DEGREE;
    double ry = 20 * dh::unit::DEGREE;
    double rz = 30 * dh::unit::DEGREE;
    std::cout << rx << ry << rz << std::endl;
    std::cout << dh::core::zxy_angle_2_quat(rx, ry, rz) << std::endl;
}

void test_quat_multiply_quat(){
    Eigen::Matrix<double, 4, 1> ql;
    Eigen::Matrix<double, 4, 1> qr;
    ql << 1, 0, 1, 0;
    qr << 1, 0.5, 0.5, 0.75;
    std::cout<<dh::core::quat_multiply_quat(ql, qr) << std::endl;
}

void test_quat_multiply_vec(){
    Eigen::Matrix<double, 4, 1> q1;
    Eigen::Matrix<double, 4, 1> q2;
    Eigen::Matrix<double, 3, 1> vec;
    q1 << 1, 0, 1, 0;
    q2 << 1, 0.5, 0.3, 0.1;
    vec << 1, -2, 3;
    std::cout << dh::core::quat_multiply_vec(q1, vec) << std::endl;
    std::cout << dh::core::quat_multiply_vec(q2, vec) << std::endl;

    double rx = 10 * dh::unit::DEGREE;
    double ry = 20 * dh::unit::DEGREE;
    double rz = 30 * dh::unit::DEGREE;
    std::cout << rx << ry << rz << std::endl;
    q1 = dh::core::zxy_angle_2_quat(rx, ry, rz);
    std::cout << dh::core::quat_multiply_vec(q1, vec) << std::endl;
}

void test_quat_add_rot(){
    Eigen::Matrix<double, 3, 1> r1;
    Eigen::Matrix<double, 3, 1> r2;
    Eigen::Matrix<double, 4 ,1> q1;
    Eigen::Matrix<double, 4, 1> q2;

    r1 << 10*dh::unit::DEGREE, 20*dh::unit::DEGREE, 30*dh::unit::DEGREE;
    r2 << 5*dh::unit::DEGREE, 10*dh::unit::DEGREE, 256*dh::unit::DEGREE;

    q1 = dh::core::zxy_angle_2_quat(r1(0), r1(1), r1(2));
    std::cout<<q1<<std::endl;
    std::cout<<r2<<std::endl;
    std::cout<<dh::core::quat_add_rot(q1, r2) << std::endl;

}

void test_gravity_in_ENU(){
    double lat = 45*dh::unit::DEGREE;
    double alt = 100;
    std::cout << dh::sins::gravity_in_ENU(lat, alt) << std::endl;
}
int main(int argc, char **argv)
{
    test_gravity_in_ENU();
    return 0;
}