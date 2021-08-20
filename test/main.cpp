/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-05 20:34:22
 * @LastEditTime: 2021-08-20 20:15:34
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/main.cpp
 */

#include "core.hpp"
#include "unit.hpp"
#include "wgs84.hpp"
#include "sins.hpp"
#include "tg.hpp"
#include "read.hpp"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <string>
#include <vector>

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

void test_quat_multiply_quat()
{
    Eigen::Matrix<double, 4, 1> ql;
    Eigen::Matrix<double, 4, 1> qr;
    ql << 1, 0, 1, 0;
    qr << 1, 0.5, 0.5, 0.75;
    std::cout << dh::core::quat_multiply_quat(ql, qr) << std::endl;
}

void test_quat_multiply_vec()
{
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

void test_quat_add_rot()
{
    Eigen::Matrix<double, 3, 1> r1;
    Eigen::Matrix<double, 3, 1> r2;
    Eigen::Matrix<double, 4, 1> q1;
    Eigen::Matrix<double, 4, 1> q2;

    r1 << 10 * dh::unit::DEGREE, 20 * dh::unit::DEGREE, 30 * dh::unit::DEGREE;
    r2 << 5 * dh::unit::DEGREE, 10 * dh::unit::DEGREE, 256 * dh::unit::DEGREE;

    q1 = dh::core::zxy_angle_2_quat(r1(0), r1(1), r1(2));
    std::cout << q1 << std::endl;
    std::cout << r2 << std::endl;
    std::cout << dh::core::quat_add_rot(q1, r2) << std::endl;
}

void test_gravity_in_ENU()
{
    double lat = 45 * dh::unit::DEGREE;
    double alt = 100;
    std::cout << dh::sins::gravity_in_enu(lat, alt) << std::endl;
}

void test_win_in_enu()
{
    double lat = 45 * dh::unit::DEGREE;
    double alt = 100;
    double ve = 10;
    double vn = 20;
    std::cout << dh::sins::win_in_enu(lat, alt, ve, vn) << std::endl;
}

void test_sins_Mpv()
{
    double lat = 45 * dh::unit::DEGREE;
    double alt = 100;
    std::cout << dh::sins::sins_Mpv_enu(lat, alt) << std::endl;
}

void test_cross_product()
{
    Eigen::Matrix<double, 3, 1> v1;
    Eigen::Matrix<double, 3, 1> v2;
    v1 << 1, 2, 3;
    v2 << 4, 7, 9;
    std::cout << dh::core::cross_product(v1, v2) << std::endl;
}

void test_angle_increment_to_quaternion()
{
    Eigen::Matrix<double, 3, 1> a;
    a << 1 * dh::unit::DEGREE, 0.5 * dh::unit::DEGREE, 0.8 * dh::unit::DEGREE;
    std::cout << dh::core::angle_increment_2_quat(a).coeffs() << std::endl;
}

void test_rotation_matrix_to_euler_angle()
{
}

void test_euler_angle_2_dcm()
{
    double r1 = 30 * dh::unit::DEGREE;
    double r2 = 3 * dh::unit::DEGREE;
    double r3 = 1 * dh::unit::DEGREE;

    Eigen::Matrix<double, 3, 3> mat;
    mat = dh::core::euler_angle_2_dcm(r1, r2, r3, dh::core::RotationAxis::Z, dh::core::RotationAxis::X, dh::core::RotationAxis::Y);

    std::cout << mat << std::endl;
}

void test_dot_product()
{
    Eigen::Matrix<double, 4, 1> v1;
    Eigen::Matrix<double, 4, 1> v2;
    v1 << 1, 2, 3, 4;
    v2 << 5, 8, 3, 5;
    std::cout << dh::core::dot_product(v1, v2) << std::endl;
}

void test_read_double_txt(std::string file1)
{
    double *d;
    int row = 41900;
    int col = 11;
    d = dh::read::double_txt_file(file1, row, col);
    std::cout << d[100] << "," << d[101] << "," << d[102] << "," << d[103] << std::endl;
}

void test_eigen_vector_data(){
    Eigen::Matrix<double, 3, 1> v;
    v << 1, 2, 3;
    std::cout << *v.data()<< std::endl;
}


void test_tg_cartesian3d(){
    dh::tg::TrajectoryGeneratorIn3DCartesianFrame traj;
    std::cout << traj.init_q.coeffs() << std::endl;
    std::cout << traj.init_position << std::endl;
}

void test_motion3d(){
    dh::tg::Motion3D m(1*dh::unit::DEGREE, 2*dh::unit::DEGREE, 30*dh::unit::DEGREE, 0, 0, 0, 0, 0, 0)

}

void test_vectory(){
    
    int ROW_SIZE = 10;
    int COL_SIZE = 10;

    std::vector<std::vector<double>> a(ROW_SIZE, std::vector<double>(COL_SIZE));
    int k = 0;
    for (int row = 0; row < ROW_SIZE; row++){
        for (int col = 0; col < COL_SIZE; col++)
        {
            a[row][col] = rand()/RAND_MAX;
        }
    }

    std::cout << a[1][2] << std::endl;
}

int main(int argc, char **argv)
{
    // std::string path = argv[1];
    test_vectory();
    
    return 0;
}