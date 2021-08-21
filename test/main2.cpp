/*
 * @Author: your name
 * @Date: 2021-08-21 09:23:17
 * @LastEditTime: 2021-08-21 11:20:02
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

void test_zxy_to_quat(){
    Eigen::Vector3d v;
    v << 1*dh::unit::degree, 2*dh::unit::degree, 30*dh::unit::degree;
    Eigen::Quaterniond q = dh::geometry::zxy_euler_angle_to_quat(v);
    std::cout << q.coeffs() << std::endl;

    Eigen::Matrix<double, 3, 3> dcm = q.toRotationMatrix();
    std::cout << dcm << std::endl;
    std::cout << dcm(2,0) << std::endl;

    Eigen::Vector3d rpy = dh::geometry::quat_to_ypr(q)/dh::unit::degree;
    std::cout << rpy << std::endl;

}

void test_utils(){
    Eigen::Vector3d v;
    v << 1, 2, 3;
    std::cout << dh::utils::askew(v) << std::endl;
}

int main(int argc, char **argv)
{
    // std::string path = argv[1];
    test_utils();
    
    return 0;
}