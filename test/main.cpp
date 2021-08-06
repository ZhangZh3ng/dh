/*
 * @Author: Zhang Zheng
 * @Date: 2021-08-05 20:34:22
 * @LastEditTime: 2021-08-06 20:51:25
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/main.cpp
 */

#include "core.hpp"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>

void test(){
    dh::core::hello();
    Eigen::Matrix<double, 4, 1> q;
    q << 0, 1, 0, 0;
    Eigen::Matrix<double, 3, 1> r;
    r << 1, 2, 3;
    std::cout << dh::core::quat_multiply_vec(q, r) << std::endl;
    Eigen::Matrix<double, 3, 3> mat;
    mat = dh::core::askew(r);
    std::cout << mat(1,2) << std::endl;
    std::cout << dh::core::quat_add_rot(q, r) << std::endl;
    std::cout << dh::core::vee(mat) << std::endl;
}

int main(int argc, char** argv){
    test();
    return 0;
}