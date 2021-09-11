/*
 * @Author: your name
 * @Date: 2021-09-01 20:29:24
 * @LastEditTime: 2021-09-11 08:43:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/test_trajectory_generator.cpp
 */

#include <iostream>
#include <string>

#include "trajectory_generator.h"
#include "geometry.h"

using namespace std;
using namespace dh::tg;
using namespace dh::geometry;

void test1(){
    Motion3d motion(0, 10, 1, 1, 1, 2, 2 ,2);
    cout << motion;
    cout << "hello world" << endl;
}

void test3(){
    double rad = M_PI/180;
    double pitch = 1*rad;
    double yaw = 30*rad;
    double roll = 2*rad;
    std::cout << zxy_euler_angle_to_dcm(pitch, roll, yaw) << std::endl;
}

void test4(){
    const double deg = M_PI/180;
    double yaw = 400*deg;
    double pitch = 91*deg;
    double roll = 190*deg;
    ypr_standerlize(yaw, pitch, roll);
    std::cout << "yaw:" << yaw/deg << std::endl;
    std::cout << "pitch:" << pitch/deg << std::endl;
    std::cout << "roll:" << roll/deg << std::endl;
}


void test2(){
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_begin = q;
    std::cout << "origin quaternion: \n" << q.coeffs() << std::endl;

    Eigen::Vector3d rot;
    rot << 0.11, 0.2, 1.5;
    quat_update_by_rot(q, rot);
    std::cout << "after add rot: \n" << q.coeffs() << std::endl;

    Eigen::Vector3d rot2;
    rot2 = quat_increment_to_rot(q_begin, q);
    std::cout << rot2 << std::endl;
}

void test5(){
    Trajectory3d traj;
}

int main(){
    test5();
    return 0;
}