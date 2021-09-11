/*
 * @Author: your name
 * @Date: 2021-09-01 20:29:24
 * @LastEditTime: 2021-09-11 15:59:20
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/test_trajectory_generator.cpp
 */

#include <iostream>
#include <string>

#include "trajectory_generator.h"
#include "geometry.h"
#include "unit.h"
#include "mywrite.h"
#include "sins.h"

using namespace std;
using namespace dh::tg;
using namespace dh::geometry;
using namespace dh::unit;
using namespace dh::write;
using namespace dh::sins;

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
    NavigationParameter3d np(0, 0, 0, 0, 0, 0, 0, 0, 0);
    traj.addMotion(Motion3d(0, 10, 0, 0, 0, 0, 1, 0));
    traj.addMotion(10, 0, 0, 0, 0, 1, 0);
    traj.addMotion(90, 0, 0, 0, 0, 0, 0);
    traj.addMotion(9, 10*degree, 0, 0, 0, 0, 0);
    traj.addMotion(100, 0, 0, 0, 0, 0, 0);

    vector<NavigationParameter3d> vnp;

    generateTrajectory<NavigationParameter3d>(vnp, np, traj, 0.01);
    writeVector<NavigationParameter3d>("/home/zz/桌面/cpp_project/dh/data/np.txt", vnp);
    std::cout << "it's ok" << std::endl;
 
}

void test6(){
    Eigen::Vector3d lla;
    lla << 45*degree, 120*degree, 100;
    std::cout << lla << std::endl;
    Eigen::Vector3d ecef;
    ecef = lla_to_ecef(lla);
    std::cout << ecef << std::endl;
    lla = ecef_to_lla(ecef);
    // lla(0) = lla(0)/degree;
    // lla(1) = lla(1)/degree;
    std::cout << lla << std::endl;
}

int main(){
    test6();
    return 0;
}