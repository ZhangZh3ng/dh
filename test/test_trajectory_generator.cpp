/*
 * @Author: your name
 * @Date: 2021-09-01 20:29:24
 * @LastEditTime: 2021-09-13 10:07:34
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
using namespace dh;

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
    q = quat_add_rot(q, rot);
    std::cout << "after add rot: \n" << q.coeffs() << std::endl;

    Eigen::Vector3d rot2;
    rot2 = quat_increment_to_rot(q_begin, q);
    std::cout << rot2 << std::endl;
}

void test5(){
    Trajectory3d traj;
    LocalNavigationParameter np(0, 0, 0, 0, 0, 0, 0, 0, 0);
    traj.addMotion(Motion3d(0, 10, 0, 0, 0, 0, 1, 0));
    traj.addMotion(10, 0, 0, 0, 0, 1, 0);
    traj.addMotion(90, 0, 0, 0, 0, 0, 0);
    traj.addMotion(9, 10*C_degree_per_second, 0, 0, 0, 0, 0);
    traj.addMotion(100, 0, 0, 0, 0, 0, 0);
    traj.briefReport();

    vector<LocalNavigationParameter> vnp;
    generateTrajectory<LocalNavigationParameter>(vnp, np, traj, 0.01);
    writeVector<LocalNavigationParameter>("/home/zz/桌面/cpp_project/dh/data/np.txt", vnp);

    std::vector<PoseQPV> vpose;
    np_to_pose<LocalNavigationParameter, PoseQPV>(vnp, vpose);
    writeVector<PoseQPV>("/home/zz/桌面/cpp_project/dh/data/pose.txt", vpose);

    std::vector<ImuMeasurement6d> vimu;
    pose_to_imu(vpose, vimu);
    // np_to_imu(vnp, vimu);
    writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imu.txt", vimu);

    ImuErrorParameter imuerr;
    imuerr.ba << 0, 0.1, 0;
    imuerr.na << 0.01, 0.02, 0.05;
    imu_add_error(vimu, imuerr);
    writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imuerr.txt", vimu);
    std::cout << "it's ok" << std::endl;
}

void test6(){
    Eigen::Vector3d lla;
    lla << 45*C_degree, 120*C_degree, 100;
    std::cout << lla << std::endl;
    Eigen::Vector3d ecef;
    ecef = lla_to_ecef(lla);
    std::cout << ecef << std::endl;
    lla = ecef_to_lla(ecef);
    // lla(0) = lla(0)/degree;
    // lla(1) = lla(1)/degree;
    std::cout << lla << std::endl;
}

void test7(){
    Eigen::Quaterniond q1, q2, dq;
    q1 = Eigen::Quaterniond::Identity();
    Eigen::Vector3d rot;
    rot << 1*C_degree, 0.5*C_degree, 0*C_degree;

    q2 = quat_add_rot(q1, rot);
    std::cout << "q2=\n" << q2.coeffs() << std::endl;

    dq = q1.inverse() * q2;
    // dq = q1.inverse();
    std::cout << "dq=\n" << dq.coeffs() << std::endl;

    rot = quat_increment_to_rot(q1, q2);
    std::cout<< rot/C_degree << std::endl;
}

int main(){
    test5();
    return 0;
}