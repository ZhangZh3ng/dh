/*
 * @Author: your name
 * @Date: 2021-09-01 20:29:24
 * @LastEditTime: 2021-09-07 19:30:46
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

void test2(){
    // Trajectory3d traj(0, 0, 0, 0, 0, 0, 0, 0, 0);
    // double t = 0;
    // traj.addMotion(Motion3d(0, 100, 0, 0, 0, 0, 1, 0));
    // traj.addMotion(Motion3d(10, 50, 1, 2, 3, 4, 5, 6));
    // traj.addMotion(100, 0, 0, 0, 0, 10, 0);
    // Eigen::Vector3d w,a;
    // traj.getAngleVelocityAndAcceleration(w, a, 20);
    // traj.briefReport();
    // std::cout << "w: " << w << std::endl;
    // std::cout << "a: " << a << std::endl;

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

void test5(){
    // const double deg = M_PI/180;
    // Trajectory3d traj(XyzNavigationParameter(0, ));
    // // double t = 0;
    // traj.addMotion(Motion3d(0, 100, 0, 0, 0, 0, 1, 0));
    // traj.addMotion(100, 0, 0, 0, 0, 0, 0);
    // traj.addMotion(Motion3d(100, 110, 9*deg, 0, 0, 0, 0, 0));
    // traj.briefReport();
    // TrajectoryGenerator tg;
    // tg.data_format = CERES_Pose3d;
    // tg.generate("/home/zz/桌面/cpp_project/dh/data/2.txt", traj);

    // VectorOfNavigationParameter3d vnp;
    // tg.generate(vnp, traj);

    // for (std::vector<NavigationParameter3d>::iterator it = vnp.begin();
    //      it != vnp.end();
    //      ++it)
    // {
    //     std::cout << *it << std::endl;
    // }

    std::cout << "it's ok" << std::endl;
}

int main(){
    test5();
    return 0;
}