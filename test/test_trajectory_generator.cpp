/*
 * @Author: your name
 * @Date: 2021-09-01 20:29:24
 * @LastEditTime: 2021-09-02 21:37:35
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
    Trajectory3d traj(0, 0, 0, 0, 0, 0, 0, 0, 0);
    double t = 0;
    traj.addMotion(Motion3d(0, 100, 0, 0, 0, 0, 1, 0));
    traj.addMotion(100, 0, 0, 0, 0, 10, 0);
    traj.briefReport();
}

void test3(){
    double rad = M_PI/180;
    double pitch = 1*rad;
    double yaw = 30*rad;
    double roll = 2*rad;
    std::cout << zxy_euler_angle_to_dcm(pitch, roll, yaw) << std::endl;

}

void test4(){
    double yaw = 2*M_PI + 0.1;
    yaw = -0.1;
    double pitch = 0;
    double roll = 0;
    ypr_standerlize(yaw, pitch, roll);
    std::cout << yaw << std::endl;
}

int main(){
    test4();
    return 0;
}