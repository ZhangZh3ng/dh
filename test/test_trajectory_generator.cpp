/*
 * @Author: your name
 * @Date: 2021-09-01 20:29:24
 * @LastEditTime: 2021-09-10 14:41:13
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


int main(){
    return 0;
}