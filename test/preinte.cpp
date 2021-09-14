/*
 * @Author: your name
 * @Date: 2021-09-13 11:06:16
 * @LastEditTime: 2021-09-14 18:49:29
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/preinte.cpp
 */

#include <iostream>
#include <vector>

#include "vinsmono/integration_base.h"
#include "parameter.h"
#include "trajectory_generator.h"
#include "unit.h"
#include "mywrite.h"
#include "myio.h"
#include "preintegration.h"


using namespace std;
using namespace dh;

void test1(){
  Eigen::Vector3d acc, gyr, ba, bg;
  acc << 0, 0, 0;
  gyr << 0, 0, 0;
  ba << 0, 0, 0;
  bg << 0, 0, 0;
  IntegrationBase inte(acc, gyr, ba, bg);
  std::cout << "gogogo" << std::endl;
}

void test2(){
    Trajectory3d traj;
    traj.addMotion(Motion3d(0, 10, 0, 0, 0, 0, 1, 0));
    traj.addMotion(10, 0, 0, 0, 0, 1, 0);
    traj.addMotion(90, 0, 0, 0, 0, 0, 0);
    traj.addMotion(9, 10*C_degree_per_second, 0, 0, 0, 0, 0);
    traj.addMotion(100, 0, 0, 0, 0, 0, 0);
    traj.briefReport();

    LocalNavigationParameter np(0, 0, 0, 0, 0, 0, 0, 0, 0);
    vector<LocalNavigationParameter> vnp;
    generateTrajectory<LocalNavigationParameter>(vnp, np, traj, 0.01);
    writeVector<LocalNavigationParameter>("/home/zz/桌面/cpp_project/dh/data/np.txt", vnp);

    std::vector<PoseQPV> vpose;
    np_to_pose<LocalNavigationParameter, PoseQPV>(vnp, vpose);
    writeVector<PoseQPV>("/home/zz/桌面/cpp_project/dh/data/pose.txt", vpose);

    std::vector<ImuMeasurement6d> vimu,vimu0, dvimu;
    pose_to_imu(vpose, vimu);
    vimu0 = vimu;
    // np_to_imu(vnp, vimu);
    writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imu.txt", vimu);

    ImuErrorParameter imuerr;
    imuerr.ba << 0, 100*C_ug, 0;
    imuerr.na << 10*C_ug_per_SqHz, 20*C_ug_per_SqHz, 50*C_ug_per_SqHz;
    imu_add_error(vimu, imuerr);
    writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imuerr.txt", vimu);

    ImuVector diff = vimu - vimu0;
    std::cout << diff << std::endl;
}

void test3(){
  Trajectory3d traj;
    traj.addMotion(Motion3d(0, 10, 0, 0, 0, 0, 1, 0));
    traj.addMotion(10, 0, 0, 0, 0, 1, 0);
    traj.addMotion(90, 0, 0, 0, 0, 0, 0);
    traj.addMotion(9, 10*C_degree_per_second, 0, 0, 0, 0, 0);
    traj.addMotion(100, 0, 0, 0, 0, 0, 0);
    traj.briefReport();

    LocalNavigationParameter np(0, 0, 0, 0, 0, 0, 0, 0, 0);
    vector<LocalNavigationParameter> vnp;
    generateTrajectory(vnp, np, traj, 0.01);
    writeVector("/home/zz/桌面/cpp_project/dh/data/np.txt", vnp);

    std::vector<ImuMeasurement6d> vimu,vimu0, dvimu;
    np_to_imu(vnp, vimu);
    vimu0 = vimu;
    writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imu.txt", vimu);

    ImuErrorParameter imuerr;
    imuerr.ba << 0, 100*C_ug, 0;
    imuerr.na << 10*C_ug_per_SqHz, 20*C_ug_per_SqHz, 50*C_ug_per_SqHz;
    imu_add_error(vimu, imuerr);
    writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imuerr.txt", vimu);

    ImuVector diff = vimu - vimu0;
    std::cout << diff << std::endl;

    Eigen::Vector3d acc, gyr, ba, bg;
    acc << 0, 0, 0;
    gyr << 0, 0, 0;
    ba << 0, 0, 0;
    bg << 0, 0, 0;
    IntegrationBase inte(acc, gyr, ba, bg);
    // PreIntegrationTerm inte(acc, gyr, ba, bg);
    // PreIntegrationTerm(vimu0, ba, bg, imuerr);
    std::cout << "gogogo" << std::endl;
}

int main(){
  test1();
  std::cout << "it's ok" << std::endl;
}