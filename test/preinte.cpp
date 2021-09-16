/*
 * @Author: your name
 * @Date: 2021-09-13 11:06:16
 * @LastEditTime: 2021-09-16 21:23:01
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/preinte.cpp
 */

#include <iostream>
#include <vector>


#include "vinsmono/utility.h"
#include "vinsmono/integration_base.h"
#include "navigation_parameter.h"
#include "imu_parameter.h"
#include "trajectory_generator.h"
#include "constant.h"
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
    traj.addMotion(Motion3d(0, 0.1, 0, 0, 0, 0, 0, 0));
    // traj.addMotion(10, 0, 0, 0, 0, 1, 0);
    // traj.addMotion(90, 0, 0, 0, 0, 0, 0);
    // traj.addMotion(9, 10*C_degree_per_second, 0, 0, 0, 0, 0);
    // traj.addMotion(100, 0, 0, 0, 0, 0, 0);
    traj.briefReport();

    LocalNavigationParameter np = LocalNavigationParameter(0, 0, 0, 0, 1, 0, 0, 0, 0);
    vector<LocalNavigationParameter> vnp;
    generateTrajectory(vnp, np, traj, 0.01);
    // writeVector("/home/zz/桌面/cpp_project/dh/data/np.txt", vnp);

    std::vector<ImuMeasurement6d> vimu,vimu0, dvimu;
    np_to_imu(vnp, vimu);
    vimu0 = vimu;
    // writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imu.txt", vimu);

    ImuErrorParameter imuerr;
    imuerr.ba << 0, 100*C_ug, 0;
    imuerr.na << 10*C_ug_per_SqHz, 20*C_ug_per_SqHz, 50*C_ug_per_SqHz;
    imu_add_error(vimu, imuerr);
    // writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imuerr.txt", vimu);

    // ImuVector diff = vimu - vimu0;
    // std::cout << diff << std::endl;

    Eigen::Vector3d acc, gyr, ba, bg;
    acc << 0, 0, 0;
    gyr << 0, 0, 0;
    ba << 0, 0, 0;
    bg << 0, 0, 0;

    // std::cout << "initial np = " << *(vnp.begin()) << std::endl;
    // std::cout << "final np = " << *(vnp.rbegin()) << std::endl;
    std::cout << "np size = " << std::endl;
    std::cout << vnp << std::endl;

    std::cout << "vimu0 size = " << vimu0.size() << std::endl;
    std::cout << vimu0 << std::endl;
    // std::cout << "vimu size = " << vimu.size() << std::endl;
    // std::cout << vimu << std::endl;


    PreIntegrationTerm preinte(vimu0, ba, bg, imuerr);
    std::cout << "sum_dt = " << preinte.sum_dt << std::endl;
    std::cout << "initial time_stamp = " << preinte.initial_time_stamp << std::endl;
    std::cout << "dt_buff size = " << preinte.dt_buf.size() << std::endl;
    std::cout << preinte.dt_buf << std::endl;

    Eigen::Matrix<double, 15, 1> residual;
    residual = preinte.evaluate(vnp.begin()->p(), vnp.begin()->q(), vnp.begin()->v(),
                                ba, bg,
                                (vnp.end()-2)->p(), (vnp.end()-2)->q(), (vnp.end()-2)->v(),
                                ba, bg);
    
    std::cout << "residual is: \n" << residual << std::endl;
    std::cout << "gogogo" << std::endl;
}

void test4(){
    Trajectory3d traj;
    traj.addMotion(Motion3d(0, 1, 0, 0, 0, 0, 0, 0));
    // traj.addMotion(10, 0, 0, 0, 0, 1, 0);
    // traj.addMotion(90, 0, 0, 0, 0, 0, 0);
    // traj.addMotion(9, 10*C_degree_per_second, 0, 0, 0, 0, 0);
    // traj.addMotion(100, 0, 0, 0, 0, 0, 0);
    traj.briefReport();

    LocalNavigationParameter np = LocalNavigationParameter(0, 0, 0, 0, 1, 0, 0, 0, 0);
    vector<LocalNavigationParameter> vnp;
    generateTrajectory(vnp, np, traj, 0.01);
    std::cout << "trajectory total time is: " << traj.total_time << std::endl;
    std::cout << "total np is:\n" << std::endl;
    std::cout << vnp << std::endl;
    // writeVector("/home/zz/桌面/cpp_project/dh/data/np.txt", vnp);

    std::vector<ImuMeasurement6d> vimu,vimu0, dvimu;
    np_to_imu(vnp, vimu);
    vimu0 = vimu;
    // writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imu.txt", vimu);

    ImuErrorParameter imuerr;
    imuerr.ba << 0, 100*C_ug, 0;
    imuerr.na << 10*C_ug_per_SqHz, 20*C_ug_per_SqHz, 50*C_ug_per_SqHz;
    imu_add_error(vimu, imuerr);
    // writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imuerr.txt", vimu);

    // ImuVector diff = vimu - vimu0;
    // std::cout << diff << std::endl;

    Eigen::Vector3d acc, gyr, ba, bg;
    acc << 0, 0, 0;
    gyr << 0, 0, 0;
    ba << 0, 0, 0;
    bg << 0, 0, 0;

    std::cout << "initial np = " << *(vnp.begin()) << std::endl;
    std::cout << "final np = " << *(vnp.rbegin()) << std::endl;

    std::cout << "vimu0 size = " << vimu0.size() << std::endl;
    std::cout << vimu0 << std::endl;
    std::cout << "vimu size = " << vimu.size() << std::endl;
    // std::cout << vimu << std::endl;
}

void test5(){
  Eigen::Vector3d rot(0.1, 0.1, 0.1);
  std::cout << "Utility" << Utility::deltaQ(rot).coeffs() << std::endl;
  std::cout << "Utility norm = " << Utility::deltaQ(rot).norm() << std::endl;
  std::cout << "my" << rot_to_quat(rot).coeffs() << std::endl;
  std::cout << "my norm = " << rot_to_quat(rot).norm() << std::endl;
}

void test6(){
   Trajectory3d traj;
    traj.addMotion(Motion3d(0, 1, 0, 0, 0, 0, 0, 0));
    // traj.addMotion(10, 0, 0, 0, 0, 1, 0);
    // traj.addMotion(90, 0, 0, 0, 0, 0, 0);
    // traj.addMotion(9, 10*C_degree_per_second, 0, 0, 0, 0, 0);
    // traj.addMotion(100, 0, 0, 0, 0, 0, 0);
    traj.briefReport();

    LocalNavigationParameter np = LocalNavigationParameter(0, 0, 0, 0, 1, 0, 0, 0, 0);
    vector<LocalNavigationParameter> vnp;
    generateTrajectory(vnp, np, traj, 0.01);
    std::cout << "trajectory total time is: " << traj.total_time << std::endl;
    std::cout << "total np is:\n" << std::endl;
    std::cout << vnp << std::endl;
    // writeVector("/home/zz/桌面/cpp_project/dh/data/np.txt", vnp);

    std::vector<ImuMeasurement6d> vimu,vimu0, dvimu;
    np_to_imu(vnp, vimu);
    vimu0 = vimu;
    // writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imu.txt", vimu);

    ImuErrorParameter imuerr;
    imuerr.ba << 0, 100*C_ug, 0;
    imuerr.na << 10*C_ug_per_SqHz, 20*C_ug_per_SqHz, 50*C_ug_per_SqHz;
    imu_add_error(vimu, imuerr);
    // writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imuerr.txt", vimu);

    // ImuVector diff = vimu - vimu0;
    // std::cout << diff << std::endl;

    Eigen::Vector3d acc, gyr, ba, bg;
    acc << 0, 0, 0;
    gyr << 0, 0, 0;
    ba << 0, 0, 0;
    bg << 0, 0, 0;

    std::cout << "initial np = " << *(vnp.begin()) << std::endl;
    std::cout << "final np = " << *(vnp.rbegin()) << std::endl;

    std::cout << "vimu0 size = " << vimu0.size() << std::endl;
    std::cout << vimu0 << std::endl;
    std::cout << "vimu size = " << vimu.size() << std::endl;
    // std::cout << vimu << std::endl;
}


int main(){
  test6();
  std::cout << "it's ok" << std::endl;
}