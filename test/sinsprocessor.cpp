/*
 * @Author: your name
 * @Date: 2021-09-17 11:06:24
 * @LastEditTime: 2021-09-17 14:40:55
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/sinsprocessor.cpp
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
#include "common_operator.h"
#include "preintegration.h"
#include "sins_processor.h"

using namespace dh;
using namespace std;

void test1(){
  Trajectory3d traj;
    traj.addMotion(Motion3d(0, 10, 0, 0, 0, 0, 1, 0));
    // traj.addMotion(10, 0, 0, 0, 0, 1, 0);
    // traj.addMotion(90, 0, 0, 0, 0, 0, 0);
    // traj.addMotion(9, 10*C_degree_per_second, 0, 0, 0, 0, 0);
    // traj.addMotion(100, 0, 0, 0, 0, 0, 0);
    traj.briefReport();

    LocalNavigationParameter np(0, 0, 0, 0, 0, 0, 0, 0, 0);
    vector<LocalNavigationParameter> vnp;
    double dt = 0.01;
    generateTrajectory<LocalNavigationParameter>(vnp, np, traj, dt);
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
    // imuerr.ba << 0, 100*C_ug, 0;
    imuerr.na << 10*C_ug_per_SqHz, 20*C_ug_per_SqHz, 50*C_ug_per_SqHz;
    imu_add_error(vimu, imuerr);
    writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imuerr.txt", vimu);

    ImuVector diff = vimu - vimu0;
    std::cout << diff << std::endl;

    vector<LocalNavigationParameter> cal_vnp, diff_vnp;
    cal_vnp.push_back(np);
    SimpleLocalSinsProcessor sins(np);

    for(ImuVector::const_iterator it = vimu.begin();
    it != vimu.end();
    ++it){
      sins.propogate(*it, dt);
      cal_vnp.push_back(sins.np);
    }
    
    std::cout << "size vnp = " << vnp.size() << std::endl;
    std::cout << *(vnp.begin()) << std::endl;
    std::cout << "size cal_vnp = " << cal_vnp.size() << std::endl;
    std::cout << *(cal_vnp.begin()) << std::endl;

    diff_vnp = cal_vnp - vnp;
    std::cout << diff_vnp << std::endl;
}

int main(){
  test1();
  std::cout << "it's ok" << std::endl;
  return 0;
}