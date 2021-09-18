/*
 * @Author: your name
 * @Date: 2021-09-17 11:06:24
 * @LastEditTime: 2021-09-18 17:02:54
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/sinsprocessor.cpp
 */

#include <iostream>
#include <vector>

#include <ceres/ceres.h>


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
#include "my_imu_factor.h"

using namespace dh;
using namespace std;

void test1(){
  Trajectory3d traj;
  traj.addMotion(Motion3d(0, 1, 0, 0, 0, 0, 1, 0));
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

  // std::vector<PoseQPV> vpose;
  // np_to_pose<LocalNavigationParameter, PoseQPV>(vnp, vpose);
  // writeVector<PoseQPV>("/home/zz/桌面/cpp_project/dh/data/pose.txt", vpose);

  std::vector<ImuMeasurement6d> vimu, vimu0, dvimu;
  // pose_to_imu(vpose, vimu);
  np_to_imu(vnp, vimu);
  vimu0 = vimu;
  // np_to_imu(vnp, vimu);
  writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imu.txt", vimu);

  ImuErrorParameter imuerr;
  // imuerr.ba << 0, 100*C_ug, 0;
  imuerr.na << 10 * C_ug_per_SqHz, 20 * C_ug_per_SqHz, 50 * C_ug_per_SqHz;
  imu_add_error(vimu, imuerr);
  writeVector<ImuMeasurement6d>("/home/zz/桌面/cpp_project/dh/data/imuerr.txt", vimu);

  ImuVector diff = vimu - vimu0;
  std::cout << diff << std::endl;

  vector<LocalNavigationParameter> cal_vnp, diff_vnp;
  cal_vnp.push_back(np);
  SimpleLocalSinsProcessor sins(np);

  for (ImuVector::const_iterator it = vimu.begin();
       it != vimu.end();
       ++it)
  {
    sins.propagate(*it, dt);
    cal_vnp.push_back(sins.nav_param);
  }

  // std::cout << "size vnp = " << vnp.size() << std::endl;
  // std::cout << *(vnp.begin()) << std::endl;
  // std::cout << "size cal_vnp = " << cal_vnp.size() << std::endl;
  // std::cout << *(cal_vnp.begin()) << std::endl;

  // diff_vnp = cal_vnp - vnp;
  // std::cout << diff_vnp << std::endl;

  PreIntegrationTerm(vimu, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), imuerr);
}

bool test2(){
  Trajectory3d traj;
  traj.addMotion(Motion3d(0, 2, 0, 0, 0, 0, 1, 0));

  LocalNavigationParameter np(0, 0, 0, 0, 0, 0, 0, 0, 0);
  vector<LocalNavigationParameter> vnp;
  double dt = 0.01;
  generateTrajectory<LocalNavigationParameter>(vnp, np, traj, dt);

  std::vector<ImuMeasurement6d> vimu, vimu0, dvimu;
  np_to_imu(vnp, vimu);
  vimu0 = vimu;

  ImuErrorParameter imuerr;
  imuerr.ba << 0, 500*C_ug, 0;
  // imuerr.na << 10 * C_ug_per_SqHz, 20 * C_ug_per_SqHz, 50 * C_ug_per_SqHz;
  imu_add_error(vimu, imuerr);

  cout << "size np = " << vnp.size() << endl;
  cout << "size imu = " << vimu.size() << endl;

  SimpleLocalSinsProcessor sins(np);
  Eigen::Vector3d ba_init, bg_init;
  ba_init << 0, 0, 0;
  bg_init << 0, 0, 0;

  constexpr int terms_num = 20;
  double para_Pose[terms_num][7];
  double para_SpeedBias[terms_num][9];
  PreIntegrationTerm *preinte[terms_num];

  // ceres::Problem problem;
  int window_size = 10;
  int index_np = 0, index_pre = 0;
  int counter = 0;

  vector<LocalNavigationParameter> vnp_sins, vnp_gps;
  // vector<LocalNavigationParameter>::const_iterator it_np = vnp.begin();
  // vnp_gps.push_back(*(it_np++));

  const int step_length = 10;
  for (ImuVector::const_iterator it = vimu.begin();
       it != vimu.end();
       ++it, ++counter)
  {
    if(counter == 0){
      vnp_sins.push_back(sins.nav_param);
      preinte[0] = new PreIntegrationTerm(it->a, it->w, ba_init, bg_init, imuerr);
    }
    else {
      preinte[index_pre]->push_back(dt, it->a, it->w);
    }

    if (counter % step_length == 0 && counter != 0)
    {
      vnp_sins.push_back(sins.nav_param);
      preinte[++index_pre] = new PreIntegrationTerm(it->a, it->w, ba_init, bg_init, imuerr);
    }

    sins.propagate(*it, dt);
  }

  cout << "vnp_sins size = " << vnp_sins.size() << endl;
  cout << vnp_sins << endl;

  // for(int i = 0; i< sizeof(preinte); ++i)
  //   cout << preinte[i]->sum_dt << endl;
  

  // return true;

  // cout << "vnp_gps size = " << vnp_gps.size() << endl;
  // cout << vnp_gps << endl; // no need now.

  // int POSE_SIZE = vnp_sins.size() - 1;
  // cout << POSE_SIZE << endl;

  // set initial parameter value.
  vector<LocalNavigationParameter>::const_iterator it = vnp_sins.begin();
  for(int i = 0; i < terms_num; ++i, ++it){
    para_Pose[i][0] = it->px;
    para_Pose[i][1] = it->py;
    para_Pose[i][2] = it->pz;
    para_Pose[i][3] = it->q().x();
    para_Pose[i][4] = it->q().y();
    para_Pose[i][5] = it->q().z();
    para_Pose[i][6] = it->q().w();
    para_SpeedBias[i][0] = it->vx;
    para_SpeedBias[i][1] = it->vy;
    para_SpeedBias[i][2] = it->vz;
    para_SpeedBias[i][3] = 0;
    para_SpeedBias[i][4] = 0;
    para_SpeedBias[i][5] = 0;
    para_SpeedBias[i][6] = 0;
    para_SpeedBias[i][7] = 0;
    para_SpeedBias[i][8] = 0;
  }
  
  std::cout << para_SpeedBias[5][1] << endl;

  return true;

  ceres::Problem problem;
  for(int i = 0; i<terms_num-5; ++i){
    MyIMUFactor* imu_factor = new MyIMUFactor(preinte[i]);
    problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[i+1], para_SpeedBias[i+1]);
  }

  ceres::Solver::Options options;
  // options.max_num_iterations = 200;
  // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();

  // return summary.IsSolutionUsable(); 
}

int main(){
  test2();
  std::cout << "it's ok" << std::endl;
  return 0;
}