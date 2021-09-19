/*
 * @Author: your name
 * @Date: 2021-09-17 11:06:24
 * @LastEditTime: 2021-09-19 16:10:34
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

void test2(){
  Trajectory3d traj;
  traj.addMotion(Motion3d(0, 2, 0, 0, 0, 0, 1, 0));

  LocalNavigationParameter np(0, 0, 0, 0, 0, 0, 0, 0, 0);
  vector<LocalNavigationParameter> vnp;
  double dt = 0.01;
  generateTrajectory<LocalNavigationParameter>(vnp, np, traj, dt);

  std::vector<ImuMeasurement6d> vimu, vimu0, dvimu;
  np_to_imu(vnp, vimu);
  vimu0 = vimu;

  ImuErrorParameter imuerr, imuerr1;
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

  imuerr1.na << 0.019, 0.019, 0.019;
  imuerr1.ng << 0.0007, 0.0007, 0.0007;
  imuerr1.rwa << 0.012, 0.012, 0.012;
  imuerr1.rwg << 0.0004, 0.0004, 0.0004;

  const int step_length = 10;
  for (ImuVector::const_iterator it = vimu0.begin();
       it != vimu0.end();
       ++it, ++counter)
  {
    if(counter == 0){
      // vnp_sins.push_back(sins.nav_param);
      preinte[0] = new PreIntegrationTerm(it->a, it->w, ba_init, bg_init, imuerr1);
    }
    else {
      preinte[index_pre]->push_back(dt, it->a, it->w);
    }

    if (counter % step_length == 0 && counter != 0)
    {
      // vnp_sins.push_back(sins.nav_param);
      preinte[++index_pre] = new PreIntegrationTerm(it->a, it->w, ba_init, bg_init, imuerr1);
    }

    // sins.propagate(*it, dt);
  }

  counter = 0;
  for(ImuVector::const_iterator it = vimu.begin();
  it != vimu.end();
  ++it, ++counter){
    if(counter%step_length == 0)
      vnp_sins.push_back(sins.nav_param);
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
    para_SpeedBias[i][3] = ba_init(0);
    para_SpeedBias[i][4] = ba_init(1);
    para_SpeedBias[i][5] = ba_init(2);
    para_SpeedBias[i][6] = bg_init(0);
    para_SpeedBias[i][7] = bg_init(1);
    para_SpeedBias[i][8] = bg_init(2);
  }
  
  
  MyIMUFactor* imufacetor = new MyIMUFactor(preinte[0]);
  double pa0[7], pa1[9], pa2[7], pa3[9];
  for(int i = 0; i < sizeof(pa0)/sizeof(pa0[1]); ++i){
    pa0[i] = para_Pose[0][i];
  }
  for(int i = 0; i < sizeof(pa1)/sizeof(pa1[1]); ++i){
    pa1[i] = para_SpeedBias[0][i];
  }

  for(int i = 0; i < sizeof(pa2)/sizeof(pa2[1]); ++i){
    pa2[i] = para_Pose[1][i];
  }
  for(int i = 0; i < sizeof(pa3)/sizeof(pa3[1]); ++i){
    pa3[i] = para_SpeedBias[1][i];
  }

  coutArray(pa0);
  coutArray(pa1);
  coutArray(pa2);
  coutArray(pa3);
  
  // return;

  double *param[4] = {pa0, pa1, pa2, pa3};
  std::cout << "residual = " << std::endl;
  std::cout << imufacetor->pre_integration->evaluate(param) << std::endl;;
  // return;

  double residual[15];
  double jaco_0[15*7], jaco_1[15*9], jaco_2[15*7], jaco_3[15*9];
  double *jacobian[4] = {jaco_0, jaco_1, jaco_2, jaco_3};

  std::cout << "jaco0 = " << endl;
  coutArray(jaco_0);
  // coutArray(residual);

  // return;

  // imufacetor->Evaluate(param, residual, jacobian);
  ceres::Problem problem;
  for(int i = 0; i<10; ++i){
    MyIMUFactor* imu_factor = new MyIMUFactor(preinte[i]);
    problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[i+1], para_SpeedBias[i+1]);
  }
  // cout << terms_num - 5 << endl;
  cout << "residual blocks num:" << problem.NumResidualBlocks() << endl;
  cout << "parameter blocks num:"  << problem.NumParameterBlocks() << endl;

  // cout << para_Pose[2][1] << endl;
  ceres::Solver::Options options;
  // options.max_num_iterations = 200;
  // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << '\n';

  std::cout << para_SpeedBias[0][4] << std::endl;

  // cout << summary.IsSolutionUsable();
}

int main(){
  test2();
  std::cout << "it's ok" << std::endl;
  return 0;
}