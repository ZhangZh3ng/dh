/*
 * @Author: your name
 * @Date: 2021-09-13 11:06:16
 * @LastEditTime: 2021-09-13 20:17:22
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/preinte.cpp
 */

#include <iostream>
#include "vinsmono/integration_base.h"

void test01(){
  // IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
  //                   const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
  Eigen::Vector3d acc, gyr, ba, bg;
  acc << 0, 0, 0;
  gyr << 0, 0, 0;
  ba << 0, 0, 0;
  bg << 0, 0, 0;
  IntegrationBase inte(acc, gyr, ba, bg);
  std::cout << "gogogo" << std::endl;
}

int main(){
  test01();
  std::cout << "it's ok" << std::endl;
}