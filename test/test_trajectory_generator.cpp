/*
 * @Author: your name
 * @Date: 2021-09-01 20:29:24
 * @LastEditTime: 2021-09-01 20:44:55
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/test_trajectory_generator.cpp
 */

#include <iostream>
#include <string>

#include "trajectory_generator.h"

using namespace std;
using namespace dh;

int main(){
    Motion3d motion(0, 10, 1, 1, 1, 2, 2 ,2);

    cout << motion;
    cout << "hello world" << endl;
}