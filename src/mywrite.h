/*
 * @Author: your name
 * @Date: 2021-09-04 15:04:02
 * @LastEditTime: 2021-09-19 15:11:03
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/io.h
 */

#ifndef DH_MYWRITE_H
#define DH_MYWRITE_H

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include "navigation_parameter.h"
#include "ceres_example_slam/pose_graph_3d/types.h"


using namespace ceres::examples;

namespace dh{

  bool writeNavigationParameters(std::fstream &file,
                                 const LocalNavigationParameter &np);
                            
  bool writeG2oVertexSE3(std::fstream &file,
                    const int pose_id,
                    const Pose3d& pose);

  bool writeCeresPose3d(std::fstream &file,
                        const int pose_id,
                        const Pose3d &pose);

  template<class T>
  bool writeVector(const std::string& filename, const std::vector<T>& data){
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile)
    {
      std::cout << "Error opening the file: " << filename;
      return false;
    }
    for (typename std::vector<T>::const_iterator it = data.begin();
         it != data.end();
         ++it)
    {
      outfile << *it << std::endl;
    }
    outfile.close();
    return true;
  }

  template<typename T>
  void coutArray(const T& arr){
    for(int i = 0; i < sizeof(arr)/ sizeof(arr[0]); ++i){
      std::cout << arr[i] << " ";
    }
    std::cout << std::endl;
  }
} // namespace dh

#endif
