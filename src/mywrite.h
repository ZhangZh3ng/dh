/*
 * @Author: your name
 * @Date: 2021-09-04 15:04:02
 * @LastEditTime: 2021-09-11 11:02:46
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

#include "parameter.h"
#include "slam/pose_graph_3d/types.h"


using namespace dh::parameter;
using namespace ceres::examples;

namespace dh{
namespace write{

  bool writeNavigationParameters(std::fstream &file,
                                 const NavigationParameter3d &np);
                            
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
} // namespace dh
} // namespace write

#endif
