/*
 * @Author: your name
 * @Date: 2021-09-04 15:04:02
 * @LastEditTime: 2021-09-10 15:18:05
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/io.h
 */

#ifndef DH_WRITE_H
#define DH_WRITE_H

#include <iostream>
#include <string>
#include <fstream>

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
} // namespace dh
} // namespace write

#endif
