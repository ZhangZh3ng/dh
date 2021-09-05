/*
 * @Author: your name
 * @Date: 2021-09-04 16:30:41
 * @LastEditTime: 2021-09-05 10:38:45
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/write.cpp
 */

#include <iostream>

#include "write.h"

namespace dh{
namespace write{
  bool writeNavigationParameters(std::fstream &file,
                                 const NavigationParameter3d &np){
    file << np.yaw << " " << np.pitch << " " << np.roll << " "
         << np.vx << " " << np.vy << " " << np.vz << " "
         << np.px << " " << np.py << " " << np.pz << std::endl;
    return true;
  }

  bool writeG2oPose(std::fstream &file,
                    const int pose_id,
                    const Pose3d& pose){
    file << Pose3d::name() << " " << pose_id << " " << pose.p.x()
         << " " << pose.p.y() << " " << pose.p.z() << " " << pose.q.x()
         << " " << pose.q.y() << " " << pose.q.z() << " " << pose.q.w()
         << std::endl;
    return true;
  }
} // namespace dh
} // namespace write