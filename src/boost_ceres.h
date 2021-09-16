/*
 * @Author: your name
 * @Date: 2021-09-16 21:11:41
 * @LastEditTime: 2021-09-16 21:21:52
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/boost_ceres.h
 */

#ifndef DH_BOOST_CERES_H
#define DH_BOOST_CERES_H

#include "ceres_example_slam/pose_graph_3d/types.h"


namespace dh{

  inline Pose3d operator-(Pose3d &p_end, Pose3d &p_begin)
  {
    Pose3d pbe;
    pbe.q = p_begin.q.conjugate() * p_end.q;
    pbe.p = p_begin.q * (p_end.p - p_begin.p);
    return pbe;
  }

} // namespace dh
#endif // DH_BOOST_CERES_H