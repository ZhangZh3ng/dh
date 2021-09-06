/*
 * @Author: your name
 * @Date: 2021-09-02 19:47:19
 * @LastEditTime: 2021-09-05 18:13:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/type.h
 */

#ifndef DH_TYPE_H
#define DH_TYPE_H

namespace dh{
namespace type{

  enum EulerAngleType{ZYX, ZXY};

  enum RotationAxis{X, Y, Z};

  enum DataFormat
  {
    G2O_VERTEX_SE3,
    CERES_Pose3d,
    DH_NavigationParameter3d
  };

} // namespace type
} // namespace dh

#endif // DH_TYPE_H