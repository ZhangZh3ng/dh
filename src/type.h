/*
 * @Author: your name
 * @Date: 2021-09-02 19:47:19
 * @LastEditTime: 2021-09-08 18:07:13
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/type.h
 */

#ifndef DH_TYPE_H
#define DH_TYPE_H

namespace dh{
namespace type{

  enum class EulerAngleType{ZYX, ZXY};

  enum class RotationAxis{X, Y, Z};

  enum class DataFormat
  {
    G2O_VERTEX_SE3,
    CERES_Pose3d,
    XYZ_NavigationParameter
  };

} // namespace type
} // namespace dh

#endif // DH_TYPE_H