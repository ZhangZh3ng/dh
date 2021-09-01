/*
 * @Author: your name
 * @Date: 2021-08-20 19:06:51
 * @LastEditTime: 2021-08-29 09:46:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/enum.hpp
 */

#ifndef DH_TYPE_H
#define DH_TYPE_H

#include<ceres/types.h>

namespace dh{
namespace type{
    /**
     * @brief Using to indicate a rotation rotates around with which axis.
     */
    enum RotationAxis
    {
        X,
        Y,
        Z
    };

    /**
     * @brief euler angle type such as zxy, zyx...
     * 
     */
    enum EulerAngleType
    {
        ZYX,
        ZXY
    };

    /**
     * @brief 
     * 
     */
    enum MessageType
    {
        LABEL,
        INDEX,
        TIME_STAMP,
        START_TIME,
        END_TIME,
        PITCH_RATE,
        ROLL_RATE,
        YAW_RATE,
        ANGULAR_VELOCITY_IN_BODY_X,
        ANGULAR_VELOCITY_IN_BODY_Y,
        ANGULAR_VELOCITY_IN_BODY_Z,
        ACCELERATION_IN_BODY_X,
        ACCELERATION_IN_BODY_Y,
        ACCELERATION_IN_BODY_Z,
        ACCELERATION_IN_FORWARD,
        ACCELERATION_IN_RIGTH,
        ACCELERATION_IN_UP
    };

    enum PositionSystem
    {
        LLA,
        ECEF,
        XYZ
    };

    enum ReferenceFrame
    {
        EAST_NORTH_UP,
        NORTH_EAST_DOWN,
        EARTH_CERTER_EARTH_FIXED,
        ORTHOGONAL_XYZ
    };

    enum TrajectoryDataFormat{
        PSINS,
        POSES,
        G2O
    };
}
}

#endif