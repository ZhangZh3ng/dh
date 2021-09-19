/*
 * @Author: your name
 * @Date: 2021-09-13 20:11:28
 * @LastEditTime: 2021-09-19 15:27:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/preintegration.h
 */

#ifndef DH_PREINTEGRATION_H
#define DH_PREINTEGRATION_H

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "navigation_parameter.h"
#include "geometry.h"
#include "vinsmono/utility.h"
#include "utils.h"

using namespace Eigen;
using namespace dh;

namespace dh{
  class PreIntegrationTerm{
  public:
    PreIntegrationTerm() = delete;
    PreIntegrationTerm(const Eigen::Vector3d &_acc_0,
                       const Eigen::Vector3d &_gyr_0,
                       const Eigen::Vector3d &_linearized_ba,
                       const Eigen::Vector3d &_linearized_bg,
                       const ImuErrorParameter &_imuerr = ImuErrorParameter::Zero())
        : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
          linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
          jacobian{Eigen::Matrix<double, 15, 15>::Identity()},
          covariance{Eigen::Matrix<double, 15, 15>::Zero()},
          sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()},
          delta_q{Eigen::Quaterniond::Identity()}, 
          delta_v{Eigen::Vector3d::Zero()},
          imuerr(_imuerr)
    {
      noise = Eigen::Matrix<double, 18, 18>::Zero();
      // noise.block<3, 3>(0, 0) = (imuerr.na(0) * imuerr.na(0)) * Eigen::Matrix3d::Identity();
      // noise.block<3, 3>(3, 3) = (imuerr.ng(0)  * imuerr.ng(0)) * Eigen::Matrix3d::Identity();
      // noise.block<3, 3>(6, 6) = (imuerr.na(0)  * imuerr.na(0)) * Eigen::Matrix3d::Identity();
      // noise.block<3, 3>(9, 9) = (imuerr.ng(0)  * imuerr.ng(0)) * Eigen::Matrix3d::Identity();
      // noise.block<3, 3>(12, 12) = (imuerr.rwa(0)  * imuerr.rwa(0)) * Eigen::Matrix3d::Identity();
      // noise.block<3, 3>(15, 15) = (imuerr.rwg(0)  * imuerr.rwg(0)) * Eigen::Matrix3d::Identity();
      // noise.block<3, 3>(0, 0) = std_to_covariance<3>(imuerr.ba);
      noise.block<3, 3>(0, 0) = std_to_covariance(imuerr.na);
      noise.block<3, 3>(3, 3) = std_to_covariance(imuerr.ng);
      noise.block<3, 3>(6, 6) = std_to_covariance(imuerr.na);
      noise.block<3, 3>(9, 9) = std_to_covariance(imuerr.ng);
      noise.block<3, 3>(12, 12) = std_to_covariance(imuerr.rwa);
      noise.block<3, 3>(15, 15) = std_to_covariance(imuerr.rwg);
    }

    PreIntegrationTerm(const ImuVector &vimu,
                       const Eigen::Vector3d &_linearized_ba,
                       const Eigen::Vector3d &_linearized_bg,
                       const ImuErrorParameter &_imuerr = ImuErrorParameter::Zero())
    {
      ImuVector::const_iterator it_imu = vimu.begin();
      Eigen::Vector3d _acc, _gyr;
      new(this) PreIntegrationTerm((*it_imu).a, (*it_imu).w, _linearized_ba, _linearized_bg, _imuerr);
      this->initial_time_stamp = (*it_imu).time_stamp;
      // this->use_mid_integration = false;
      for (++it_imu; it_imu != vimu.end(); ++it_imu)
      {
        this->push_back(*it_imu);
      }
    }

    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
      dt_buf.push_back(dt);
      acc_buf.push_back(acc);
      gyr_buf.push_back(gyr);
      propagate(dt, acc, gyr);
    }

    void push_back(const ImuMeasurement6d& imu){
      const double dt = imu.time_stamp - sum_dt - initial_time_stamp;
      this->push_back(dt, imu.a, imu.w);
    }

    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    {
      sum_dt = 0.0;
      acc_0 = linearized_acc;
      gyr_0 = linearized_gyr;
      delta_p.setZero();
      delta_q.setIdentity();
      delta_v.setZero();
      linearized_ba = _linearized_ba;
      linearized_bg = _linearized_bg;
      jacobian.setIdentity();
      covariance.setZero();
      for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
        propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
    }

    void midPointIntegration(double _dt,
                             const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                             const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
      //ROS_INFO("midpoint integration");
      // std::cout << "use midintegration" << std::endl;
      Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
      Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
      result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
      Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
      Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
      result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
      result_delta_v = delta_v + un_acc * _dt;
      result_linearized_ba = linearized_ba;
      result_linearized_bg = linearized_bg;

      if (update_jacobian)
      {
        Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        Vector3d a_0_x = _acc_0 - linearized_ba;
        Vector3d a_1_x = _acc_1 - linearized_ba;
        Matrix3d R_w_x, R_a_0_x, R_a_1_x;

        R_w_x << 0, -w_x(2), w_x(1),
            w_x(2), 0, -w_x(0),
            -w_x(1), w_x(0), 0;
        R_a_0_x << 0, -a_0_x(2), a_0_x(1),
            a_0_x(2), 0, -a_0_x(0),
            -a_0_x(1), a_0_x(0), 0;
        R_a_1_x << 0, -a_1_x(2), a_1_x(1),
            a_1_x(2), 0, -a_1_x(0),
            -a_1_x(1), a_1_x(0), 0;

        MatrixXd F = MatrixXd::Zero(15, 15);
        F.block<3, 3>(0, 0) = Matrix3d::Identity();
        F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                              -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
        F.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * _dt;
        F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
        F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
        F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
        F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3, 3) * _dt;
        F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                              -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
        F.block<3, 3>(6, 6) = Matrix3d::Identity();
        F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
        F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
        F.block<3, 3>(9, 9) = Matrix3d::Identity();
        F.block<3, 3>(12, 12) = Matrix3d::Identity();
        //cout<<"A"<<endl<<A<<endl;

        MatrixXd V = MatrixXd::Zero(15, 18);
        V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * 0.5 * _dt;
        V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
        V.block<3, 3>(3, 3) = 0.5 * MatrixXd::Identity(3, 3) * _dt;
        V.block<3, 3>(3, 9) = 0.5 * MatrixXd::Identity(3, 3) * _dt;
        V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * 0.5 * _dt;
        V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
        V.block<3, 3>(9, 12) = MatrixXd::Identity(3, 3) * _dt;
        V.block<3, 3>(12, 15) = MatrixXd::Identity(3, 3) * _dt;

        //step_jacobian = F;
        //step_V = V;
        jacobian = F * jacobian;
        covariance = F * covariance * F.transpose() + V * noise * V.transpose();
      }
    }

    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
      dt = _dt;
      acc_1 = _acc_1;
      gyr_1 = _gyr_1;
      Vector3d result_delta_p;
      Quaterniond result_delta_q;
      Vector3d result_delta_v;
      Vector3d result_linearized_ba;
      Vector3d result_linearized_bg;

      // if(use_mid_integration)
        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1);
      // else
      //   eulerIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
      //                    linearized_ba, linearized_bg,
      //                    result_delta_p, result_delta_q, result_delta_v,
      //                    result_linearized_ba, result_linearized_bg, 1);

      //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
      //                    linearized_ba, linearized_bg);
      delta_p = result_delta_p;
      delta_q = result_delta_q;
      delta_v = result_delta_v;
      linearized_ba = result_linearized_ba;
      linearized_bg = result_linearized_bg;
      delta_q.normalize();
      sum_dt += dt;
      acc_0 = acc_1;
      gyr_0 = gyr_1;
    }

    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
      Eigen::Matrix<double, 15, 1> residuals;

      // Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
      // Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

      // Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

      // Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
      // Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);
      Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(0, 9);
      Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(0, 12);

      Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(3, 12);

      Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(6, 9);
      Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(6, 12);

      Eigen::Vector3d dba = Bai - linearized_ba;
      Eigen::Vector3d dbg = Bgi - linearized_bg;

      Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
      // Eigen::Quaterniond corrected_delta_q = delta_q * rot_to_quat(dq_dbg * dbg);
      Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
      Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

      residuals.block<3, 1>(0, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
      residuals.block<3, 1>(3, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
      residuals.block<3, 1>(6, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
      residuals.block<3, 1>(9, 0) = Baj - Bai;
      residuals.block<3, 1>(12, 0) = Bgj - Bgi;
      return residuals;
    }

    Eigen::Matrix<double, 15, 1> Evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj){
      return this->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);                                        
    }

    Eigen::Matrix<double, 15, 1> evaluate(double const *const *parameters){
      Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
      Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

      Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
      Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
      Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

      Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
      Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

      Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
      Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
      Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);
      return this->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);
    }

    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;
    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 18, 18> noise;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;

    double initial_time_stamp = 0;
    ImuErrorParameter imuerr;
    const Eigen::Vector3d G = Eigen::Vector3d(0, 0, 9.8);
    // bool use_mid_integration = true;

    // void eulerIntegration(double _dt,
    //                       const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
    //                       const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
    //                       const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
    //                       const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
    //                       Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
    //                       Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    // {
    //   // std::cout << "use eulerintegration" << std::endl;
    //     result_delta_p = delta_p + delta_v * _dt + 0.5 * (delta_q * (_acc_1 - linearized_ba)) * _dt * _dt;
    //     result_delta_v = delta_v + delta_q * (_acc_1 - linearized_ba) * _dt;
    //     Vector3d omg = _gyr_1 - linearized_bg;
    //     omg = omg * _dt / 2;
    //     Quaterniond dR(1, omg(0), omg(1), omg(2));
    //     result_delta_q = (delta_q * dR);   
    //     result_linearized_ba = linearized_ba;
    //     result_linearized_bg = linearized_bg;         

    //     if(update_jacobian)
    //     {
    //         Vector3d w_x = _gyr_1 - linearized_bg;
    //         Vector3d a_x = _acc_1 - linearized_ba;
    //         Matrix3d R_w_x, R_a_x;

    //         R_w_x<<0, -w_x(2), w_x(1),
    //             w_x(2), 0, -w_x(0),
    //             -w_x(1), w_x(0), 0;
    //         R_a_x<<0, -a_x(2), a_x(1),
    //             a_x(2), 0, -a_x(0),
    //             -a_x(1), a_x(0), 0;

    //         MatrixXd A = MatrixXd::Zero(15, 15);
    //         // one step euler 0.5
    //         A.block<3, 3>(0, 3) = 0.5 * (-1 * delta_q.toRotationMatrix()) * R_a_x * _dt;
    //         A.block<3, 3>(0, 6) = MatrixXd::Identity(3,3);
    //         A.block<3, 3>(0, 9) = 0.5 * (-1 * delta_q.toRotationMatrix()) * _dt;
    //         A.block<3, 3>(3, 3) = -R_w_x;
    //         A.block<3, 3>(3, 12) = -1 * MatrixXd::Identity(3,3);
    //         A.block<3, 3>(6, 3) = (-1 * delta_q.toRotationMatrix()) * R_a_x;
    //         A.block<3, 3>(6, 9) = (-1 * delta_q.toRotationMatrix());
    //         //cout<<"A"<<endl<<A<<endl;

    //         MatrixXd U = MatrixXd::Zero(15,12);
    //         U.block<3, 3>(0, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
    //         U.block<3, 3>(3, 3) =  MatrixXd::Identity(3,3);
    //         U.block<3, 3>(6, 0) =  delta_q.toRotationMatrix();
    //         U.block<3, 3>(9, 6) = MatrixXd::Identity(3,3);
    //         U.block<3, 3>(12, 9) = MatrixXd::Identity(3,3);

    //         // put outside
    //         Eigen::Matrix<double, 12, 12> noise = Eigen::Matrix<double, 12, 12>::Zero();
    //         noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    //         noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    //         noise.block<3, 3>(6, 6) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
    //         noise.block<3, 3>(9, 9) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();

    //         //write F directly
    //         MatrixXd F, V;
    //         F = (MatrixXd::Identity(15,15) + _dt * A);
    //         V = _dt * U;
    //         step_jacobian = F;
    //         step_V = V;
    //         jacobian = F * jacobian;
    //         covariance = F * covariance * F.transpose() + V * noise * V.transpose();
    //     }
    // }     
  };
}


#endif