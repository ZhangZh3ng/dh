/*
 * @Author: your name
 * @Date: 2021-08-16 14:16:30
 * @LastEditTime: 2021-08-16 19:43:58
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/opt2.cpp
 */
#include <iostream>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include "read.hpp"
#include <string>
#include <vector>

struct NormalResidual
{
    NormalResidual(double x) : x_(x) {}

    template <typename T>
    bool operator()(const T *const x_estimated, T *residual) const
    {
        residual[0] = x_ - x_estimated;
        return true;
    }

private:
    const double x_;
};

struct DistanceResidual
{
    DistanceResidual(double d) : distance_(d) {}

    template <typename T>
    bool operator()(const T *const p_start, const T *const p_end, T *residual) const
    {
        residual[0] = distance_ - (p_end - p_start);
        return true;
    }

private:
    const double distance_;
};

int main(int argc, char **argv)
{
    const std::string file_name;
    const int DATA_ROW = 41900;
    const int DATA_COL = 14;
    std::vector<double> data;
    data = dh::read::numerical_txt_file(argv[1], DATA_ROW, DATA_COL);

    std::vector<double> time_stamp, px, py, pz;
    std::vector<double> px_meas, py_meas, pz_meas;
    std::vector<double> dx, dy, dz;
    std::vector<double> px_init, py_init, pz_init, pos_flag;
    std::vector<double> px_est, py_est, pz_est;

    for (std::vector<double>::iterator it = data.begin(); it != data.end();)
    {
        time_stamp.push_back(*it++);
        px.push_back(*it++);
        py.push_back(*it++);
        pz.push_back(*it++);
        px_meas.push_back(*it++);
        py_meas.push_back(*it++);
        pz_meas.push_back(*it++);
        dx.push_back(*it++);
        dy.push_back(*it++);
        dz.push_back(*it++);
        px_init.push_back(*it++);
        py_init.push_back(*it++);
        pz_init.push_back(*it++);
        pos_flag.push_back(*it++);
    }

    px_est = px_init;
    std::cout << px_est[0] << std::endl;
    std::cout << data.size() << std::endl;

    ceres::Problem problem;

    for (int i = 0; i < time_stamp.size() - 10; i++)
    {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<DistanceResidual, 1, 1>(new DistanceResidual(dx[i])),
                                 NULL,
                                 &px_est[i],
                                 &px_est[i + 1]);

        if (pos_flag[i])
        {
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<NormalResidual, 1, 1>(new NormalResidual(px_est[i])),
                                     NULL,
                                     &px_meas[i]);
        }
    }

    ceres::Solver::Options option;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 20;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    std::cout << "done!" << std::endl;
}