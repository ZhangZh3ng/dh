/*
 * @Author: your name
 * @Date: 2021-08-15 10:24:21
 * @LastEditTime: 2021-08-15 16:00:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/test/readfile.cpp
 */

#include <iostream>
#include <fstream>
#include <queue>
#include <chrono>

int main(int argc, char **argv)
{
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    std::cout << "hello" << std::endl;
    if (argc != 3)
    {
        std::cout << "You must input two argrments, in which the former is reference file the other is measurement file" << std::endl;
        return 0;
    }

    // Designate input file
    std::ifstream ref_file;
    std::ifstream meas_file;

    ref_file.open(argv[1], std::ios::in);
    if (!ref_file.is_open())
    {
        std::cout << "reference data file open failed" << std::endl;
        return 0;
    }

    meas_file.open(argv[2], std::ios::in);
    if (!meas_file.is_open())
    {
        std::cout << "measurment data file open failed" << std::endl;
        return 0;
    }

    const int ref_row = 41901;
    const int ref_col = 7;
    const int meas_row = 41901;
    const int meas_col = 8;

    double data;
    double ref_data[ref_row][ref_col];
    double meas_data[meas_row][meas_col];

    int row = 0;
    int col = 0;

    while (ref_file >> data)
    {
        ref_data[row][col] = data;
        col += 1;
        if (col >= ref_col)
        {
            col = 0;
            row += 1;
        }
    }

    row = 0;
    col = 0;

    while (meas_file >> data)
    {
        meas_data[row][col] = data;
        col += 1;
        if (col >= meas_col)
        {
            col = 0;
            row += 1;
        }
    }

    ref_file.close();
    meas_file.close();
    std::cout << "read successfully." << std::endl;
    std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> t_used = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
    std::cout << "total time: " << t_used.count() << std::endl;
    return 0;
}
