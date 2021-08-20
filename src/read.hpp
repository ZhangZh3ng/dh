/*
 * @Author: your name
 * @Date: 2021-08-15 16:04:31
 * @LastEditTime: 2021-08-16 16:31:47
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /dh/src/read.hpp
 */

#ifndef DH_READ_H
#define DH_READ_H

#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>

namespace dh
{
    namespace read
    {
        double *double_txt_file(const std::string file_path, const int row, const int col)
        {
            double *data = new double[row * col];

            std::ifstream file;
            file.open(file_path, std::ios::in);
            if (!file.is_open())
            {
                std::cout << "input file open failed" << std::endl;
                return 0;
            }
            int i = 0;
            while (file >> data[i++])
            {
            }
            file.close();
            return data;
        }

        std::vector<double> numerical_txt_file(const std::string file_path, const int row, const int col)
        {
            std::vector<double> data;

            std::ifstream file;
            file.open(file_path, std::ios::in);
            if (!file.is_open())
            {
                std::cout << "input file open failed" << std::endl;
                throw 1;
            }

            double msg;
            while (file >> msg)
            {
                data.push_back(msg);
            }
            file.close();

            return data;
        }

    }
}

#endif
