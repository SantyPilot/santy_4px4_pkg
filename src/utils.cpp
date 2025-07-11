/*
 * @file: utils.cpp
 * @brief: utility tools
 * @author: zhangxin
 * @date: 2025-7-11
 */
#include "utils.h"
#include <cmath>

namespace santy_4px4_pkg {
    std::vector<double> 
        Utils::Quant2RPY(const std::vector<double>& q) {
        std::vector<double> rpy = {0, 0, 0};
        rpy[0] = atan2(2 * (q[0]*q[1] + q[2]*q[3]), 
                1 - 2 * (q[1]*q[1] + q[2]*q[2]));
        rpy[1] = asin(2 * (q[0] * q[2] - q[3] * q[1]));
        rpy[2] = atan2(2 * (q[0]*q[3] + q[1]*q[2]), 
                1 - 2 * (q[2]*q[2] + q[3]*q[3]));
        return rpy;
    }

    std::vector<double> 
        Utils::Quant2RotMat(const std::vector<double>& q) {
        std::vector<double> rot_mat;
        rot_mat.resize(9);
        auto& w = q[0]; 
        auto& x = q[1];
        auto& y = q[2];
        auto& z = q[3];
        rot_mat = { 1-2*y*y - 2*z*z, 2*x*y - 2*w*z,     2*x*z + 2*w*y,
                    2*x*y + 2*w*z,   1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x,
                    2*x*z - 2*w*y,   2*y*z + 2*w*x,     1 - 2*x*x - 2*y*y};
        return rot_mat;
    }

    double Utils::distance(const std::vector<double>& p1,
        const std::vector<double>& p2) {
        double dist = 0;
        if (p1.size() != 3 || p2.size() != 3) {
            return dist;
        }
        dist = pow(p1[0] - p2[0], 2) +
               pow(p1[1] - p2[1], 2) +
               pow(p1[2] - p2[2], 2);
        return dist;
    }
} // santy_4px4_pkg
