/*
 * @file: utils.h
 * @brief: utility tools
 * @author: zhangxin
 * @date: 2025-7-11
 */
#ifndef _SANTY_4PX4_PKG_UTILS_H
#define _SANTY_4PX4_PKG_UTILS_H
#include <vector>

namespace santy_4px4_pkg {
class Utils {
public:
    static std::vector<double>
        Quant2RPY(const std::vector<double>& q);

    static std::vector<double> 
        Quant2RotMat(const std::vector<double>& rpy);

    static double distance(const std::vector<double>& p1,
        const std::vector<double>& p2);
}; // Utils
} // santy_4px4_pkg
#endif // _SANTY_4PX4_PKG_UTILS_H
