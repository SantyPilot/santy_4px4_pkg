/*
 * @file: TargetGenerator.cpp
 * @brief: target trajectory generator decl
 * @author: zhangxin
 * @date: 2025-7-10
 */
#include "TargetGenerator.h"
#include <cmath>

#define _USE_MATH_DEFINES
namespace santy_4px4_pkg {
bool TargetGenerator::move(MoveType&, MoveInfo&) { 
    return true; // do nothing
}

// ---
bool CircleTargetGenerator::move(MoveType& mt, MoveInfo& mi) {
    // make target, check target arrived, then generate next one
    return true;
}

std::vector<double> CircleTargetGenerator::circle(
    const size_t& total, const size_t& idx) {
    double angle_in_rad = 2 * M_PI * idx / total;
    double pos_x = radius * std::cos(angle_in_rad);
    double pos_y = radius * std::sin(angle_in_rad);
    return {pos_x, pos_y, height};
}
} // santy_4px4_pkg
