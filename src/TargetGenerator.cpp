/*
 * @file: TargetGenerator.cpp
 * @brief: target trajectory generator decl
 * @author: zhangxin
 * @date: 2025-7-10
 */
#include "TargetGenerator.h"
#include "utils.h"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

#define _USE_MATH_DEFINES
namespace santy_4px4_pkg {
void TargetGenerator::init(ros::NodeHandle& nh) {}

void TargetGenerator::move(MoveInfo&) {}

bool TargetGenerator::arrived() {
    return true;
}

// ---
CircleTargetGenerator::CircleTargetGenerator(): _idx(0) {}

void CircleTargetGenerator::init(ros::NodeHandle& nh) {
    std::function<void(const geometry_msgs::PoseStamped::ConstPtr& msg)>
        pose_cb = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        _local_pose.pos_inited = true;
        _local_pose.pos = { msg->pose.position.x - _local_pose.pos_offset[0], 
                            msg->pose.position.y - _local_pose.pos_offset[1], 
                            msg->pose.position.z - _local_pose.pos_offset[2] };
        _local_pose.q = { msg->pose.orientation.w, 
                          msg->pose.orientation.x,
                          msg->pose.orientation.y,
                          msg->pose.orientation.z};
        _local_pose.rpy = Utils::Quant2RPY(_local_pose.q);
    };
    _local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
}

void CircleTargetGenerator::move(MoveInfo& mi) {
    _idx++;
    const auto& pos = circle(total, _idx);
    mi.pos = pos;
    mi.mt = MoveType::MT_PENU;
    return;
}

bool CircleTargetGenerator::arrived() {
    const auto& pos = circle(total, _idx); // current target
    const double& eps = 0.1; // m3
    return (Utils::distance(_local_pose.pos, pos) < eps);
}

std::vector<double> CircleTargetGenerator::circle(
    const size_t& total, const size_t& idx) {
    double angle_in_rad = 2 * M_PI * idx / total;
    double pos_x = radius * std::cos(angle_in_rad);
    double pos_y = radius * std::sin(angle_in_rad);
    return {pos_x, pos_y, height};
}
} // santy_4px4_pkg
