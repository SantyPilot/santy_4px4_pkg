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

#define _USE_MATH_DEFINES // M_PI

namespace santy_4px4_pkg {
void TargetGenerator::init(ros::NodeHandle& nh) {}

void TargetGenerator::reset() {}

bool TargetGenerator::move(MoveInfo&) { return true; }

bool TargetGenerator::arrived() {
    return true;
}

// ---
CircleTargetGenerator::CircleTargetGenerator(): _idx(-1) {}

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

void CircleTargetGenerator::reset() {
    _idx = 0;
}

bool CircleTargetGenerator::move(MoveInfo& mi) {
    _idx++;
    const auto& pos_yaw = circle(total, _idx);
    mi.pos = { pos_yaw[0], pos_yaw[1], pos_yaw[2] };
    mi.yaw = pos_yaw[3];
    mi.mt = MoveType::MT_PENU;
    // ROS_INFO_STREAM("circle task move to next target, pos " <<
    //    mi.pos[0] << ", " << mi.pos[1] << ", " << mi.pos[2]);
    if (_idx > total) {
        reset();
    }
    return true; // exceed, just finish
}

bool CircleTargetGenerator::arrived() {
    if (_idx == -1) {
        return true;
    }
    const auto& pos_yaw = circle(total, _idx); // current target
    const std::vector<double>& pos = { pos_yaw[0], pos_yaw[1], pos_yaw[2] };
    const double& eps = 0.1; // m3
    /*ROS_INFO_STREAM("current position is " <<
        _local_pose.pos[0] << ", " << _local_pose.pos[1]
        << ", " << _local_pose.pos[2]);
        */
    return (Utils::distance(_local_pose.pos, pos) < eps);
}

std::vector<double> CircleTargetGenerator::circle(
    const size_t& total, const size_t& idx) {
    double angle_in_rad = 2 * M_PI * idx / total;
    double pos_x = radius * std::cos(angle_in_rad);
    double pos_y = radius * std::sin(angle_in_rad);
    double yaw = M_PI / 2 + 2 * M_PI * idx / total;
    if (yaw > M_PI) {
        yaw = yaw - 2 * M_PI;
    }
    return {pos_x, pos_y, height, yaw};
}
} // santy_4px4_pkg
