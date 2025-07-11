/*
 * @file: LocalPoseInfo.h
 * @brief: used to record vehicle state
 * @author: zhangxin
 * @date: 2025-7-11
 */
#ifndef _LOCAL_POSE_INFO_H
#define _LOCAL_POSE_INFO_H
namespace santy_4px4_pkg {
struct LocalPoseInfo {
    bool pos_inited = false;
    std::vector<double> pos = {0, 0, 0};
    std::vector<double> pos_offset = {0, 0, 0};
    double yaw_offset = 0;
    std::vector<double> q; // attitude
    std::vector<double> rpy = {0, 0, 0}; // redudent
};
} // santy_4px4_pkg
#endif // _LOCAL_POSE_INFO_H
