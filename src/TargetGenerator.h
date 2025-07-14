/*
 * @file: TargetGenerator.h
 * @brief: target trajectory generator decl
 * @author: zhangxin
 * @date: 2025-7-10
 */
#ifndef _TARGET_GENERATOR_H
#define _TARGET_GENERATOR_H
#include <vector>
#include <ros/ros.h>
#include "LocalPoseInfo.h"

namespace santy_4px4_pkg {
enum MoveType {
    MT_VFLU = 0,
    MT_VENU,
    MT_PENU // pos in enu
}; // MoveType
struct MoveInfo {
    std::vector<double> pos = {0, 0, 0};
    std::vector<double> vel = {0, 0, 0};
    double yaw = 0;
    double yr = 0;
    MoveType mt = MT_PENU;
};
/*
 * TODO: should support a series of tasks
 */
class TargetGenerator {
public:
    virtual void init(ros::NodeHandle& nh);
    virtual void reset();
    virtual bool move(MoveInfo&);
    virtual bool arrived();
};

// demo circle trajectory generator
class CircleTargetGenerator: public TargetGenerator {
public:
    CircleTargetGenerator();
    ~CircleTargetGenerator();
    virtual void init(ros::NodeHandle& nh) override;
    virtual void reset() override;
    virtual bool move(MoveInfo&) override;
    virtual bool arrived() override;
protected:
    std::vector<double> circle(const size_t& total, 
        const size_t& idx);
private:
    ros::Subscriber _local_pos_sub;
    const double radius = 3; // 3m
    const double height = 3;
    const size_t total = 10;
    int32_t _idx = -1;

    LocalPoseInfo _local_pose;
};
} // santy_4px4_pkg
#endif // _TARGET_GENERATOR_H
