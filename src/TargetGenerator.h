/*
 * @file: TargetGenerator.h
 * @brief: target trajectory generator decl
 * @author: zhangxin
 * @date: 2025-7-10
 */
#ifndef _TARGET_GENERATOR_H
#define _TARGET_GENERATOR_H
#include <vector>
namespace santy_4px4_pkg {
enum MoveType {
    MT_VFLU = 0,
    MT_VENU,
    MT_PENU
}; // MoveType
struct MoveInfo {
    std::vector<double> pos = {0, 0, 0};
    std::vector<double> vel = {0, 0, 0};
    double yaw = 0;
    double yr = 0;
};
/*
 * TODO: should support a series of tasks
 */
class TargetGenerator {
public:
    virtual bool move(MoveType&, MoveInfo&);
};

// demo circle trajectory generator
class CircleTargetGenerator: public TargetGenerator {
public:
    virtual bool move(MoveType&, MoveInfo&);
protected:
    std::vector<double> circle(const size_t& total, 
        const size_t& idx);
private:
    const double radius = 3; // 3m
    const double height = 3;
};
} // santy_4px4_pkg
#endif // _TARGET_GENERATOR_H
