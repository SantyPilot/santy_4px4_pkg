/*
 * @brief: subscribe joystick msg and gen target
 * @file: TGJoystick.h
 * @author: zhangxin
 * @date: 2025-7-14
 */
#ifndef _TG_JOY_STICK_H
#define _TG_JOY_STICK_H
#include "TargetGenerator.h"

namespace santy_4px4_pkg {
enum JS_AXIS_INDEX {
    LSTICK_LEFT = 0, // 0, 1
    LSTICK_RIGHT,    // 0, -1
    LSTICK_FRONT,
    LSTICK_BACK,
    L2BUT_STILL,
    L2BUT_PUSH,
    RSTICK_LEFT,
    RSTICK_RIGHT,
    RSTICK_FRONT,
    RSTICK_BACK,
    R2BUT_STILL,
    R2BUT_PUSH,
    LBUT_PUSH, // left hand side button, still 0, push 1
    RBUT_PUSH,
    FBUT_PUSH,
    BACT_BUT_PUSH
}; // JS_AXIX_INDEX
enum JS_BUTTON_INDEX {
    ABUT_PUSH = 0, // down 1
    BBUT_PUSH,
    XBUT_PUSH,
    YBUT_PUSH,
    L1BUT_PUSH,
    R1BUT_PUSH,
    SELECT_BUT,
    START_BUT,
    MODE_BUT,
    LSTICK_BUT,
    RSTICK_BUT,
}; // JS_BUTTON_INDEX
enum ActionType { // reserved
    AT_ARM = 0,
    AT_DISARM,
    AT_TAKEOFF,
    AT_LAND,
    AT_FLIGHT,
    AT_PARAM_TUNING,
    AT_STOP,
}; // ActionType
struct JSCtrlInfo {
    std::vector<double> move = {0, 0, 0, 0}; // l-r, f-b, yl-yr, u-d
    ActionType atype = AT_FLIGHT;
}; // JSCtrlInfo
class TGJoystick: public TargetGenerator {
public:
    virtual void init(ros::NodeHandle& nh) override;
    virtual void reset() override;
    // PX4Controller only opened limited ability
    virtual bool move(MoveInfo&) override;
    virtual bool arrived() override;
private:
    JSCtrlInfo _jsc_info;
    ros::Subscriber _joystick_sub;
    const double total_vel = 3; // 3m/s
}; // TGJoystick
} // santy_4px4_pkg
#endif // _TG_JOY_STICK_H
