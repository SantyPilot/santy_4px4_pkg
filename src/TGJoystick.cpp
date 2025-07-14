/*
 * @brief: subscribe joystick msg and gen target
 * @file: TGJoystick.cpp
 * @author: zhangxin
 * @date: 2025-7-14
 */
#include "TGJoystick.h"
#include <sensor_msgs/Joy.h>

namespace santy_4px4_pkg {
void TGJoystick::init(ros::NodeHandle& nh) {
    std::function<void(const sensor_msgs::Joy::ConstPtr& msg)>
        js_cb = [this](const sensor_msgs::Joy::ConstPtr& msg) {
        // left stick, right stick msg
        const size_t& lsl_idx = (size_t)JS_AXIS_INDEX::LSTICK_LEFT / 2;
        const size_t& lsf_idx = (size_t)JS_AXIS_INDEX::LSTICK_FRONT / 2;
        const size_t& rsl_idx = (size_t)JS_AXIS_INDEX::RSTICK_LEFT / 2;
        const size_t& rsf_idx = (size_t)JS_AXIS_INDEX::RSTICK_FRONT / 2;
        double ylyr_val = msg->axes[lsl_idx]; // 1 -> -1
        double fb_val = msg->axes[lsf_idx];
        double lr_val = msg->axes[rsl_idx];
        double ud_val = msg->axes[rsf_idx];
        /*
        ROS_INFO_STREAM("left right ctrl value: " << lr_val
                << ", front back ctrl value: " << fb_val
                << ", yaw left yaw ctrl right value: " << ylyr_val
                << ", up down ctrl value: " << ud_val);
                */
        _jsc_info.move = {lr_val, fb_val, ud_val, ylyr_val};

        // stop task
        const size_t& stop_task_idx = (size_t)JS_BUTTON_INDEX::ABUT_PUSH / 2;
        bool stop_flight = (msg->buttons[stop_task_idx] > 0);
        _jsc_info.atype = stop_flight ? ActionType::AT_STOP: ActionType::AT_FLIGHT;
    };
    _joystick_sub = nh.subscribe<sensor_msgs::Joy>
            ("/joy", 10, js_cb);
}

void TGJoystick::reset() {
    _jsc_info.atype = AT_FLIGHT;
}

bool TGJoystick::move(MoveInfo& mi) {
    if (_jsc_info.atype == AT_STOP) {
        return false;
    }
    mi.mt = MoveType::MT_VFLU;
    mi.vel = {
        _jsc_info.move[1], // f
        _jsc_info.move[0], // l
        _jsc_info.move[2] // u
    };
    mi.yr = _jsc_info.move[3]; // yaw but actually yaw rate
    return true; 
}

bool TGJoystick::arrived() {
    // should delay?
    return true;
}
} // santy_4px4_pkg
