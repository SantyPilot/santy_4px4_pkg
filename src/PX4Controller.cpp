/*
 * @file: PX4Controller.h
 * @brief: px4 mavros control message wrapper decl
 * @author: zhangxin
 * @date: 2025-7-5
 */
#include "PX4Controller.h"
#include <functional>
#include <mutex>
#include <thread>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>

namespace santy_4px4_pkg {
PX4Controller::PX4Controller(): _inited(false), 
    _target_gen {nullptr} {}

PX4Controller::~PX4Controller() {
    // stop all async task
    stopAsyncMoveTask();
}

bool PX4Controller::init(ros::NodeHandle& nh) {
    if (_inited) {
        ROS_INFO("PX4Controller has already been inited!");
        return true;
    }
    // sub
    std::function<void(const mavros_msgs::State::ConstPtr& msg)>
        state_cb = [this](const mavros_msgs::State::ConstPtr& msg) {
        _current_state = *msg; // TODO: not thread safe
    };
    _state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // client
    _arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    _land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    _set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    _command_client = nh.serviceClient<mavros_msgs::CommandLong>
            ("mavros/cmd/command");
    // pub
    /*
     * Note: The setpoint_raw/attitude uses NED as attitude 
     *   while the setpoint_attitude/attitude topic uses ENU
     */
    _set_raw_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10/* queue size */);
    _set_raw_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    _local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10); // simple control
    _inited = true;
    return true;
}

void PX4Controller::setTargetGenerator(TargetGenerator* tg) {
    _target_gen = tg;
}

void PX4Controller::startAsyncMoveTask() {
    std::function<void()> task = [&]() {
        static ros::Time last_cycle = ros::Time::now();
        while (ros::ok() && !_should_exit) {
            // 1. check if not in offboard, switch mode
            offboard();
            // 2. if not armed, do arm
            arm();
            // 3. publish move target message
            if (ros::Time::now() - last_cycle < ros::Duration(1 / 50.0)) {
                continue; // control freq 50hz >> 2hz offboard ddl
            }
            last_cycle = ros::Time::now();
            CtrlMode cm = CtrlMode::CM_VEL;
            ctrlMode(cm); // safe get
            mavros_msgs::PositionTarget ptarget;
            getPTargetBuffer(ptarget); // safe get
            switch (cm) {
                case CtrlMode::CM_VEL:
                    _set_raw_att_pub.publish(ptarget);
                    break;
                case CtrlMode::CM_ATT:
                    _set_raw_pos_pub.publish(ptarget);
                    break;
                default:
                    break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep 10ms
        }
    };
    std::thread t1(task);
    if (t1.joinable()) {
        t1.join();
        ROS_INFO("async thread quit!");
    }
    return;
}

void PX4Controller::stopAsyncMoveTask() {
    _should_exit = false;
}

void PX4Controller::startOffboardMoveCycle() {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    ros::Rate rate(20.0);
    while (ros::ok()) { // cyclly do target calculation
        // 1. prepare if necessary, according to current state
        offboard(); // switch mode
        arm();
        // continuously send offboard command
        _local_pos_pub.publish(pose);
        // calculate task logic

        ros::spinOnce();
        rate.sleep();
    }
}

bool PX4Controller::offboard() {
    return setFlightMode(FlightMode::FM_OFFB);
}

/*
 * Note: directly control with mavros message
 *   there is command long version, force arm
 *   to implement
 */
bool PX4Controller::arm() {
    if (!_inited || !reachRequestInterval() ||
        _current_state.armed) {
        return false;
    }
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if(_arming_client.call(arm_cmd) &&
        arm_cmd.response.success) {
        _last_request = ros::Time::now(); // record timestamp
        ROS_INFO("vehicle armed!");
        return true;
    }
    _last_request = ros::Time::now();
    ROS_WARN("vehicle arm failed!");
    return false;
}

bool PX4Controller::disArm() {
    if (!_inited || !reachRequestInterval() ||
        !_current_state.armed) {
        return false;
    }
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    if(_arming_client.call(arm_cmd) &&
        arm_cmd.response.success) {
        _last_request = ros::Time::now();
        ROS_INFO("vehicle armed");
        return true;
    }
    _last_request = ros::Time::now();
    ROS_WARN("vehicle arm failed!");
    return false;
}

bool PX4Controller::takeoff(const double& height) {
    moveByPosENU({0, 0, height}); // publish not request
    return true;
}

bool PX4Controller::land() {
    if (!_inited || !reachRequestInterval()) {
        return false;
    }
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    if(_land_client.call(land_cmd) &&
        land_cmd.response.success) {
        ROS_INFO("vehicle landed");
        return true;
    }
    ROS_WARN("vehicle land failed!");
    return false;
}

/*
 * Note: reference to: https://github.com/
 *   rfly-zzfly/zzfly-2022-real-share/ 
 *   more precise and fast control strategy
 *   asyncly post ptarget to buffer
 */
void PX4Controller::moveByVelocityYawrateBodyFrame(
    const std::vector<double>& vel, const double& yaw_rate) {
    setCtrlMode(CtrlMode::CM_VEL);
    const auto& ptarget = makeVelTarget(vel, yaw_rate, FrameType::FT_BODY_NED);
    setPTargetBuffer(ptarget);
}

void PX4Controller::moveByVelocityYawrateENU(
    const std::vector<double>& vel, const double& yaw_rate) {
    setCtrlMode(CtrlMode::CM_VEL);
    const auto& ptarget = makeVelTarget(vel, yaw_rate, FrameType::FT_LOCAL_NED);
    setPTargetBuffer(ptarget);
}

void PX4Controller::moveByPosENU(const std::vector<double>& pos,
    const double& yaw, const double& yaw_rate) {
    setCtrlMode(CtrlMode::CM_VEL);
    const auto& ptarget = makePosTarget(pos, yaw, yaw_rate, FrameType::FT_LOCAL_NED);
    setPTargetBuffer(ptarget);
}

// -- protected
bool PX4Controller::setFlightMode(const FlightMode& fm) {
    // http://wiki.ros.org/mavros/CustomModes
    mavros_msgs::SetMode target_set_mode;
    std::string mode_str = "OFFBOARD";
    switch (fm) {
        case FM_OFFB:
            mode_str = "OFFBOARD";
            break;
        default:
            break;
    }
    target_set_mode.request.custom_mode = mode_str;
    if (_current_state.mode != mode_str && reachRequestInterval()) {
        if (_set_mode_client.call(target_set_mode) &&
            target_set_mode.response.mode_sent) {
            ROS_INFO("target flight mode enabled!");
        }
        _last_request = ros::Time::now();
    }
    return true;
}

void PX4Controller::setCtrlMode(const CtrlMode& cm) { 
    std::lock_guard<std::mutex> lk(_cm_mut);
    _ctrl_mode = cm; 
    return;
}

void PX4Controller::ctrlMode(CtrlMode& cm) { 
    std::lock_guard<std::mutex> lk(_cm_mut);
    cm =_ctrl_mode; 
    return;
}

mavros_msgs::PositionTarget PX4Controller::makeVelTarget(
    const std::vector<double>& vel, const double& yaw_rate, const FrameType& ft) {
    mavros_msgs::PositionTarget ptarget;
    uint8_t frame_type = 1;
    switch (ft) {
        case FT_LOCAL_NED: 
            frame_type = 1;
            break;
        case FT_LOCAL_ONED:
            frame_type = 7;
            break;
        case FT_BODY_NED:
            frame_type = 8;
            break;
        case FT_BODY_ONED:
            frame_type = 9;
            break;
        default: // default 1
            break;
    }
    ptarget.header.stamp = ros::Time::now();
    ptarget.coordinate_frame = frame_type;
    if (vel.size() < 3) {
        ptarget.velocity.x = 0;
        ptarget.velocity.y = 0;
        ptarget.velocity.z = 0;
    } else {
        ptarget.velocity.x = vel[0];
        ptarget.velocity.y = vel[1];
        ptarget.velocity.z = vel[2];
    }
    ptarget.type_mask = mavros_msgs::PositionTarget::IGNORE_PX +
                        mavros_msgs::PositionTarget::IGNORE_PY +
                        mavros_msgs::PositionTarget::IGNORE_PZ +
                        mavros_msgs::PositionTarget::IGNORE_AFX +
                        mavros_msgs::PositionTarget::IGNORE_AFY +
                        mavros_msgs::PositionTarget::IGNORE_AFZ +
                        mavros_msgs::PositionTarget::FORCE +
                        mavros_msgs::PositionTarget::IGNORE_YAW;
    ptarget.yaw_rate = yaw_rate;
    return ptarget;
}

mavros_msgs::PositionTarget PX4Controller::makePosTarget(const std::vector<double>& pos, 
    const double& yaw, const double& yaw_rate, const FrameType& ft) {
    uint8_t frame_type = 1;
    switch (ft) {
        case FT_LOCAL_NED: 
            frame_type = 1;
            break;
        case FT_LOCAL_ONED:
            frame_type = 7;
            break;
        case FT_BODY_NED:
            frame_type = 8;
            break;
        case FT_BODY_ONED:
            frame_type = 9;
            break;
        default: // default 1
            break;
    }
    mavros_msgs::PositionTarget ptarget;
    ptarget.header.stamp = ros::Time::now();
    ptarget.coordinate_frame = frame_type;
    if (pos.size() < 3) {
        ptarget.position.x = 0;
        ptarget.position.y = 0;
        ptarget.position.z = 0;
    } else {
        ptarget.position.x = pos[0];
        ptarget.position.y = pos[1];
        ptarget.position.z = pos[2];
    }
    ptarget.type_mask = mavros_msgs::PositionTarget::IGNORE_VX +
                        mavros_msgs::PositionTarget::IGNORE_VY +
                        mavros_msgs::PositionTarget::IGNORE_VZ +
                        mavros_msgs::PositionTarget::IGNORE_AFX +
                        mavros_msgs::PositionTarget::IGNORE_AFY +
                        mavros_msgs::PositionTarget::IGNORE_AFZ +
                        mavros_msgs::PositionTarget::FORCE;
    ptarget.yaw = yaw;
    ptarget.yaw_rate = yaw_rate;
    return ptarget;
}

// TODO: lock is not efficient, should opt this
void PX4Controller::setPTargetBuffer(const mavros_msgs::PositionTarget& ptarget) {
    std::lock_guard<std::mutex> lk(_pt_mut);
    _ptarget_buffer = ptarget;
}

void PX4Controller::getPTargetBuffer(mavros_msgs::PositionTarget& ptarget) {
    std::lock_guard<std::mutex> lk(_pt_mut);
    ptarget = _ptarget_buffer;
}

bool PX4Controller::reachRequestInterval() {
    const double& interval = 5; // 5s
    return (ros::Time::now() - _last_request > ros::Duration(interval));
}

} // santy_4px4_pkg
