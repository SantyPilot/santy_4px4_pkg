/*
 * @file: PX4Controller.h
 * @brief: px4 mavros control message wrapper
 * @author: zhangxin
 * @date: 2025-7-5
 */
#ifndef _PX4_CONTROLLER_H
#define _PX4_CONTROLLER_H
#include <mutex>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include "LocalPoseInfo.h"

namespace santy_4px4_pkg {
enum FlightMode {
    FM_OFFB = 0,
    FM_RTL, // return to launch
    FM_PHOLD,
    FM_LAND,
};
enum CtrlMode {
    CM_VEL = 0,
    CM_ATT,
};
enum FrameType {
    FT_LOCAL_NED = 0,
    FT_LOCAL_ONED,
    FT_BODY_NED,
    FT_BODY_ONED,
};
class TargetGenerator;
class PX4Controller {
public:
   PX4Controller();
   ~PX4Controller();
   bool init(ros::NodeHandle& nh);
   void setTargets(const std::vector<TargetGenerator*>&);
   /*
    * Note: async publish target task
    *   PX4 has a timeout of 500ms 
    *   between two Offboard commands. 
    *   If this timeout is exceeded, 
    *   the commander will fall back to 
    *   the last mode the vehicle was 
    *   in before entering Offboard mode.
    */
   void startAsyncMoveTask();
   void stopAsyncMoveTask();
   void startOffboardMoveCycle();
   bool offboard(); // switch to offboard mode
   // basic interface to controller quadcopter
   bool arm();
   bool forceArm();
   bool disArm();
   bool takeoff(const double& vz, const double& height);
   bool land();

   void moveByVelocityYawrateBodyFrame(const std::vector<double>& vel = {0, 0, 0}, 
        const double& yaw_rate = 0);
   void moveByVelocityYawrateENU(const std::vector<double>& vel = {0, 0, 0},
        const double& yaw_rate = 0);
   void moveByPosENU(const std::vector<double>& pos = {0, 0, 0}, 
        const double& yaw = 0, const double& yaw_rate = 0);
    bool moveTargetAboutToArrive(const double&);

protected:
   bool setFlightMode(const FlightMode&);
   void setCtrlMode(const CtrlMode& cm);
   void ctrlMode(CtrlMode& cm);
   mavros_msgs::PositionTarget makeVelTarget(const std::vector<double>& vel = {0, 0, 0}, 
        const double& yaw_rate = 0, const FrameType& = FrameType::FT_BODY_NED);
   mavros_msgs::PositionTarget makePosTarget(const std::vector<double>& pos, 
        const double& yaw, const double& yaw_rate, const FrameType&);
   void setPTargetBuffer(const mavros_msgs::PositionTarget& ptarget);
   void getPTargetBuffer(mavros_msgs::PositionTarget& ptarget);
   bool reachRequestInterval();
   void publishTargetCmd();
private:
    bool _inited { false };
    mavros_msgs::State _current_state; // vehicle current state

    ros::Subscriber _state_sub;
    ros::Subscriber _local_pos_sub;
    ros::Publisher _local_pos_pub;
    ros::Publisher _set_raw_att_pub;
    ros::Publisher _set_raw_pos_pub;
    ros::ServiceClient _arming_client;
    ros::ServiceClient _land_client;
    ros::ServiceClient _set_mode_client;
    ros::ServiceClient _command_client;

    CtrlMode _ctrl_mode;
    ros::Time _last_request;
    mavros_msgs::PositionTarget _ptarget_buffer;
    std::atomic<bool> _should_exit { true };
    LocalPoseInfo _local_pose;
    // deprecate, use sync cycle
    std::mutex _pt_mut;
    std::mutex _cm_mut;

    std::vector<TargetGenerator*> _target_list;

    const double height = 1; // 2m
};
} // santy_4px4_pkg
#endif // _PX4_CONTROLLER_H
