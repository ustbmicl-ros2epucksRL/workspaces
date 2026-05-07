#include "robot_status.hpp"
#include "robot_agent.hpp"
#include "std_msgs/Bool.h"

RobotStatus::RobotStatus():nh("~"),status_(ROBOT_STATUS_ACTIVE) {
    //status_ = ROBOT_STATUS_ACTIVE;
    status_pub = nh.advertise<std_msgs::Bool>("status", 1000);
}


/**
 * @brief Returns agent status
 * 
 * @param void
 * @return status_e
 * 
 */
RobotStatus::status_e RobotStatus::getStatus() {
    std::lock_guard<std::mutex> lock(mutex);
    return status_;
}


void RobotStatus::setStatus(status_e new_status) {
    std::lock_guard<std::mutex> lock(mutex);
    status_ = new_status;
    
    std_msgs::Bool msg;
    msg.data = (new_status==RobotStatus::ROBOT_STATUS_NO_HEARTBEAT ? false : true);
    status_pub.publish(msg);
}