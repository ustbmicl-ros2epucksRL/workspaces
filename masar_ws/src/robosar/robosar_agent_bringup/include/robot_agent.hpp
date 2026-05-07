// Created by Indraneel on 22/01/21

#ifndef ROBOT_AGENT_HPP
#define ROBOT_AGENT_HPP

#include <json/json.h>
#include <string.h>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "async_udp_server.hpp"
#include "ros_feedback_bridge.hpp"
#include "odom_tf.hpp"
#include "robot_status.hpp"
#include <robosar_messages/agents_status.h>
class RobotAgent
{

public:
    ~RobotAgent();
    RobotAgent(const std::string robot_id, const std::string ip_address, const std::string server_ip_addr,
               const std::string path_to_code, const std::string path_to_perception_code, const int feedback_port, const int control_port, 
               const int feedback_freq, const int control_timeout,
               const double deadman_timer_duration, const int freq_calculation_dur, const bool camera_enabled);

    RobotStatus::status_e getAgentStatus();
    void velocityCallback(const geometry_msgs::Twist &vel_msg);
    void timerCallback(const ros::TimerEvent& timer_event);
    void updateAgentStatus(const ros::TimerEvent& timer_event);
    void calculateFrequency(const ros::TimerEvent& timer_event);
    void resetOdometry();
    std::string getAgentStatusString();
    int getActualFrequency();
    int getBatteryLevel();

    const std::string path_to_code_;
    const std::string robot_id_;
    const std::string ip_address_;
    const std::string server_ip_addr_;
    const int feedback_port_;
    const int control_port_;
    const int feedback_freq_hz_;
    const int control_timeout_ms_;
    const int freq_calculation_dur_;
    robosar_messages::agents_status agent_status_;

private:
    boost::asio::io_service io_service;
    boost::asio::io_service::work work;
    ros::NodeHandle nh_;
    std::shared_ptr<ROSFeedbackBridge> bridgePtr;
    udp_server comm_channel_;
    ros::Subscriber control_subscriber_;
    ros::Timer deadman_timer_;
    ros::Timer feedback_timer_;
    ros::Timer freq_calculation_timer_;
    int actual_freq_hz;
    int battery_lvl;
    std::shared_ptr<ros::Timer> timer_ptr_;
    odomTF odom_TF_pub;
    std::string package_path;
    std::shared_ptr<RobotStatus> status_ptr_;
    ros::Publisher agent_status_publisher_;
};

#endif