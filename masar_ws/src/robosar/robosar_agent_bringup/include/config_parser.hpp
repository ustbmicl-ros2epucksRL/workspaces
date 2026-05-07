// Created by Indraneel on 22/01/21

#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <json/json.h>
#include "robot_agent.hpp"
#include <memory>
#include "robosar_messages/agent_status.h"
#include "robosar_messages/sys_odom_reset.h"
#include "robosar_messages/agents_status.h"
class ConfigParser
{

public:
    ~ConfigParser() = default;
    ConfigParser();

    std::vector<std::shared_ptr<RobotAgent>> agents_vec;
    // Creating another vector as need to maintain all agents not just the ones alive
    std::vector<std::shared_ptr<RobotAgent>> all_agents_vec;
    void publishAgentStatus(const ros::TimerEvent& timer_event);
    bool pubAgentInfo(robosar_messages::agent_status::Request  &req,
                     robosar_messages::agent_status::Response &res); 
    bool resetAgentsOdom(robosar_messages::sys_odom_reset::Request  &req,
                     robosar_messages::sys_odom_reset::Response &res); 

private:
    static bool is_initialized_;
    bool simulation_flag;
    std::set<std::string> agent_names;

    void configSystemInit(Json::Value config);
    ros::NodeHandle nh;
    ros::ServiceServer sh; // Status service
    ros::ServiceServer shOdom; // Odom reset service
    ros::Publisher agent_status_pub_;
    ros::Timer feedback_timer_;
};

#endif