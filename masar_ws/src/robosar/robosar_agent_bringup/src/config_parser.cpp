// Created by Indraneel on 22/01/21

#include "config_parser.hpp"
#include <ros/package.h>
#include <assert.h>
#include <memory.h>
#include <ros/console.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include "robot_status.hpp"
#include "easylogging++.h"
#include "robosar_messages/agents_status.h"

bool ConfigParser::is_initialized_ = false;
INITIALIZE_EASYLOGGINGPP

ConfigParser::ConfigParser() : nh("~"), simulation_flag(false)
{

    // Get path to config file
    std::string package_path = ros::package::getPath("robosar_agent_bringup");
    std::string config_file_path_ = package_path + "/config/user_config.json";

    ROS_INFO("Looking for config file in path %s", &config_file_path_[0]);

    Json::Reader reader;
    Json::Value config;
    std::filebuf fb;
    bool parsing_status = false;

    if (fb.open(config_file_path_, std::ios::in))
    {
        std::istream config_stream(&fb);
        parsing_status = reader.parse(config_stream, config, false);
    }

    // Check parsing result
    assert(parsing_status);

    ROS_INFO("Config file parsed successfully, chosen config : %s", &config["config_selection"].asString()[0]);

    configSystemInit(config);

    sh = nh.advertiseService("agent_status", &ConfigParser::pubAgentInfo, this);
    shOdom = nh.advertiseService("sys_odom_reset", &ConfigParser::resetAgentsOdom, this);
    agent_status_pub_ = nh.advertise<robosar_messages::agents_status>("all_agent_status", 1);

    if(!config["simulation"].asBool()){
    feedback_timer_ = nh.createTimer(ros::Duration(config["update_status_dur"].asInt()),boost::bind(&ConfigParser::publishAgentStatus, this, _1));
    }

}

/**
 * @brief initialises system based on the user config file
 * 
 * @param config user config file
 * @return void
 * 
 */

void ConfigParser::configSystemInit(Json::Value config)
{

    std::string path_to_khepera_code = config["path_to_khepera_code"].asString();
    std::string path_to_perception_code = config["path_to_perception_code"].asString();
    std::string server_ip_add = config["server_ip_address"].asString();

    simulation_flag = config["simulation"].asBool();
    ROS_INFO("Simulation Flag: %s", simulation_flag ? "true" : "false");
    if(simulation_flag)
        ROS_WARN("Simulation flag is active!");
    else
    {
        // Hardware mode still relies on the external template directories.
        struct stat info;
        assert(stat(&path_to_khepera_code[0], &info) == 0);
        assert(stat(&path_to_perception_code[0], &info) == 0);
    }
    config = config[config["config_selection"].asString()];

    // Iterate through all agents
    int it = 0;
    for (const auto &itr : config.getMemberNames())
    {

        it++;
        Json::Value agent_config;
        agent_config = config[itr];

        // Create new Agent
        std::string agent_name = itr;
        if(agent_names.find(itr)==agent_names.end())
            agent_names.insert(itr);
        else
            agent_name =  "agent" + std::to_string(it);
        
        if(true)
        {
            ROS_INFO("111");
            std::shared_ptr<RobotAgent> agentPtr(nullptr);
            agents_vec.push_back(agentPtr);
            continue;
        }
        assert(agent_config["freq_calculation_duration"].asInt() != 0);
        std::shared_ptr<RobotAgent> agentPtr(new RobotAgent(agent_name,
                                                            agent_config["ip_address"].asString(),
                                                            server_ip_add,
                                                            path_to_khepera_code,
                                                            path_to_perception_code,
                                                            agent_config["feedback_port"].asInt(),
                                                            agent_config["control_port"].asInt(),
                                                            agent_config["feedback_freq_hz"].asInt(),
                                                            agent_config["control_timeout_ms"].asInt(),
                                                            agent_config["deadman_timer_s"].asDouble(),
                                                            agent_config["freq_calculation_duration"].asInt(),
                                                            agent_config["camera_enabled"].asBool()));
        // Adding all agents (not only the ones alive) to the vector
        all_agents_vec.push_back(agentPtr);
        
        // Check if agent is alive
        if (agentPtr->getAgentStatus() == RobotStatus::ROBOT_STATUS_ACTIVE)
        {
            // Add it to vector
            agents_vec.push_back(agentPtr);
        }
        //print agent status for awareness
        ROS_INFO("%s\n", &(agent_name + " :" + agentPtr->getAgentStatusString())[0]);
        
    }

    ROS_INFO("222");
    ROS_INFO("Number of agents online : %ld/%d\n", agents_vec.size(), it);
    
}

bool ConfigParser::pubAgentInfo(robosar_messages::agent_status::Request  &req, robosar_messages::agent_status::Response &res)
{
    std::vector<std::string> status;
    if(simulation_flag)
    {
        for (auto itr: agent_names)
            status.push_back(itr);
        res.agents_active = status;
        return true;
    }
    
    try
    {
        for (auto agent : agents_vec) {
            
            if(agent->getAgentStatus()==RobotStatus::ROBOT_STATUS_ACTIVE)
            {
                status.push_back(agent->robot_id_);
            }
        }
        res.agents_active = status;
        return true;
    }
    catch(const std::exception& e)
    {
        std::cout<<e.what();
        return false;
    }
}

bool ConfigParser::resetAgentsOdom(robosar_messages::sys_odom_reset::Request  &req, robosar_messages::sys_odom_reset::Response &res)
{
    // Reset odometry for each agent
    try
    {
        for (auto agent : agents_vec) {
            
            agent->resetOdometry();
        }
        return true;
    }
    catch(const std::exception& e)
    {
        std::cout<<e.what();
        return false;
    }
}

void ConfigParser::publishAgentStatus(const ros::TimerEvent& timer_event) {
    if(all_agents_vec.empty()) {
        ROS_WARN("No agents available to publish status");
        return;
    }
    std::vector<std::string> all_robot_id;
    std::vector<std::string> all_ip;
    std::vector<int> all_battery_lvl;
    std::vector<int> all_feedback_freq;
    std::vector<std::string> all_status;
    robosar_messages::agents_status all_agents_status;
    for (auto agent : all_agents_vec) {
        all_robot_id.push_back(agent->robot_id_);
        all_ip.push_back(agent->ip_address_);
        all_battery_lvl.push_back(agent->getBatteryLevel());
        all_feedback_freq.push_back(agent->getActualFrequency());
        all_status.push_back(agent->getAgentStatusString());
    }
    all_agents_status.robot_id = all_robot_id;
    all_agents_status.ip_adress = all_ip;
    all_agents_status.battery_lvl = all_battery_lvl;
    all_agents_status.feedback_freq = all_feedback_freq;
    all_agents_status.status = all_status;
    agent_status_pub_.publish(all_agents_status);
}
