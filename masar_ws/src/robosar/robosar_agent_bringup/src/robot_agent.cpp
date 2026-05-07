// Created by Indraneel on 22/01/21

#include "robot_agent.hpp"
#include "ssh_session.hpp"
#include <ros/console.h>
#include <memory>
#include <ros/package.h>
#include "boost/bind.hpp"
#include <boost/thread.hpp>
#include <robosar_messages/agents_status.h>
RobotAgent::RobotAgent(const std::string robot_id, const std::string ip_address, const std::string server_ip_addr,
                       const std::string path_to_code, const std::string path_to_perception_code, const int feedback_port, const int control_port, 
                       const int feedback_freq, const int control_timeout, const double deadman_timer_duration, const int freq_calculation_dur, const bool camera_enabled) :

                        robot_id_(robot_id), ip_address_(ip_address), server_ip_addr_(server_ip_addr),bridgePtr(std::make_shared<ROSFeedbackBridge>(robot_id,nh_,feedback_freq)),
                        path_to_code_(path_to_code), feedback_port_(feedback_port), control_port_(control_port), control_timeout_ms_(control_timeout),
                        feedback_freq_hz_(feedback_freq), nh_("~" + robot_id), comm_channel_(robot_id, io_service, feedback_port, ip_address, control_port, bridgePtr), work(io_service),
                        odom_TF_pub(nh_, robot_id), freq_calculation_dur_(freq_calculation_dur)
{

    // @indraneel initialise communication
    status_ptr_ = std::make_shared<RobotStatus>(); 
    comm_channel_.status_ptr_ = status_ptr_;

    // Set up khepera robot
    std::shared_ptr<SSHSession> sessionPtr(new SSHSession(ip_address_));
    if (!sessionPtr->initiateConnection())
    {
        status_ptr_->setStatus(RobotStatus::ROBOT_STATUS_UNREACHABLE);
        ROS_WARN("Could not reach %s", &robot_id_[0]);
        return;
    }

    if(camera_enabled) {
        struct stat info;
        assert(stat(&path_to_perception_code[0], &info) == 0);
    }

    // Get path to config file
    package_path = ros::package::getPath("robosar_agent_bringup");
    std::string shell = package_path + "/script/khepera_setup.sh";
    // Run setup script
    std::system(&(shell + " " + path_to_code_ + " " + ip_address_ + " " + server_ip_addr_ +
                  " " + std::to_string(feedback_port_) + " " + std::to_string(control_port_) + " " + 
                  std::to_string(feedback_freq_hz_) + " " + std::to_string(control_timeout_ms_) +
                  " " +  std::to_string(camera_enabled) +" " + path_to_perception_code)[0]);

    deadman_timer_ = nh_.createTimer(ros::Duration(deadman_timer_duration), boost::bind(&RobotAgent::timerCallback, this, _1));
    freq_calculation_timer_ = nh_.createTimer(ros::Duration(freq_calculation_dur_),boost::bind(&RobotAgent::calculateFrequency, this, _1));
    // Timer only starts once agent is up
    deadman_timer_.stop();
    timer_ptr_ = std::make_shared<ros::Timer>(deadman_timer_);
    comm_channel_.deadman_timer_ptr_  = timer_ptr_;


    // Create io_service background thread for udp server
    try
    {
        boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Exception: %s\n", e.what());
        status_ptr_->setStatus(RobotStatus::ROBOT_STATUS_COMM_FAIL);
        return;
    }

    // Create ROS nodes for this agent
    control_subscriber_ = nh_.subscribe("cmd_vel", 1, &RobotAgent::velocityCallback, this);
}

RobotAgent::~RobotAgent() {
    std::cout<<"Killing :"<<&robot_id_[0]<<std::endl;
    
    // Check if robot is reachable
    RobotStatus::status_e myCurStatus = getAgentStatus();
    if(myCurStatus!=RobotStatus::ROBOT_STATUS_NO_HEARTBEAT &&
        myCurStatus!=RobotStatus::ROBOT_STATUS_UNREACHABLE)
    {
        // Cleanly exit the agent side software
        std::string shell = package_path + "/script/khepera_setdown.sh";
        std::system(&(shell + " " + ip_address_)[0]);
    }
    else
    {
        std::cout<<"Could not kill agent cleanly!"<<std::endl;
    }
    io_service.stop();
}

/**
 * @brief Returns agent status
 * 
 * @param void
 * @return status_e
 * 
 */
RobotStatus::status_e RobotAgent::getAgentStatus()
{
    return status_ptr_->getStatus();
}

std::string RobotAgent::getAgentStatusString() {
    return status_ptr_->robotStatusStrVec[status_ptr_->getStatus()];
}


/**
 * @brief Receives data from ROS and then sends it to robot using UDP
 * 
 * @param vel_msg twist message from ROS
 * @return void
 * 
 */
void RobotAgent::velocityCallback(const geometry_msgs::Twist &vel_msg)
{
    ROS_DEBUG("Sending velocity v:%f w:%f to %s!\n", vel_msg.linear.x, vel_msg.angular.z, &robot_id_[0]);
    // Convert m/s to mm/s because khepera code needs it in mm/s
    std::string command_msg = std::to_string(vel_msg.angular.z) + "x" + std::to_string(vel_msg.linear.x*1000.0);

    memcpy(comm_channel_.send_data_, command_msg.data(), command_msg.length());
    //async send
    comm_channel_.do_send(command_msg.length());
}

/**
 * @brief Called once deadman timer expires
 * 
 * @return void
 * 
 */
void RobotAgent::timerCallback(const ros::TimerEvent& timer_event) {
    status_ptr_->setStatus(RobotStatus::ROBOT_STATUS_NO_HEARTBEAT);
    ROS_WARN("%s: NO HEARTBEAT", &robot_id_[0]);
    timer_ptr_->stop();
}

/**
 * @brief Calculates the actual feedback frequency
 *
 * @return void
 *
 */
void RobotAgent::calculateFrequency(const ros::TimerEvent& timer_event) {
    this->actual_freq_hz = bridgePtr->getMessageCounter()/freq_calculation_dur_;

    //Battery Level wont change much, hence sufficient to update it at low freq
    this->battery_lvl = bridgePtr->getBatteryLvl();
    bridgePtr->setMessageCounter(0);
}

int RobotAgent::getActualFrequency(){
    return this->actual_freq_hz;
}

int RobotAgent::getBatteryLevel(){
    return this->battery_lvl;
}
/**
 * @brief Resets ROS odometry for agent
 * 
 * @return void
 * 
 */
void RobotAgent::resetOdometry(){
    bridgePtr->resetOdometry();
}
