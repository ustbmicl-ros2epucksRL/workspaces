// Created by Indraneel on 3/02/21

#ifndef ROS_FEEDBACK_BRIDGE_HPP
#define ROS_FEEDBACK_BRIDGE_HPP

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "robosar.pb.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <angles/angles.h>
#include "odom_node.hpp"
#include <nav_msgs/Odometry.h>
#include <mutex>
#include "easylogging++.h"
#include <ros/package.h>
#include <robosar_messages/agents_status.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
class ROSFeedbackBridge
{

public:
    ROSFeedbackBridge(std::string robot_id_, ros::NodeHandle nh,const int fb_freq_):
        nh_(nh),node_alive_(true),odom_freq_hz(fb_freq_*2),khepera_frame(robot_id_+"/base_link"),rid(robot_id_),
        odom_data_pub_euler(nh_.advertise<nav_msgs::Odometry>("odom_data_euler", 100)),odom_data_pub_quat(nh_.advertise<nav_msgs::Odometry>("odom_data_quat", 100)),
        odom_node_(rid, odom_data_pub_euler, odom_data_pub_quat), seq_id_prev_(-1)
    {

        // Create ROS nodes for this agent
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("feedback/IMU", 1, true);
        lrf_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("feedback/scan", 1, true);
        apriltag_publisher_ = nh_.advertise<apriltag_ros::AprilTagDetectionArray>("feedback/apriltag", 1, true);

        //// Start Odom node
        odom_delay_ms = (int)((1.0/(double)(odom_freq_hz))*1000.0);
        odom_thread_ = std::thread(&ROSFeedbackBridge::runOdometry, this);

        std::string package_path = ros::package::getPath("robosar_agent_bringup");

        // Create log file for this agent
        el::Configurations agentLogConf;
        agentLogConf.setToDefault();
        // Values are always std::string
        
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::Format, "%datetime %level %msg");
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::ToFile,"true");
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::ToStandardOutput,"false");
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::Filename,package_path+"/logs/"+robot_id_+".txt");
        agentLogConf.set(el::Level::Info,
                el::ConfigurationType::MaxLogFileSize,"1000000");
        // Create new logger
        logger = el::Loggers::getLogger(robot_id_);
        // default logger uses default configurations
        el::Loggers::reconfigureLogger(robot_id_, agentLogConf);

        logger->info("*************************************************************************");
        logger->info("All hail lord gupta");
        logger->info("*************************************************************************");
        
    }

    ~ROSFeedbackBridge()
    {
        ROS_INFO("Killing feedback bridge");
        node_alive_ = false;
        odom_thread_.join();
        el::Loggers::unregisterLogger(rid);
    }

    void unpack_feedback_message(robosar_fms::SensorData* feedback) {

        ROS_DEBUG("Unpacking message");
        // Sequence ID
        int32_t seq_diff = feedback->seq_id() - seq_id_prev_;
        if(seq_diff < 0)
        {
            // Gone backwards; out of order. Log, report, discard.
            // If difference is huge, then due to restart; update prev id
            ROS_WARN("[unpack_feedback_message] Packets out of order for %s; expected sequence ID %d, got %u. Ignoring.", rid.c_str(), seq_id_prev_+1, feedback->seq_id());
            logger->info("    WARNING: Packets out of order detected (prev: %v, new: %v)", seq_id_prev_, feedback->seq_id());
            if(seq_diff < -100)
            {
              seq_id_prev_ = feedback->seq_id();
              ROS_WARN("[unpack_feedback_message] Packet ID difference is too large for %s (%d); updating local ID to %u", rid.c_str(), seq_diff, feedback->seq_id());
              logger->info("    INFO: Difference in ID too large; updating local ID to %v", feedback->seq_id());
            }
            return;
        }
        else if(seq_diff == 0)
        {
            // Duplicate; something is very wrong. Log, report, discard.
            ROS_WARN("[unpack_feedback_message] Repeated packet for %s; expected sequence ID %d, got %u.", rid.c_str(), seq_id_prev_+1, feedback->seq_id());
            logger->info("    WARNING: Packet sequence duplicate detected (prev: %v, new: %v)", seq_id_prev_, feedback->seq_id());
            return;
        }
        else if(seq_diff > 1)
        {
            // Missed packet(s). Not unusual, so don't need to print or log
            seq_id_prev_ = feedback->seq_id();
        }
        else
        {
            // No issues
            seq_id_prev_ = feedback->seq_id();
        }
        // Battery level
        battery_lvl = feedback->agent_status_data().battery_level();
        //IMU
        sensor_msgs::Imu imu_msg;
        
        imu_msg.header.frame_id = khepera_frame;
        imu_msg.header.seq = feedback->seq_id();
        imu_msg.header.stamp = ros::Time::now();

        //@indraneel convert to m/s^2
        imu_msg.linear_acceleration.x = feedback->accel_data().acc_x();
        imu_msg.linear_acceleration.y = feedback->accel_data().acc_y();
        imu_msg.linear_acceleration.z = feedback->accel_data().acc_z();

        imu_msg.angular_velocity.x = angles::from_degrees(feedback->gyro_data().gyro_x());
        imu_msg.angular_velocity.y = angles::from_degrees(feedback->gyro_data().gyro_y());
        imu_msg.angular_velocity.z = angles::from_degrees(feedback->gyro_data().gyro_z());
        
        imu_publisher_.publish(imu_msg);

        logger->info("IMU seq :%v, accel_x :%v, accel_y :%v, accel_z :%v, ang_x :%v,  ang_y :%v,  ang_z :%v",imu_msg.header.seq,
                                                                                                            imu_msg.linear_acceleration.x,
                                                                                                            imu_msg.linear_acceleration.y,
                                                                                                            imu_msg.linear_acceleration.z,
                                                                                                            imu_msg.angular_velocity.x,
                                                                                                            imu_msg.angular_velocity.y,
                                                                                                            imu_msg.angular_velocity.z);
        // encoder callback
        int pos_left, pos_right;
        {
            std::lock_guard<std::mutex> guard(mtx);
            pos_left = feedback->count_data().left();
            pos_right = feedback->count_data().right();
            odom_node_.update_encoders(pos_left, pos_right);
        }
        logger->info("Left ticks: %v", pos_left);
        logger->info("Right ticks: %v", pos_right);

        // LaserScan
        sensor_msgs::LaserScan lrf_msg;

        lrf_msg.header.frame_id = khepera_frame;
        lrf_msg.header.stamp = ros::Time::now();
        lrf_msg.header.seq = feedback->seq_id();
        // verified from URG-04LX-UG01 specification datasheet
        lrf_msg.angle_min = -2.0923497676849365 + 0.006135923322290182;
        lrf_msg.angle_max = 2.0923497676849365;
        lrf_msg.angle_increment = 0.006135923322290182;
        lrf_msg.time_increment = 9.765627328306437e-05;
        lrf_msg.scan_time = 0.10000000149011612;
        lrf_msg.range_min = 0.019999999552965164;
        lrf_msg.range_max = 5.599999904632568;
        robosar_fms::LaserScanner lrf_feedback = feedback->lrf_data();
        uint64_t laser_scan_max = -INT64_MAX, laser_scan_min = INT64_MAX;
        //ROS_INFO("Lrf data size : %d\n",lrf_feedback.values_size());
        for(int i=0;i<lrf_feedback.values_size();i++)
        {
            if(lrf_feedback.values(i)>laser_scan_max){
                laser_scan_max = lrf_feedback.values(i);
            }
            if(lrf_feedback.values(i)<laser_scan_min){
                laser_scan_min = lrf_feedback.values(i);
            }
            // mm to metres
            lrf_msg.ranges.push_back((float)(lrf_feedback.values(i))/1000.0f);
        }
        logger->info("Laser Scan Max: %v", laser_scan_max);
        logger->info("laser Scan Min: %v", laser_scan_min);
        logger->info("Laser Scan Size: %v", lrf_feedback.values_size());

        message_counter++;
        lrf_publisher_.publish(lrf_msg);

        // Apriltag detections
        robosar_fms::AllDetections apriltag_feedback = feedback->april_detections();
        apriltag_ros::AprilTagDetectionArray tag_msg;
        tag_msg.header.frame_id = khepera_frame;
        tag_msg.header.stamp = ros::Time::now();
        tag_msg.header.seq = feedback->seq_id();

        for(int i=0;i<apriltag_feedback.tag_detections_size();i++) {
            robosar_fms::AprilTagDetection tag = apriltag_feedback.tag_detections(i);

            logger->info("Apriltag id %v w translation %v %v %v and rotation mat %v %v %v | %v %v %v | %v %v %v",
                            tag.tag_id(), tag.pose().t().x(),tag.pose().t().y(),tag.pose().t().z(), \
                             tag.pose().r().r11(),tag.pose().r().r12(),tag.pose().r().r13(),
                             tag.pose().r().r21(),tag.pose().r().r22(),tag.pose().r().r23(),
                             tag.pose().r().r31(),tag.pose().r().r32(),tag.pose().r().r33());

            apriltag_ros::AprilTagDetection tag_detection;
            tag_detection.id.push_back(static_cast<int>(tag.tag_id()));
            //tag_detection.size = tag.size();
            tag_detection.pose.pose.pose.position.x =  tag.pose().t().x();
            tag_detection.pose.pose.pose.position.y =  tag.pose().t().y();
            tag_detection.pose.pose.pose.position.z =  tag.pose().t().z();

            // Rotation matrix to euler angles
            tf2::Matrix3x3 mat(tag.pose().r().r11(),tag.pose().r().r12(),tag.pose().r().r13(),
                              tag.pose().r().r21(),tag.pose().r().r22(),tag.pose().r().r23(),
                              tag.pose().r().r31(),tag.pose().r().r32(),tag.pose().r().r33());
            
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            // Euler angles to quaternion
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            quat.normalize();

            tag_detection.pose.pose.pose.orientation.x = quat.x();
            tag_detection.pose.pose.pose.orientation.y = quat.y();
            tag_detection.pose.pose.pose.orientation.z = quat.z();
            tag_detection.pose.pose.pose.orientation.w = quat.w();
            tag_msg.detections.push_back(tag_detection);
        }
        if(apriltag_feedback.tag_detections_size()>0)
            apriltag_publisher_.publish(tag_msg);
    }

    void runOdometry()
    {
        //ROS_INFO("Spinning odometry with delay %d ms",odom_delay_ms);
        while(node_alive_)
        {
            {
                std::lock_guard<std::mutex> guard(mtx);
                // Update odometry messages
                odom_node_.update_odom();
            }
            // Publish odometry messages
            odom_node_.publish_euler();
            odom_node_.publish_quat();
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_delay_ms));   
        }
    }

    void resetOdometry(){
        std::lock_guard<std::mutex> guard(mtx);
        // Update odometry messages
        odom_node_.reset();
    }

    int getMessageCounter(){
        std::lock_guard<std::mutex> guard(mtx);
        return message_counter;
    }

    int getBatteryLvl(){
        std::lock_guard<std::mutex> guard(mtx);
        return battery_lvl;
    }
    void setMessageCounter(int count){
        std::lock_guard<std::mutex> guard(mtx);
        message_counter=count;
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Publisher odom_data_pub_euler;
    ros::Publisher odom_data_pub_quat;
    ros::Publisher lrf_publisher_;
    ros::Publisher apriltag_publisher_;
    std::string khepera_frame;
    std::thread odom_thread_;
    bool node_alive_;
    int odom_freq_hz;
    int odom_delay_ms;
    std::mutex mtx;
    el::Logger* logger; 
    std::string rid;
    OdomNode odom_node_;
    int message_counter;
    int battery_lvl;
    int32_t seq_id_prev_;
};

#endif