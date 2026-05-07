#ifndef ODOM_NODE_HPP
#define ODOM_NODE_HPP

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

#define SizeOfCovarianceMatrix 36

class OdomNode
{

public:
    OdomNode(const std::string robot_id, ros::Publisher &odom_data_pub_euler, ros::Publisher &odom_data_pub_quat);
    ~OdomNode(){};

    // Functions
    void update_encoders(int enc_left, int enc_right); // update encoder values
    void update_odom(); // update OdomNew, OdomOld
    void publish_quat(); // publish quat odom
    void publish_euler(); // publish euler odom
    void reset();
    nav_msgs::Odometry get_odom(); // get odomNew

private:
    // Robot id
    const std::string robot_id_;
    // Publishers
    ros::Publisher &odom_data_pub_euler_;
    ros::Publisher &odom_data_pub_quat_;
    // Odometry messages
    nav_msgs::Odometry odomNew_;
    nav_msgs::Odometry odomOld_;
    // Encoder values
    int pos_left_ = 0;
    int pos_right_ = 0;
    int pos_left_prev_ = 0;
    int pos_right_prev_ = 0;
    // Robot physical constants
    const double PI = 3.141592;
    const double TICKS_PER_REVOLUTION = 19456;              // For reference purposes.
    const double wheel_distance = 0.10470;                  // Center of left tire to center of right tire
    const double wheel_conversion_left = 0.13194 / 19456.0; // perimeter over pulses per rev
    const double wheel_conversion_right = 0.13194 / 19456.0;
};


#endif