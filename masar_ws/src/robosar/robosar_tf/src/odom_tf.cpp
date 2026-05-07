#include "odom_tf.hpp"

odomTF::odomTF(ros::NodeHandle& nh, const std::string robot_id): robot_id_(robot_id) {
    nh_ = nh;
    sub = nh_.subscribe("odom_data_euler", 1000, &odomTF::transformCallback, this);
}

void odomTF::transformCallback(const nav_msgs::Odometry& odom) {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = odom.header.stamp;
    odom_trans.header.frame_id = robot_id_ + "/odom";
    odom_trans.child_frame_id = robot_id_ + "/base_link";

    // translation
    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = odom.pose.pose.position.z;
    // quaternion
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom.pose.pose.orientation.z);
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

}