#ifndef GNSS_MESSENGER_HPP
#define GNSS_MESSENGER_HPP

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include "gnss_sdk/gnss_protocol/gnss_base.hpp"

namespace wescore
{
class GnssROSMessenger
{
public:
    explicit GnssROSMessenger(ros::NodeHandle *nh);
    GnssROSMessenger(GnssBase *gnss, ros::NodeHandle *nh);

    std::string odom_frame_;
    std::string base_frame_;

    bool simulated_robot_ = false;

    int sim_control_rate_ = 50;

    void SetupSubscription();

    void PublishStateToROS();

private:
    GnssBase *gnss_;
    ros::NodeHandle *nh_;

    ros::Publisher gps_publisher_;
    ros::Publisher imu_publisher_;

    ros::Time last_time_;
    ros::Time current_time_;

    void PublishOdometryToROS(double linear, double angular, double dt);
};
} // namespace wescore

#endif /* GNSS_MESSENGER_HPP */
