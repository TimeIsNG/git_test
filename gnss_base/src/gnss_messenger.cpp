#include "gnss_base/gnss_messenger.hpp"


namespace wescore
{
GnssROSMessenger::GnssROSMessenger(ros::NodeHandle *nh) : gnss_(nullptr), nh_(nh)
{
//    接受一个 ROS 节点句柄指针 nh。
//    gnss_ 被初始化为 nullptr，表示没有绑定任何 GnssBase 实例。
}

GnssROSMessenger::GnssROSMessenger(GnssBase *gnss, ros::NodeHandle *nh) : gnss_(gnss), nh_(nh)
{
//    接受一个 GnssBase 实例指针 gnss 和 ROS 节点句柄指针 nh。
//    用于绑定一个 GnssBase 对象，以从中获取 GNSS 状态。
}

void GnssROSMessenger::SetupSubscription()
{
    // 队列大小为 10，表示最多缓存 10 条尚未被订阅者处理的消息。
    // gps publisher
    // TODO gps_publisher_ 发布 nmea_msgs::Sentence 类型消息，话题名为 /nmea_sentence
    gps_publisher_ = nh_->advertise<nmea_msgs::Sentence>("/nmea_sentence", 10);

    // imu publisher
    // TODO imu_publisher_ 发布 sensor_msgs::Imu 类型消息，话题名为 /imu_from_rtk
    imu_publisher_ = nh_->advertise<sensor_msgs::Imu>("/imu_from_rtk",10);
}

// 将从 GnssBase 对象获取的 GNSS 数据解析并发布到 ROS 主题。
void GnssROSMessenger::PublishStateToROS()
{
    current_time_ = ros::Time::now();
//    double dt = (current_time_ - last_time_).toSec();

// 第一次运行时初始化 last_time_ 为当前时间，并直接返回，不执行后续代码。
    static bool init_run = true;
    if (init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }

// 通过 GnssBase 的 GetGnssState() 方法获取当前 GNSS 状态。
    auto state = gnss_->GetGnssState();

// 检查 state.gpgga_data 是否非空，若非空，则发布 NMEA GPGGA 数据到 /nmea_sentence 话题。
    // publish gnss state message
    if (strlen(state.gpgga_data) > 0)
    {
        // $GPGGA
//        std::cout << "publish gga" << std::endl;
        nmea_msgs::Sentence gps_msg;
        gps_msg.header.stamp = current_time_;
        gps_msg.sentence = state.gpgga_data;

        gps_publisher_.publish(gps_msg);
    }
    if (strlen(state.gpfpd_data) > 0)
    {
        // $GPFPD
        nmea_msgs::Sentence gps_msg;
        gps_msg.header.stamp = current_time_;
        gps_msg.sentence = state.gpfpd_data;

        gps_publisher_.publish(gps_msg);
    }
    if (strlen(state.gprmc_data) > 0)
    {
        // $GPRMC
        nmea_msgs::Sentence gps_msg;
//        std::cout << "publish rmc" << std::endl;
        gps_msg.header.stamp = current_time_;
        gps_msg.sentence = state.gprmc_data;

        gps_publisher_.publish(gps_msg);
    }
    if (strlen(state.gtimu_data) > 8)
    {
        // 检查 gtimu_data 是否长度大于 8，若是，则发布 IMU 数据到 /imu_from_rtk 话题。
        // $GTIMU
        std::string str;
        std::vector<std::string> data;
        std::stringstream ss(state.gtimu_data); // 使用字符串流解析 gtimu_data，将其分割为数据字段存储到 data 数组。

//        使用 getline 函数以逗号 , 为分隔符，将字符串逐字段分割，将解析出的字段依次存入 data 向量。
        while (getline(ss, str, ','))
          data.push_back(str);

        // "$GTIMU,0,879.720,-0.0133,-0.0599,-0.0623,0.0185,0.0041,0.9958,33.0*47";
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = current_time_;
        // 方向协方差：设置为无效值（全 -1）
        imu_msg.orientation_covariance = {-1,-1,-1,-1,-1,-1,-1,-1,-1};

        imu_msg.angular_velocity.x = std::atof(data[3].c_str());
        imu_msg.angular_velocity.y = std::atof(data[4].c_str());
        imu_msg.angular_velocity.z = std::atof(data[5].c_str());

        imu_msg.linear_acceleration.x = std::atof(data[6].c_str()) * 9.8;
        imu_msg.linear_acceleration.y = std::atof(data[7].c_str()) * 9.8;
        imu_msg.linear_acceleration.z = std::atof(data[8].c_str()) * 9.8;

        imu_publisher_.publish(imu_msg);

    }

    // record time for next integration将当前时间存储为 last_time_，以供下次调用使用
    last_time_ = current_time_;
}

} // namespace wescore
