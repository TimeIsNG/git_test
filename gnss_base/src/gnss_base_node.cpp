#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "gnss_base/gnss_messenger.hpp"

using namespace wescore;

void DetachRobot(int signal) {
  std::terminate();
}

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "gnss_odom");
    ros::NodeHandle node(""), private_node("~");

    std::signal(SIGINT, DetachRobot);

    // instantiate a robot object
    GnssBase robot; // 表示 GNSS 基站的硬件接口，负责与实际 GNSS 设备通信
    GnssROSMessenger messenger(&robot, &node); // 负责 GNSS 数据的 ROS 消息处理，包括订阅指令、发布状态数据等

    // fetch parameters before connecting to robot在连接到机器人之前获取参数
    std::string port_name;
    private_node.param<std::string>("port_name", port_name, std::string("/dev/ttyUSB0")); // GNSS 设备的串口端口名称，默认为 /dev/ttyUSB0
    private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom")); //里程计坐标系的名称，默认为 odom
    private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link")); // 机器人底盘的坐标系，默认为 base_link
    private_node.param<bool>("simulated_robot", messenger.simulated_robot_, false); // 是否为仿真模式，默认为 false

    // connect to robot and setup ROS subscription连接到机器人
    if (port_name.find("can") == std::string::npos)
    {
        robot.Connect(port_name, 115200);
        ROS_INFO("Using UART to talk with the robot");
    }
    else
    {
        return 0;
    }
    messenger.SetupSubscription();// 设置 ROS 订阅，监听来自其他节点的消息（例如速度指令

    // publish robot state at 50Hz while listening to twist commands在监听 Twist 命令时以 50Hz 的频率发布机器人状态
    ros::Rate rate_10hz(10); // 10Hz
    while (true)
    {
        messenger.PublishStateToROS(); // 发布 GNSS 状态数据（如里程计消息）
        ros::spinOnce();
        rate_10hz.sleep();
    }

    return 0;
}
