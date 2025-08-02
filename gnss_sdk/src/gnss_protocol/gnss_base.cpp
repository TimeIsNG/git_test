#include "gnss_sdk/gnss_protocol/gnss_base.hpp"

#include <string>
#include <string.h>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

namespace
{
    // 定义了一个计时器工具 StopWatch，用于精确测量时间间隔并支持线程睡眠
struct StopWatch
{
    // StopWatch 是一个计时器工具，用于精确测量时间间隔并支持线程睡眠。
    using Clock = std::chrono::high_resolution_clock;
    using time_point = typename Clock::time_point;
    using duration = typename Clock::duration;

    StopWatch() { tic_point = Clock::now(); };

    time_point tic_point;

    void tic()
    {
        tic_point = Clock::now();
    };

    double toc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count() / 1000000.0;
    };

    // for different precisions
    double stoc() // 返回秒级间隔
    {
        return std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - tic_point).count();
    };

    double mtoc() // 返回毫秒级间隔
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();
    };

    double utoc() // 返回微秒级间隔
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();
    };

    double ntoc() // 返回纳秒级间隔
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - tic_point).count();
    };

    // you have to call tic() before calling this function
    // 根据给定的时间间隔（毫秒或微秒），计算剩余时间并让线程睡眠以等待特定时间点
    void sleep_until_ms(int64_t period_ms)
    {
        int64_t duration = period_ms - std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(duration));
    };

    void sleep_until_us(int64_t period_us)
    {
        int64_t duration = period_us - std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(duration));
    };
};
} // namespace

namespace wescore
{
    // 实现了 GnssBase 类，处理串口通信、数据解析以及 GNSS 状态的更新
    // GnssBase 是一个抽象类，提供与 GNSS 设备通信的基本功能，包括连接设备、接收数据、解析数据和维护设备状态。
GnssBase::~GnssBase()
{
    if (serial_connected_)
        serial_if_->close();
    std::cout<<"close"<<std::endl;
//    在对象销毁时关闭串口连接。
//    确保资源（如串口）被正确释放，避免资源泄漏。
}

void GnssBase::Connect(std::string dev_name, int32_t baud_rate)
{
//    功能：配置串口连接。
//    参数：
//    dev_name：串口设备名称（如 /dev/ttyUSB0）。
//    baud_rate：波特率（如 115200）。
//    如果串口未能连接，输出错误信息。
    if (baud_rate != 0)
    {
        ConfigureSerial(dev_name, baud_rate);

        if (!serial_connected_)
            std::cerr << "ERROR: Failed to connect to serial port" << std::endl;
    }
}

void GnssBase::Disconnect()
{
//    关闭串口连接，释放资源。
    if (serial_connected_)
    {
        if (serial_if_->is_open())
            serial_if_->close();
    }
}


void GnssBase::ConfigureSerial(const std::string uart_name, int32_t baud_rate)
{
//    创建一个异步串口对象 serial_if_。
//    打开串口连接。
//    注册回调函数 ParseUARTBuffer()，用于处理接收到的串口数据。
    serial_if_ = std::make_shared<ASyncSerial>(uart_name, baud_rate); // 使用 std::make_shared 创建 ASyncSerial 对象。
    serial_if_->open();

    if (serial_if_->is_open())
        serial_connected_ = true;
//    std::bind 将成员函数 ParseUARTBuffer 与串口接收事件绑定，确保数据到来时自动调用解析函数。
    serial_if_->set_receive_callback(std::bind(&GnssBase::ParseUARTBuffer, this,
                                               std::placeholders::_1,
                                               std::placeholders::_2,
                                               std::placeholders::_3));
}

GnssState GnssBase::GetGnssState()
{
    std::lock_guard<std::mutex> guard(gnss_state_mutex_);
    return gnss_state_;
}

void GnssBase::ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
//    逐字节解析串口缓冲区中的数据。
//    遍历接收到的字节流。
//    调用 DecodeGnssStatusMsgFromUART() 解析字节，尝试生成完整的 GNSS 状态消息。
//    如果成功解析消息，则调用回调函数 NewStatusMsgReceivedCallback() 进行处理。
    // std::cout << "bytes received from serial: " << bytes_received << std::endl;
    GnssStatusMessage status_msg;
    for (int i = 0; i < bytes_received; ++i)
    {
        if (DecodeGnssStatusMsgFromUART(buf[i], &status_msg))
            NewStatusMsgReceivedCallback(status_msg);
    }
}

void GnssBase::NewStatusMsgReceivedCallback(const GnssStatusMessage &msg)
{
//    在解析成功后，更新内部的 GNSS 状态 gnss_state_。
//    使用 std::lock_guard 确保对共享数据的线程安全访问。
    // std::cout << "new status msg received" << std::endl;
    std::lock_guard<std::mutex> guard(gnss_state_mutex_);
    UpdateGnssState(msg, gnss_state_);
}

void GnssBase::UpdateGnssState(const GnssStatusMessage &status_msg, GnssState &state)
{
//    将解码的状态消息中的数据更新到内部状态 gnss_state_。
//    使用 strcpy 复制字符串数据（例如 GNSS 的 $GPGGA 消息）。
//    可能存储定位信息（如经纬度、高度）或设备状态。
    strcpy(state.gpgga_data,status_msg.gpgga_data);
    strcpy(state.gprmc_data,status_msg.gprmc_data);
    strcpy(state.gtimu_data,status_msg.gtimu_data);
    strcpy(state.gpfpd_data,status_msg.gpfpd_data);
}
} // namespace wescore
