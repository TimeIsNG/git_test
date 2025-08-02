#ifndef GNSS_BASE_HPP
#define GNSS_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "gnss_sdk/async_io/async_can.hpp"
#include "gnss_sdk/async_io/async_serial.hpp"

#include "gnss_sdk/gnss_protocol/gnss_protocol.h"
#include "gnss_sdk/gnss_protocol/gnss_uart_parser.h"

#include "gnss_sdk/gnss_protocol/gnss_types.hpp"

namespace wescore
{
class GnssBase
{
public:
    GnssBase() = default;
    ~GnssBase();

    // do not allow copy
    GnssBase(const GnssBase &gnss) = delete;
    GnssBase &operator=(const GnssBase &gnss) = delete;

public:
    // connect to roboot from CAN or serial
    void Connect(std::string dev_name, int32_t baud_rate = 0);

    // disconnect from roboot, only valid for serial port
    void Disconnect();

    // get robot state
    GnssState GetGnssState();

private:
    // hardware communication interface
    std::shared_ptr<ASyncSerial> serial_if_;

    // CAN priority higher than serial if both connected
    bool serial_connected_ = false;

    // serial port related variables
    uint8_t tx_cmd_len_;

    std::mutex gnss_state_mutex_;

    GnssState gnss_state_;


    // internal functions
    void ConfigureSerial(const std::string uart_name = "/dev/ttyUSB0", int32_t baud_rate = 115200);

    void ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received);

    void NewStatusMsgReceivedCallback(const GnssStatusMessage &msg);

public:
    static void UpdateGnssState(const GnssStatusMessage &status_msg, GnssState &state);
};
} // namespace wescore

#endif /* GNSS_BASE_HPP */
