#ifndef GNSS_STATE_HPP
#define GNSS_STATE_HPP

#include <cstdint>
#include <iostream>

namespace wescore
{
struct GnssState
{
    char gpgga_data[100];
    char gprmc_data[100];
    char gtimu_data[100];
    char gpfpd_data[1000];
};
} // namespace wescore

#endif /* GNSS_STATE_HPP */
