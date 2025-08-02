#ifndef GNSS_PROTOCOL_H
#define GNSS_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*-------------------- Feedback Messages -----------------------*/

/* No padding in the struct */
// reference: https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma pack(push, 1)

// Note: id could be different for UART and CAN protocol

// System Status Feedback
typedef struct {
    char gpgga_data[100];
    char gprmc_data[100];
    char gtimu_data[100];
    char gpfpd_data[1000];
} GnssStatusMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* GNSS_PROTOCOL_H */
