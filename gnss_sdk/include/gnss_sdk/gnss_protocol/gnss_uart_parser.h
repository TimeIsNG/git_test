#ifndef GNSS_UART_PARSER_H
#define GNSS_UART_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "gnss_sdk/gnss_protocol/gnss_protocol.h"

bool DecodeGnssStatusMsgFromUART(uint8_t c, GnssStatusMessage *msg);

uint8_t CalcGnssUARTChecksum(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_UART_PARSER_H */
