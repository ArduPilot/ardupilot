#ifndef __APM_XBEE_H__
#define __APM_XBEE_H__

#include <stdint.h>
#include <AP_Common.h>
#include <math.h> 

void int_to_bytes(int loword, int hiword, unsigned char *hexdata);
void XBee_sendFollower(const uint8_t flwid, const struct Location *loc, const uint8_t type);
void Calc_pkt_crc(uint16_t data_sum, uint8_t *crc, uint8_t type);
void Follower_print_recieved(uint8_t *data);
long bytes_to_int(uint8_t *hexdata);

#endif
