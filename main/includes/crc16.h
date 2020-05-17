/*
 * crc16.h
 *
 */
#ifndef crc16_h_included
#define crc16_h_included

#define CRC16_SEED    0xA001

unsigned short calc_crc16(unsigned short seed, const unsigned char* buf, int len);

#endif /* crc16_h_included */

