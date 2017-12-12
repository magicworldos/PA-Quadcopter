/*
 * frame_crc.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef __FRAME_CRC_H_
#define __FRAME_CRC_H_

#include <typedef.h>

u8 crc8_value(u8 *buff, u8 len);

int crc8_check(u8 *buff, u8 len, u8 crc8);

u16 crc16_value(u8 *buff, u8 len);

int crc16_check(u8 *buff, u8 len, u16 crc16);

#endif /* SRC_INCLUDE_FRAMECRC8_H_ */
