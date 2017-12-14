/*
 * framecrc8.h
 *
 *  Created on: Dec 8, 2017
 *      Author: lidq
 */

#ifndef SRC_INCLUDE_FRAMECRC16_H_
#define SRC_INCLUDE_FRAMECRC16_H_

#include <typedef.h>

u8 crc8_value(u8 *buff, u8 len);

int crc8_check(u8 *buff, u8 len, u8 crc8);

u16 crc16_value(u8 *buff, u8 len);

int crc16_check(u8 *buff, u8 len, u16 crc16);

#endif /* SRC_INCLUDE_FRAMECRC8_H_ */
