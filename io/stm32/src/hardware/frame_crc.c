/*
 * frame_crc.c
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#include <frame_crc.h>
#include <crc_table.h>

u8 crc8_value(u8 *buff, u8 len)
{
	u8 crc8 = 0;
	for (u8 i = 0; i < len; i++)
	{
		crc8 = crc8table[crc8 ^ buff[i]];
	}
	return crc8;
}

int crc8_check(u8 *buff, u8 len, u8 crc8)
{
	u8 sum = 0;
	for (u8 i = 0; i < len; i++)
	{
		sum = crc8table[buff[i] ^ sum];
	}
	if (sum == crc8)
	{
		return 1;
	}
	return 0;
}

u16 crc16_value(u8 *buff, u8 len)
{
	u16 crc16 = 0;
	for (u8 i = 0; i < len; i++)
	{
		crc16 = crc16table[((crc16 >> 8) ^ buff[i]) & 0xff] ^ (crc16 << 8);
	}
	return crc16;
}

int crc16_check(u8 *buff, u8 len, u16 crc16)
{
	u16 sum = 0;
	for (u8 i = 0; i < len; i++)
	{
		sum = crc16table[((sum >> 8) ^ buff[i]) & 0xff] ^ (sum << 8);
	}
	if (sum == crc16)
	{
		return 1;
	}
	return 0;
}
