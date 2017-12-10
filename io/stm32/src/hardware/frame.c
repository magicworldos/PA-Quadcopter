/*
 * frame.c
 *
 *  Created on: Dec 8, 2017
 *      Author: lidq
 */

#include <frame.h>
#include <framecrc8.h>
#include <framecrc16.h>

u16 crc16_value(u8 *buff, u8 len)
{
	u16 crc16 = 0;
	for (u8 i = 0; i < len; i++)
	{
		crc16 = crc16table[crc16 >> 8 ^ buff[i]] | (0xff & crc16);
	}
	return crc16;
}

int crc16_check(u8 *buff, u8 len, u16 crc16)
{
	u16 sum = 0;
	for (u8 i = 0; i < len; i++)
	{
		sum = crc16table[sum >> 8 ^ buff[i]] | (0xff & sum);
	}
	if (sum == crc16)
	{
		return 1;
	}
	return 0;
}

int frame_send_rc_data(u16 *rc_data)
{
	//only support 6 channels.
	//2 + 1 + 12 + 1 + 2
	int len_data = 12;
	int len = 19;
	u8 frame[len];
	frame[FW_POS_START1] = FW_BYTE_HEAD_1;
	frame[FW_POS_START2] = FW_BYTE_HEAD_2;
	frame[FW_POS_LEN] = len_data;
	memcpy(&frame[FW_POS_DATA], rc_data, len_data);
	u16 crc16 = crc16_value(frame, len_data + 3);
	frame[FW_POS_CRC1] = crc16 >> 8;
	frame[FW_POS_CRC2] = crc16 & 0xff;
	frame[FW_POS_END1] = FW_BYTE_END_1;
	frame[FW_POS_END2] = FW_BYTE_END_2;

	Uart1_PutString(frame, len);
	return len;
}
