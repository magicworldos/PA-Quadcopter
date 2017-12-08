/*
 * frame.c
 *
 *  Created on: Dec 8, 2017
 *      Author: lidq
 */

#include <frame.h>
#include <framecrc8.h>

int crc8_value(u8 *buff, u8 len)
{
	u8 crc8 = 0;
	for (u8 i = 0; i < len; i++)
	{
		crc8 = crc8table[buff[i] ^ crc8];
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

int frame_send_rc_data(u16 *rc_data)
{
	//only support 6 channels.
	//2 + 1 + 12 + 1 + 2
	int len_data = 12;
	int len = 18;
	u8 frame[len];
	frame[FW_POS_START1] = FW_BYTE_HEAD_1;
	frame[FW_POS_START2] = FW_BYTE_HEAD_2;
	frame[FW_POS_LEN] = len_data;
	memcpy(&frame[FW_POS_DATA], rc_data, len_data);
	frame[FW_POS_CRC] = crc8_value(frame, len_data + 3);
	frame[FW_POS_END1] = FW_BYTE_END_1;
	frame[FW_POS_END2] = FW_BYTE_END_2;

	Uart1_PutString(frame, len);
	return len;
}
