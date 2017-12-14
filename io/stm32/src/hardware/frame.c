/*
 * frame.c
 *
 *  Created on: Dec 8, 2017
 *      Author: lidq
 */

#include <frame.h>
#include <crc.h>

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
