#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

typedef signed char s8;
typedef unsigned char u8;
typedef signed short s16;
typedef unsigned short u16;
typedef signed int s32;
typedef unsigned int u32;

#define BUFFER_SIZE	(256)

#define	FW_POS_START1                         	0
#define	FW_POS_START2	                      	1
#define	FW_POS_LEN	                      		2
#define	FW_POS_DATA	                      		3
#define	FW_POS_CRC  	                      	15
#define	FW_POS_END1  	                     	16
#define	FW_POS_END2  	                     	17

#define FW_BYTE_HEAD_1                         	0X55
#define FW_BYTE_HEAD_2                         	0XAA
#define FW_BYTE_END_1                          	0XA5
#define FW_BYTE_END_2		                   	0X5A

#define FW_HEAD                                 0
#define FW_LEN    								1
#define FW_END    								2

u8 crc8table[256] =
{ 0x76, 0x41, 0xbd, 0xdf, 0xd9, 0xaf, 0x93, 0xd5, 0x77, 0x85, 0xff, 0xae, 0x51, 0x9d, 0x8e, 0x9e, 0xe3, 0xc2, 0xa8, 0x20, 0x1b, 0x2a, 0x75, 0x8a, 0x88, 0xc8, 0xfe, 0x5c, 0x16, 0xca, 0xf1, 0xb2, 0x7c, 0x39, 0x24, 0x5b, 0xb4, 0xbc, 0xd1, 0xfb, 0x54, 0xa2, 0x55, 0x81, 0x89, 0x32, 0x82, 0x4c, 0xf3, 0x49, 0x6d, 0xb8, 0xac, 0xf2, 0xc6, 0xc4, 0x6b, 0x7b, 0xed, 0xcc, 0xe0, 0xbe, 0x45, 0x6f, 0x6c, 0x4b, 0x15, 0xe6, 0x9a, 0x9f, 0x50, 0x58, 0x03, 0xc1, 0x4e, 0x7e, 0x36, 0xf9, 0xc3, 0x52, 0xfa, 0x17, 0xfc, 0x64, 0xad, 0x02, 0xef, 0x37, 0x30, 0x0f, 0x3c, 0x47, 0x60, 0xf4, 0x00, 0xbb, 0x11, 0x72, 0x46, 0xc5, 0xf6, 0x86, 0xcb, 0x1f, 0xb7, 0x08, 0xa0, 0x74, 0xc9, 0x31, 0x3f, 0x65, 0x99, 0xe5, 0x42, 0xda, 0xe8, 0x23, 0xa3, 0x78, 0xd2, 0x68, 0x4f, 0xec, 0x2c, 0x43, 0x96, 0x80, 0xde, 0xcf, 0xb9, 0xa6, 0x09,
        0xdc, 0xd8, 0x0c, 0x83, 0x38, 0x62, 0xf8, 0xe2, 0xbf, 0xdd, 0x53, 0x8c, 0xc7, 0x13, 0x5d, 0x14, 0x12, 0x05, 0x18, 0x19, 0x21, 0x01, 0x90, 0xdb, 0x67, 0x59, 0x22, 0xf5, 0x2f, 0x6a, 0xe7, 0x56, 0x87, 0xba, 0x95, 0x3e, 0x84, 0x3b, 0x8b, 0xb3, 0x1e, 0xaa, 0x71, 0xb6, 0x4a, 0x69, 0x9c, 0x4d, 0x06, 0x98, 0x44, 0xab, 0xa9, 0x27, 0x57, 0xee, 0xeb, 0x29, 0x73, 0x40, 0x2e, 0x0a, 0x0d, 0x79, 0x9b, 0xb5, 0x8d, 0xfd, 0xf7, 0xce, 0xb1, 0x3d, 0x6e, 0x25, 0x5f, 0x2b, 0xd0, 0xe9, 0x0e, 0x7d, 0xd3, 0x48, 0xf0, 0xa5, 0xa7, 0x07, 0x63, 0x5a, 0x66, 0x91, 0xa4, 0xcd, 0x1d, 0xd6, 0xc0, 0x61, 0xd7, 0x33, 0x7a, 0x04, 0x2d, 0x8f, 0x1a, 0x28, 0xa1, 0x97, 0xe1, 0x70, 0x26, 0x0b, 0xd4, 0x1c, 0x94, 0x7f, 0x5e, 0xe4, 0xea, 0xb0, 0x92, 0x10, 0x35, 0x3a, 0x34 };

struct uart_buffer_s
{
	s16 head;
	s16 tail;
	s16 size;
	u8 buffer[BUFFER_SIZE];
	u32 total_len;
	u32 over;
	u32 user_buf_over;
};

int fd = -1;

struct uart_buffer_s _recv;
u8 _packet_buffer[BUFFER_SIZE];

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

void read_data_from_uart()
{
	u8 tmp_serial_buf[BUFFER_SIZE];
	int len = read(fd, tmp_serial_buf, sizeof(tmp_serial_buf));
//	printf("len %d\n", len);
//	for (int i = 0; i < len; i++)
//	{
//		printf("%02x ", tmp_serial_buf[i]);
//	}
//	printf("\n");
	for (int i = 0; i < len; i++)
	{
		_recv.buffer[_recv.head] = tmp_serial_buf[i];
		_recv.head = (_recv.head + 1) % _recv.size;
		if (_recv.head == _recv.tail)
		{
			_recv.over++;
		}
	}
}

int uart_buffer_count(struct uart_buffer_s *lb)
{
	s16 n = lb->head - lb->tail;
	if (n < 0)
	{
		n += lb->size;
	}
	return n;
}

int parse_mag_feedback()
{
	u16 data_cnt = 0;
	u16 parse_packet_step = 0;
	u16 packet_index = 0;
	u8 FrameLen = 0;
	u8 Frame_data_len = 0;
	int ret = 0;
	data_cnt = uart_buffer_count(&_recv);
	for (u16 i = 0; i < data_cnt; i++)
	{
		_packet_buffer[packet_index++] = _recv.buffer[_recv.tail];
		switch (parse_packet_step)
		{
			case FW_HEAD:
				if (packet_index >= 2)
				{
					if ((_packet_buffer[FW_POS_START1] == FW_BYTE_HEAD_1) && (_packet_buffer[FW_POS_START2] == FW_BYTE_HEAD_2))
					{
						parse_packet_step = FW_LEN;
					}
					else if (_packet_buffer[FW_POS_START2] == FW_BYTE_HEAD_1)
					{
						_packet_buffer[FW_POS_START1] = FW_BYTE_HEAD_1;
						packet_index = 1;
					}
					else
					{
						packet_index = 0;
					}
				}
				break;

			case FW_LEN:
				if (packet_index >= 6)
				{
					Frame_data_len = _packet_buffer[FW_POS_LEN];
					FrameLen = Frame_data_len + 6;
					if (FrameLen <= BUFFER_SIZE)
					{
						parse_packet_step = FW_END;
					}
					else
					{
						parse_packet_step = FW_HEAD;
						packet_index = 0;
						memset(_packet_buffer, 0x00, sizeof(_packet_buffer));
					}
				}
				break;

			case FW_END:
				if (packet_index == FrameLen)
				{
					if (crc8_check(&_packet_buffer[FW_POS_START1], Frame_data_len + 3, _packet_buffer[FW_POS_CRC]))
					{
						ret = 1;
					}
					parse_packet_step = FW_HEAD;
					packet_index = 0;
				}
				break;

			default:
				break;
		}
		_recv.tail = (_recv.tail + 1) % _recv.size;
	}
	return ret;
}

int main(int argc, char** argv)
{
	fd = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK); //| O_NOCTTY | O_NDELAY
	if (fd == -1)
	{
		printf("Can't Open Serial Port\n");
		return -1;
	}

	struct termios tios;
	if (tcgetattr(fd, &tios) != 0)
	{
		printf("tcgetattr error\n");
		return -1;
	}
	cfsetispeed(&tios, B115200);
	cfsetospeed(&tios, B115200);
	tios.c_cflag &= ~(CSTOPB | PARENB);
	if ((tcsetattr(fd, TCSANOW, &tios)) != 0)
	{
		printf("tcgetattr error\n");
		return -1;
	}

	_recv.head = 0;
	_recv.tail = 0;
	_recv.size = BUFFER_SIZE;
	memset(_recv.buffer, 0x00, BUFFER_SIZE);

	u16 rc_data[6];

	while (1)
	{
		read_data_from_uart();
		if (parse_mag_feedback())
		{
			memcpy(rc_data, &_packet_buffer[FW_POS_DATA], sizeof(u16) * 6);
			for (int i = 0; i < 6; i++)
			{
				printf("%4d ", rc_data[i]);
			}
			printf("\n");
		}
		usleep(10000);
	}
	close(fd);

	return 0;
}

