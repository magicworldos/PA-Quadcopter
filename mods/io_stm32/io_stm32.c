/*
 * driver.c
 *
 *  Created on: Apr 16, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <io_stm32.h>

//电机实际速度
s32 speed[MOTOR_COUNT];
pthread_t pthddr;
s32 sts = 0;
s32 r = 0;
pthread_t pthd;
s_engine* e = NULL;
s_params* p = NULL;

//摇控器pwm信号噪声
f32 ctl_est_devi = 0.2;
f32 ctl_measure_devi = 0.5;
//前后卡尔曼滤波
f32 fb_est = 0.0, fb_devi = 0.0;
//左右卡尔曼滤波
f32 lr_est = 0.0, lr_devi = 0.0;
//油门卡尔曼滤波
f32 pw_est = 0.0, pw_devi = 0.0;
//第4通道卡尔曼滤波
f32 md_est = 0.0, md_devi = 0.0;
//第5通道卡尔曼滤波
f32 ud_est = 0.0, ud_devi = 0.0;
//方向比例通道卡尔曼滤波
f32 di_est = 0.0, di_devi = 0.0;

u8 crc8table[256] = { 0x76, 0x41, 0xbd, 0xdf, 0xd9, 0xaf, 0x93, 0xd5, 0x77, 0x85, 0xff, 0xae, 0x51, 0x9d, 0x8e, 0x9e, 0xe3, 0xc2, 0xa8, 0x20, 0x1b, 0x2a, 0x75, 0x8a, 0x88, 0xc8, 0xfe, 0x5c, 0x16, 0xca, 0xf1, 0xb2, 0x7c, 0x39, 0x24, 0x5b, 0xb4, 0xbc, 0xd1, 0xfb, 0x54, 0xa2, 0x55, 0x81, 0x89, 0x32, 0x82, 0x4c, 0xf3, 0x49, 0x6d, 0xb8, 0xac, 0xf2, 0xc6, 0xc4, 0x6b, 0x7b, 0xed, 0xcc, 0xe0, 0xbe, 0x45, 0x6f, 0x6c, 0x4b, 0x15, 0xe6, 0x9a, 0x9f, 0x50, 0x58, 0x03, 0xc1, 0x4e, 0x7e, 0x36, 0xf9, 0xc3, 0x52, 0xfa, 0x17, 0xfc, 0x64, 0xad, 0x02, 0xef, 0x37, 0x30, 0x0f, 0x3c, 0x47, 0x60, 0xf4, 0x00, 0xbb, 0x11, 0x72, 0x46, 0xc5, 0xf6, 0x86, 0xcb, 0x1f, 0xb7, 0x08, 0xa0, 0x74, 0xc9, 0x31, 0x3f, 0x65, 0x99, 0xe5, 0x42, 0xda, 0xe8, 0x23, 0xa3, 0x78, 0xd2, 0x68, 0x4f, 0xec, 0x2c, 0x43, 0x96, 0x80, 0xde,
		0xcf, 0xb9, 0xa6, 0x09, 0xdc, 0xd8, 0x0c, 0x83, 0x38, 0x62, 0xf8, 0xe2, 0xbf, 0xdd, 0x53, 0x8c, 0xc7, 0x13, 0x5d, 0x14, 0x12, 0x05, 0x18, 0x19, 0x21, 0x01, 0x90, 0xdb, 0x67, 0x59, 0x22, 0xf5, 0x2f, 0x6a, 0xe7, 0x56, 0x87, 0xba, 0x95, 0x3e, 0x84, 0x3b, 0x8b, 0xb3, 0x1e, 0xaa, 0x71, 0xb6, 0x4a, 0x69, 0x9c, 0x4d, 0x06, 0x98, 0x44, 0xab, 0xa9, 0x27, 0x57, 0xee, 0xeb, 0x29, 0x73, 0x40, 0x2e, 0x0a, 0x0d, 0x79, 0x9b, 0xb5, 0x8d, 0xfd, 0xf7, 0xce, 0xb1, 0x3d, 0x6e, 0x25, 0x5f, 0x2b, 0xd0, 0xe9, 0x0e, 0x7d, 0xd3, 0x48, 0xf0, 0xa5, 0xa7, 0x07, 0x63, 0x5a, 0x66, 0x91, 0xa4, 0xcd, 0x1d, 0xd6, 0xc0, 0x61, 0xd7, 0x33, 0x7a, 0x04, 0x2d, 0x8f, 0x1a, 0x28, 0xa1, 0x97, 0xe1, 0x70, 0x26, 0x0b, 0xd4, 0x1c, 0x94, 0x7f, 0x5e, 0xe4, 0xea, 0xb0, 0x92, 0x10, 0x35, 0x3a, 0x34 };

u16 crc16table[256] = { 0x2ae2, 0xf965, 0xa429, 0x1b33, 0xd0ce, 0xd13a, 0x69c2, 0xe829, 0xa785, 0xbec6, 0xe141, 0x3093, 0xb528, 0x94d3, 0xdfb9, 0x5c62, 0x98c2, 0x6fad, 0x234b, 0x8f03, 0xba66, 0x1ebb, 0x2060, 0xb68e, 0x68c9, 0xa3b5, 0xbceb, 0x17dc, 0x7b28, 0x63ba, 0x91aa, 0xa60b, 0x5d20, 0x35d4, 0xc13e, 0x2def, 0x870e, 0x2b01, 0x9618, 0xae94, 0x69c7, 0x775a, 0xdf27, 0x1ef1, 0x0c2e, 0x3ee1, 0xfb52, 0x24f0, 0xae8e, 0x1e9e, 0x33f3, 0xe8f4, 0xbd58, 0xd452, 0x1f83, 0x2622, 0x7808, 0x5c6f, 0x3dfe, 0x7331, 0xc029, 0x4fa8, 0x193d, 0x1d4a, 0x057d, 0xda7b, 0xcb38, 0x8c8b, 0x857d, 0x6151, 0x3b20, 0x6f44, 0x58ab, 0x9a47, 0x8e35, 0xe4d9, 0x5928, 0x0988, 0x09ca, 0x87b6, 0xa825, 0xbdbd, 0x70ab, 0x657e, 0x9210, 0x102f, 0x0ba1, 0x8a19, 0x6c9e, 0xc99e, 0xfd4a, 0x2cc8, 0x1948, 0x9687, 0xca11, 0x1ec5, 0xf102,
		0x954a, 0x2b50, 0x7680, 0x769b, 0xe670, 0xe5c5, 0xcf46, 0x00b8, 0xf3fa, 0x3420, 0x59e1, 0x7d83, 0xbdea, 0xe197, 0x25a9, 0x7ba8, 0xd243, 0x8b28, 0x8db8, 0xe272, 0x96c9, 0x17d2, 0x4f11, 0xe067, 0x951c, 0x7bd9, 0xf9af, 0x2ba4, 0x45eb, 0x9874, 0x9ca7, 0x5b35, 0xc3c5, 0x9327, 0x51d0, 0x2a36, 0x78ed, 0x2117, 0x2aee, 0xece8, 0xd537, 0x84cf, 0xea6b, 0x9322, 0xe667, 0x1015, 0x0ecb, 0xb8ab, 0x9b3d, 0x9c83, 0x9b1e, 0xb206, 0x3456, 0xea2f, 0x126f, 0x4972, 0xe608, 0x0c1f, 0xf516, 0xabf3, 0x2494, 0x91be, 0x0729, 0x6859, 0x24e6, 0xd8f8, 0x928f, 0x1dd4, 0x7a0f, 0xbd7d, 0x8abc, 0x4f47, 0xc24d, 0x7528, 0xe269, 0x28b5, 0x853d, 0xf134, 0xe160, 0xa07b, 0x0db9, 0x7c7f, 0xd281, 0xc20e, 0xe6ae, 0xe4f0, 0x0b81, 0xccb7, 0x7110, 0x0098, 0xf8aa, 0x95a4, 0x1256, 0x7fd3, 0xfdfd, 0x373d, 0x58cc, 0x908d, 0xd510,
		0xd2db, 0xce0a, 0x5fcd, 0x2224, 0x1058, 0xd4f5, 0x048e, 0x390d, 0xda32, 0x75c3, 0x9a6d, 0xfaad, 0x837c, 0x16ed, 0xcd30, 0xc58a, 0x7d9b, 0x3221, 0xd10b, 0xca52, 0xa331, 0x51a4, 0xc2fd, 0x38d6, 0xe3f9, 0x42d1, 0xb6d3, 0x1b37, 0x9b9d, 0xc760, 0x7047, 0x6e7a, 0x956c, 0xd014, 0x109e, 0x25c4, 0x250a, 0x952b, 0xded1, 0x7f3d, 0x0aef, 0x793f, 0x79eb, 0x0e6b, 0x902d, 0xc71b, 0xd3f5, 0x8dc8, 0xf93d, 0x2502, 0x581c, 0x9c6f, 0xf6a5, 0x1b1a, 0x5546, 0xda9f, 0x5dec, 0x8c19, 0x75d7, 0x7989 };

int fd = -1;

struct uart_buffer_s _recv;
u8 _packet_buffer[BUFFER_SIZE];

//初始化时电调可能需要行程校准，通常是3秒最大油门，再3秒最小油门，但不是必要的，可以不做
s32 __init(s_engine* engine, s_params* params)
{
	e = engine;
	p = params;

	r = 1;
	sts = 1;

	fd = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK); //| O_NOCTTY | O_NDELAY
	if (fd == -1)
	{
		printf("Can't Open Serial Port\n");
		return -1;
	}

	if (set_opt(fd, 115200, 8, 'N', 1))
	{
		perror("set_opt error");
		return -1;
	}

	_recv.head = 0;
	_recv.tail = 0;
	_recv.size = BUFFER_SIZE;
	memset(_recv.buffer, 0x00, BUFFER_SIZE);

	u16 rc_data[6];

	for (s32 i = 0; i < MOTOR_COUNT; i++)
	{
		//电机初始速度为0
		speed[i] = 0;
		pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &motor_balance_compensation, NULL);
		pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &read_controller, NULL);
	}

	printf("[ OK ] Motor Init.\n");

	return 0;
}

s32 __destory(s_engine* e, s_params* p)
{
	r = 0;
	usleep(10 * 1000);

	//电机停止
	for (s32 i = 0; i < MOTOR_COUNT; i++)
	{
		speed[i] = 0;
	}

	usleep(10 * 1000);

	sts = 0;

	return 0;
}

s32 __status()
{
	return sts;
}

void motor_balance_compensation()
{
	u16 rc_data[6] = { 0, 0, 0, 0 };
	u16 pwm_data[MOTOR_COUNT] = { 0, 0, 0, 0 };

	while (r)
	{
		e->v = e->v > MAX_SPEED_RUN_MAX ? MAX_SPEED_RUN_MAX : e->v;
		e->v = e->v < MAX_SPEED_RUN_MIN ? MAX_SPEED_RUN_MIN : e->v;

		//在电机锁定时，停止转动，并禁用平衡补偿，保护措施
		if (e->lock || e->v < PROCTED_SPEED)
		{
			for (s32 i = 0; i < MOTOR_COUNT; i++)
			{
				speed[i] = 0;
				pwm_data[i] = speed[i] / 2 + CTL_PWM_MIN;
			}
			frame_send_rc_data(pwm_data);
			usleep(ENG_TIMER * 1000);
			continue;
		}

#ifdef _FLY_MODE_I_
		//四轴平衡补偿I型
		speed[0] = (int)(e->v - e->vz_devi + e->xv_devi - e->zv_devi);
		speed[1] = (int)(e->v - e->vz_devi + e->yv_devi + e->zv_devi);
		speed[2] = (int)(e->v - e->vz_devi - e->xv_devi - e->zv_devi);
		speed[3] = (int)(e->v - e->vz_devi - e->yv_devi + e->zv_devi);
#endif

#ifdef _FLY_MODE_X_
		//四轴平衡补偿X型
		speed[0] = (int) (e->v - e->vz_devi + (e->xv_devi / 2) - (e->yv_devi / 2) - e->zv_devi);
		speed[1] = (int) (e->v - e->vz_devi + (e->xv_devi / 2) + (e->yv_devi / 2) + e->zv_devi);
		speed[2] = (int) (e->v - e->vz_devi - (e->xv_devi / 2) + (e->yv_devi / 2) - e->zv_devi);
		speed[3] = (int) (e->v - e->vz_devi - (e->xv_devi / 2) - (e->yv_devi / 2) + e->zv_devi);
#endif

		//对电机限幅
		for (s32 i = 0; i < MOTOR_COUNT; i++)
		{
			speed[i] = speed[i] > MAX_SPEED_RUN_MAX ? MAX_SPEED_RUN_MAX : speed[i];
			speed[i] = speed[i] < MAX_SPEED_RUN_MIN ? MAX_SPEED_RUN_MIN : speed[i];

		}
		//3
		pwm_data[0] = speed[3] / 2 + CTL_PWM_MIN;
		//2
		pwm_data[1] = speed[2] / 2 + CTL_PWM_MIN;
		//0
		pwm_data[2] = speed[0] / 2 + CTL_PWM_MIN;
		//1
		pwm_data[3] = speed[1] / 2 + CTL_PWM_MIN;
		frame_send_rc_data(pwm_data);

		usleep(ENG_TIMER * 1000);
	}

	sts = 0;
}

void read_controller()
{
	u16 rc_data[6] = { 0, 0, 0, 0 };

	while (r)
	{
		read_data_from_uart();
		if (parse_mag_feedback())
		{
			memcpy(rc_data, &_packet_buffer[FW_POS_DATA], sizeof(u16) * 6);

			fb_est = controller_kalman_filter(fb_est, ctl_est_devi, rc_data[1], ctl_measure_devi, &fb_devi);
			lr_est = controller_kalman_filter(lr_est, ctl_est_devi, rc_data[3], ctl_measure_devi, &lr_est);
			pw_est = controller_kalman_filter(pw_est, ctl_est_devi, rc_data[2], ctl_measure_devi, &pw_devi);
			md_est = controller_kalman_filter(md_est, ctl_est_devi, rc_data[0], ctl_measure_devi, &md_devi);
			ud_est = controller_kalman_filter(ud_est, ctl_est_devi, rc_data[4], ctl_measure_devi, &ud_devi);
			di_est = controller_kalman_filter(di_est, ctl_est_devi, rc_data[5], ctl_measure_devi, &di_devi);

			controller_fb_pwm(rc_data[1]);
			controller_lr_pwm(rc_data[3]);
			controller_pw_pwm(rc_data[2]);
			controller_md_pwm(rc_data[0]);
			controller_ud_pwm(rc_data[4]);
			controller_di_pwm(rc_data[5]);
		}

		usleep(ENG_TIMER * 1000);
	}

	sts = 0;
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

u16 crc16_value(u8 *buff, u8 len)
{
	u16 crc16 = 0;
	for (u8 i = 0; i < len; i++)
	{
		crc16 = crc16table[crc16 >> 8 ^ buff[i]] | (0xff & crc16);
	}
	return crc16;
}

int frame_send_rc_data(u16 *pwm)
{
	//only support 4 channels.
	//2 + 1 + 8 + 1 + 2
	int len_data = 8;
	int len = 15;
	u8 frame[len];
	frame[PWM_POS_START1] = PWM_BYTE_HEAD_1;
	frame[PWM_POS_START2] = PWM_BYTE_HEAD_2;
	frame[PWM_POS_LEN] = len_data;
	memcpy(&frame[PWM_POS_DATA], pwm, len_data);
	u16 crc16 = crc16_value(frame, len_data + 3);
	frame[PWM_POS_CRC1] = crc16 >> 8;
	frame[PWM_POS_CRC2] = crc16 & 0xff;
	frame[PWM_POS_END1] = PWM_BYTE_END_1;
	frame[PWM_POS_END2] = PWM_BYTE_END_2;

	write(fd, frame, len);
	return len;
}

void read_data_from_uart()
{
	u8 tmp_serial_buf[BUFFER_SIZE];
	int len = read(fd, tmp_serial_buf, BUFFER_SIZE);
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
	s16 tail = _recv.tail;
	for (u16 i = 0; i < data_cnt; i++)
	{
		_packet_buffer[packet_index++] = _recv.buffer[tail];
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
					u16 crc16 = _packet_buffer[FW_POS_CRC1] << 8 | _packet_buffer[FW_POS_CRC2];
					if (crc16_check(&_packet_buffer[FW_POS_START1], Frame_data_len + 3, crc16))
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
		tail = (tail + 1) % _recv.size;
	}
	if (ret == 1)
	{
		_recv.tail = tail;
	}
	return ret;
}

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio, oldtio;
	/*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	bzero(&newtio, sizeof(newtio));
	/*步骤一，设置字符大小*/
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	/*设置停止位*/
	switch (nBits)
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}
	/*设置奇偶校验位*/
	switch (nEvent)
	{
		case 'O': //奇数
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK | ISTRIP);
			break;
		case 'E': //偶数
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N': //无奇偶校验位
			newtio.c_cflag &= ~PARENB;
			break;
	}
	/*设置波特率*/
	switch (nSpeed)
	{
		case 2400:
			cfsetispeed(&newtio, B2400);
			cfsetospeed(&newtio, B2400);
			break;
		case 4800:
			cfsetispeed(&newtio, B4800);
			cfsetospeed(&newtio, B4800);
			break;
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
		case 460800:
			cfsetispeed(&newtio, B460800);
			cfsetospeed(&newtio, B460800);
			break;
		default:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
	}
	/*设置停止位*/
	if (nStop == 1)
		newtio.c_cflag &= ~CSTOPB;
	else if (nStop == 2)
		newtio.c_cflag |= CSTOPB;
	/*设置等待时间和最小接收字符*/
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	/*处理未接收字符*/
	tcflush(fd, TCIFLUSH);
	/*激活新配置*/
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}
	printf("set done!\n");
	return 0;
}

//读入摇控器“前/后”的PWM信号
void controller_fb_pwm(s32 fb)
{
	if (fb < CTL_PWM_MIN || fb > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_fb_zero < CTL_PWM_MIN || p->ctl_fb_zero > CTL_PWM_MAX)
	{
		p->ctl_fb_zero = 1500;
	}
	e->ctl_fb = fb;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	//采用二次曲线来对倾斜角做过滤，使角度变化更平滑
	e->ctlmx = controller_parabola((float) (fb - p->ctl_fb_zero));
}

//读入摇控器“左/右”的PWM信号
void controller_lr_pwm(s32 lr)
{
	if (lr < CTL_PWM_MIN || lr > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_lr_zero < CTL_PWM_MIN || p->ctl_lr_zero > CTL_PWM_MAX)
	{
		p->ctl_lr_zero = 1500;
	}
	e->ctl_lr = lr;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	//采用二次曲线来对倾斜角做过滤，使角度变化更平滑
	e->ctlmy = controller_parabola((float) (lr - p->ctl_lr_zero));

	//如果是最左或最右
	if (abs(lr - p->ctl_lr_zero) > 160)
	{
		//如果是最左
		if (lr - p->ctl_lr_zero < 0)
		{
			e->lock_status |= (0x1 << 2);
			e->lock_status &= ~(0x1 << 1);
			return;
		}

		e->lock_status |= (0x1 << 1);
		e->lock_status &= ~(0x1 << 2);
		return;
	}

	e->lock_status &= ~(0x1 << 1);
	e->lock_status &= ~(0x1 << 2);
}

//读入摇控器“油门”的PWM信号
void controller_pw_pwm(s32 pw)
{
	if (pw < CTL_PWM_MIN || pw > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_pw_zero < CTL_PWM_MIN || p->ctl_pw_zero > CTL_PWM_MAX)
	{
		p->ctl_pw_zero = 1000;
	}
	e->ctl_pw = pw;
	//读入速度
	f32 v = (float) (pw - p->ctl_pw_zero);
	//校验速度范围
	v = v > MAX_SPEED_RUN_MAX ? MAX_SPEED_RUN_MAX : v;
	v = v < MAX_SPEED_RUN_MIN ? MAX_SPEED_RUN_MIN : v;

	//在电机锁定时，停止转动，并禁用平衡补偿，保护措施
	if (e->lock || v < PROCTED_SPEED)
	{
		//设置速度为0
		v = 0;
	}

	//设置引擎的速度
	e->v = v;

	//如果是最低油门
	if (abs(pw - p->ctl_pw_zero) < PROCTED_SPEED)
	{
		e->lock_status |= 0x1;
	}
	else
	{
		e->lock_status &= (~0x1);
	}
}

//读入摇控器第4通道PWM信号
void controller_md_pwm(s32 md)
{
	if (md < CTL_PWM_MIN || md > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_md_zero < CTL_PWM_MIN || p->ctl_md_zero > CTL_PWM_MAX)
	{
		p->ctl_md_zero = 2000;
	}
	e->ctl_md = md;

	//读入读数
	f32 val = (float) (md - p->ctl_md_zero);
	if (abs(val) < 500)
	{
		//手动模式
		//e->mode = MODE_MANUAL;
		return;
	}

	//自动模式
	//e->mode = MODE_AUTO;
}

//读入摇控器第5通道PWM信号
void controller_ud_pwm(s32 ud)
{
	if (ud < CTL_PWM_MIN || ud > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_ud_zero < CTL_PWM_MIN || p->ctl_ud_zero > CTL_PWM_MAX)
	{
		p->ctl_ud_zero = 1060;
	}
	e->ctl_ud = ud;
}

//读入摇控器方向舵比例缩放通道PWM信号
void controller_di_pwm(s32 di)
{
	if (di < CTL_PWM_MIN || di > CTL_PWM_MAX)
	{
		return;
	}
	//如果读数超出范围
	if (p->ctl_di_zero < CTL_PWM_MIN || p->ctl_di_zero > CTL_PWM_MAX)
	{
		//这个通道比较特殊，它是对方向舵数值做比例缩放用的，所以当它为读数超出范围时不希望它起作用所以默认读数为0
		p->ctl_di_zero = 0;
	}
	e->ctl_di = di;
}

//取绝对值
f32 controller_abs(f32 x)
{
	if (x < 0)
	{
		return -x;
	}
	return x;
}

//二次曲线函数
f32 controller_parabola(f32 x)
{
	f32 max_pwm = (CTL_PWM_MAX - CTL_PWM_MIN) / 2;

	if (controller_abs(x) < 0.0001)
	{
		return 0;
	}

	if (x > max_pwm)
	{
		return MAX_ANGLE;
	}

	if (x < -max_pwm)
	{
		return -MAX_ANGLE;
	}

	f32 angle = x / max_pwm * MAX_ANGLE;
	angle = angle > MAX_ANGLE ? MAX_ANGLE : angle;
	angle = angle < -MAX_ANGLE ? -MAX_ANGLE : angle;
	angle = angle * M_PI / 180.0;

	//如果方向比例通道无读数则直接返回倾斜角
	if (e->ctl_di < CTL_DI_MIN || e->ctl_di > CTL_DI_MAX)
	{
		return angle;
	}

	//如果方向舵比例通道有读数，倾斜角需要根据此读数做缩放
	f32 sacle = controller_abs((float) e->ctl_di - CTL_DI_MIN) / (CTL_DI_MAX - CTL_DI_MIN);
	return angle * sacle;
}

/***
 * est预估值
 * est_devi预估偏差
 * measure测量读数
 * measure_devi测量噪声
 * devi上一次最优偏差
 */
f32 controller_kalman_filter(f32 est, f32 est_devi, f32 measure, f32 measure_devi, float* devi)
{
	//预估高斯噪声的偏差
	f32 q = sqrt((*devi) * (*devi) + est_devi * est_devi);
	//卡尔曼增益
	f32 kg = q * q / (q * q + measure_devi * measure_devi);
	//滤波结果
	f32 val = est + kg * (measure - est);
	//最优偏差
	*devi = sqrt((1.0 - kg) * q * q);
	return val;
}
