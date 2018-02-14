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
f32 ctl_est_devi = 15;
f32 ctl_measure_devi = 6;
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

u16 crc16table[256] = { 0x2ae2, 0xf965, 0xa429, 0x1b33, 0xd0ce, 0xd13a, 0x69c2, 0xe829, 0xa785, 0xbec6, 0xe141, 0x3093, 0xb528, 0x94d3, 0xdfb9, 0x5c62, 0x98c2, 0x6fad, 0x234b, 0x8f03, 0xba66, 0x1ebb, 0x2060, 0xb68e, 0x68c9, 0xa3b5, 0xbceb, 0x17dc, 0x7b28, 0x63ba, 0x91aa, 0xa60b, 0x5d20, 0x35d4, 0xc13e, 0x2def, 0x870e, 0x2b01, 0x9618, 0xae94, 0x69c7, 0x775a, 0xdf27, 0x1ef1, 0x0c2e, 0x3ee1, 0xfb52, 0x24f0, 0xae8e, 0x1e9e, 0x33f3, 0xe8f4, 0xbd58, 0xd452, 0x1f83, 0x2622, 0x7808, 0x5c6f, 0x3dfe, 0x7331, 0xc029, 0x4fa8, 0x193d, 0x1d4a, 0x057d, 0xda7b, 0xcb38, 0x8c8b, 0x857d, 0x6151, 0x3b20, 0x6f44, 0x58ab, 0x9a47, 0x8e35, 0xe4d9, 0x5928, 0x0988, 0x09ca, 0x87b6, 0xa825, 0xbdbd, 0x70ab, 0x657e, 0x9210, 0x102f, 0x0ba1, 0x8a19, 0x6c9e, 0xc99e, 0xfd4a, 0x2cc8, 0x1948, 0x9687, 0xca11, 0x1ec5, 0xf102,
		0x954a, 0x2b50, 0x7680, 0x769b, 0xe670, 0xe5c5, 0xcf46, 0x00b8, 0xf3fa, 0x3420, 0x59e1, 0x7d83, 0xbdea, 0xe197, 0x25a9, 0x7ba8, 0xd243, 0x8b28, 0x8db8, 0xe272, 0x96c9, 0x17d2, 0x4f11, 0xe067, 0x951c, 0x7bd9, 0xf9af, 0x2ba4, 0x45eb, 0x9874, 0x9ca7, 0x5b35, 0xc3c5, 0x9327, 0x51d0, 0x2a36, 0x78ed, 0x2117, 0x2aee, 0xece8, 0xd537, 0x84cf, 0xea6b, 0x9322, 0xe667, 0x1015, 0x0ecb, 0xb8ab, 0x9b3d, 0x9c83, 0x9b1e, 0xb206, 0x3456, 0xea2f, 0x126f, 0x4972, 0xe608, 0x0c1f, 0xf516, 0xabf3, 0x2494, 0x91be, 0x0729, 0x6859, 0x24e6, 0xd8f8, 0x928f, 0x1dd4, 0x7a0f, 0xbd7d, 0x8abc, 0x4f47, 0xc24d, 0x7528, 0xe269, 0x28b5, 0x853d, 0xf134, 0xe160, 0xa07b, 0x0db9, 0x7c7f, 0xd281, 0xc20e, 0xe6ae, 0xe4f0, 0x0b81, 0xccb7, 0x7110, 0x0098, 0xf8aa, 0x95a4, 0x1256, 0x7fd3, 0xfdfd, 0x373d, 0x58cc, 0x908d, 0xd510,
		0xd2db, 0xce0a, 0x5fcd, 0x2224, 0x1058, 0xd4f5, 0x048e, 0x390d, 0xda32, 0x75c3, 0x9a6d, 0xfaad, 0x837c, 0x16ed, 0xcd30, 0xc58a, 0x7d9b, 0x3221, 0xd10b, 0xca52, 0xa331, 0x51a4, 0xc2fd, 0x38d6, 0xe3f9, 0x42d1, 0xb6d3, 0x1b37, 0x9b9d, 0xc760, 0x7047, 0x6e7a, 0x956c, 0xd014, 0x109e, 0x25c4, 0x250a, 0x952b, 0xded1, 0x7f3d, 0x0aef, 0x793f, 0x79eb, 0x0e6b, 0x902d, 0xc71b, 0xd3f5, 0x8dc8, 0xf93d, 0x2502, 0x581c, 0x9c6f, 0xf6a5, 0x1b1a, 0x5546, 0xda9f, 0x5dec, 0x8c19, 0x75d7, 0x7989 };

int fd = -1;

s_buff _recv;
u8 _buff[BUFF_SIZE];

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

	if (set_opt(fd, B115200, 8, 'N', 1))
	{
		perror("set_opt error");
		return -1;
	}

	_recv.head = 0;
	_recv.tail = 0;
	_recv.size = BUFF_SIZE;
	memset(_recv.buffer, 0x00, BUFF_SIZE);

	u16 rc_data[8];

	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &io_rc_data, NULL);

	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &io_pwm_data, NULL);

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

void io_pwm_data()
{
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
				pwm_data[i] = CTL_PWM_MIN;
			}
			frame_send_pwm_data(pwm_data);
			usleep(ENG_TIMER * 1000);
			continue;
		}

#ifdef _FLY_MODE_I_
		//四轴平衡补偿I型
		speed[0] = (int)(e->v + e->xv_devi - e->zv_devi);
		speed[1] = (int)(e->v + e->yv_devi + e->zv_devi);
		speed[2] = (int)(e->v - e->xv_devi - e->zv_devi);
		speed[3] = (int)(e->v - e->yv_devi + e->zv_devi);
#endif

#ifdef _FLY_MODE_X_
		//四轴平衡补偿X型
		speed[0] = (int) (e->v + (e->xv_devi / 2) - (e->yv_devi / 2) - e->zv_devi);
		speed[1] = (int) (e->v + (e->xv_devi / 2) + (e->yv_devi / 2) + e->zv_devi);
		speed[2] = (int) (e->v - (e->xv_devi / 2) + (e->yv_devi / 2) - e->zv_devi);
		speed[3] = (int) (e->v - (e->xv_devi / 2) - (e->yv_devi / 2) + e->zv_devi);
#endif

		//对电机限幅
		for (s32 i = 0; i < MOTOR_COUNT; i++)
		{
			speed[i] = speed[i] > MAX_SPEED_RUN_MAX ? MAX_SPEED_RUN_MAX : speed[i];
			speed[i] = speed[i] < MAX_SPEED_RUN_MIN ? MAX_SPEED_RUN_MIN : speed[i];

		}
		//3
		pwm_data[0] = speed[3] + CTL_PWM_MIN;
		//2
		pwm_data[1] = speed[2] + CTL_PWM_MIN;
		//0
		pwm_data[2] = speed[0] + CTL_PWM_MIN;
		//1
		pwm_data[3] = speed[1] + CTL_PWM_MIN;

		frame_send_pwm_data(pwm_data);

		usleep(ENG_TIMER * 1000);
	}

	sts = 0;
}

void io_rc_data()
{
	u16 rc_data[RCCH_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	u32 rc_error_count = 0;
	while (r)
	{
		frame_read_rc_data();
		if (frame_parse_rc())
		{
			rc_error_count = 0;
			memcpy(rc_data, &_buff[RC_POS_DATA], sizeof(u16) * RCCH_COUNT);
			controller_pitch_pwm(rc_data[4]);
			controller_roll_pwm(rc_data[2]);
			controller_power_pwm(rc_data[3]);
			controller_pro_pwm(rc_data[0]);
			controller_mod0_pwm(rc_data[5]);
			controller_mod1_pwm(rc_data[1]);
			//for (int i = 0; i < RCCH_COUNT; i++)
			//{
			//	printf("%4d ", rc_data[i]);
			//}
			//printf("\n");
		}
		if (rc_error_count < 2 * PWM_ERR_MAX)
		{
			rc_error_count++;
		}
		if (rc_error_count > PWM_ERR_MAX)
		{
			memset(rc_data, 0, sizeof(u16) * RCCH_COUNT);
			controller_pitch_pwm(rc_data[4]);
			controller_roll_pwm(rc_data[2]);
			controller_power_pwm(rc_data[3]);
			controller_pro_pwm(rc_data[0]);
			controller_mod0_pwm(rc_data[5]);
			controller_mod1_pwm(rc_data[1]);
		}
		usleep(ENG_TIMER * 1000);
	}

	sts = 0;
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

int frame_send_pwm_data(u16 *pwm)
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

void frame_read_rc_data()
{
	u8 tmp_serial_buf[BUFF_SIZE];
	int len = read(fd, tmp_serial_buf, BUFF_SIZE);
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

int frame_count_rc(s_buff *lb)
{
	s16 n = lb->head - lb->tail;
	if (n < 0)
	{
		n += lb->size;
	}
	return n;
}

int frame_parse_rc()
{
	u16 data_cnt = 0;
	u16 parse_packet_step = 0;
	u16 packet_index = 0;
	u8 frameLen = 0;
	u8 frame_data_len = 0;
	int ret = 0;
	data_cnt = frame_count_rc(&_recv);
	s16 tail = _recv.tail;
	for (u16 i = 0; i < data_cnt; i++)
	{
		_buff[packet_index++] = _recv.buffer[tail];
		switch (parse_packet_step)
		{
			case RC_HEAD:
				if (packet_index >= 2)
				{
					if ((_buff[RC_POS_START1] == RC_BYTE_HEAD_1) && (_buff[RC_POS_START2] == RC_BYTE_HEAD_2))
					{
						parse_packet_step = RC_LEN;
					}
					else if (_buff[RC_POS_START2] == RC_BYTE_HEAD_1)
					{
						_buff[RC_POS_START1] = RC_BYTE_HEAD_1;
						packet_index = 1;
					}
					else
					{
						packet_index = 0;
					}
				}
				break;

			case RC_LEN:
				if (packet_index >= 6)
				{
					frame_data_len = _buff[RC_POS_LEN];
					frameLen = frame_data_len + 6;
					if (frameLen <= BUFF_SIZE)
					{
						parse_packet_step = RC_END;
					}
					else
					{
						parse_packet_step = RC_HEAD;
						packet_index = 0;
						memset(_buff, 0x00, sizeof(_buff));
					}
				}
				break;

			case RC_END:
				if (packet_index == frameLen)
				{
					u16 crc16 = _buff[RC_POS_CRC1] << 8 | _buff[RC_POS_CRC2];
					if (crc16_check(&_buff[RC_POS_START1], frame_data_len + 3, crc16))
					{
						ret = 1;
					}
					parse_packet_step = RC_HEAD;
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
	//设置波特率
	cfsetispeed(&newtio, nSpeed);
	cfsetospeed(&newtio, nSpeed);
	/*设置停止位*/
	if (nStop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if (nStop == 2)
	{
		newtio.c_cflag |= CSTOPB;
	}
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
void controller_pitch_pwm(s32 fb)
{
	//if (fb < PROCTED_SPEED)
	//{
	//	e->ctl_fb = 0;
	//	return;
	//}
	if (fb < CTL_PWM_MIN || fb > CTL_PWM_MAX)
	{
		e->ctl_fb = 1500;
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
void controller_roll_pwm(s32 lr)
{
	if (lr < PROCTED_SPEED)
	{
		e->ctl_lr = 0;
		return;
	}
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
		if (lr - p->ctl_lr_zero > 0)
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
void controller_power_pwm(s32 pw)
{
	if (pw < PROCTED_SPEED)
	{
		e->ctl_pw = 0;
		e->v = 0;
		return;
	}
	if (pw > CTL_PWM_MAX)
	{
		e->ctl_pw = CTL_PWM_MAX;
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
void controller_mod0_pwm(s32 md)
{
	if (md < CTL_PWM_MIN || md > CTL_PWM_MAX)
	{
		e->ctl_md = 1500;
		return;
	}
	if (p->ctl_md_zero < CTL_PWM_MIN || p->ctl_md_zero > CTL_PWM_MAX)
	{
		p->ctl_md_zero = 1500;
	}
	e->ctl_md = md;
	float z = ((float) (e->ctl_md - p->ctl_md_zero)) / 30000.0;
	if (fabs(z) > 0.002)
	{
		e->ctlmz += z;
	}

}

//读入摇控器第5通道PWM信号
void controller_mod1_pwm(s32 ud)
{
	if (ud < PROCTED_SPEED)
	{
		e->ctl_ud = 0;
		return;
	}
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
void controller_pro_pwm(s32 di)
{
	if (di < PROCTED_SPEED)
	{
		e->ctl_di = 0;
		return;
	}
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
