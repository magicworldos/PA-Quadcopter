/*
 * serial_port.c
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#include <serial_port.h>
#include <frame_crc.h>

s_serial_port recv;
u8 buff[BUFF_SIZE];
u32 pwm_out_error_count = 0;

void serial_port_init()
{
	serial_port_gpio_config();
	serial_port_mode_config();

	recv.head = 0;
	recv.tail = 0;
	recv.size = BUFF_SIZE;
	memset(recv.buffer, 0x00, BUFF_SIZE);
}

void serial_port_gpio_config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void serial_port_mode_config()
{
	USART_InitTypeDef usart1_init_struct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	USART_Cmd(USART1, ENABLE);
	usart1_init_struct.USART_BaudRate = USART_BAUDRATE;
	usart1_init_struct.USART_WordLength = USART_WordLength_8b;
	usart1_init_struct.USART_StopBits = USART_StopBits_1;
	usart1_init_struct.USART_Parity = USART_Parity_No;
	usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &usart1_init_struct);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
}

u8 serial_port_putchar(u8 ch)
{
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	//USART_GetFlagStatus(USART1, USART_FLAG_TC);
	USART_SendData(USART1, (u8) ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
	{
	}
	return ch;
}

void serial_port_puts(u8* buf, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		serial_port_putchar(*buf++);
	}
}

int serial_port_buff_count(s_serial_port *sp)
{
	s16 n = sp->head - sp->tail;
	if (n < 0)
	{
		n += sp->size;
	}
	return n;
}

int serial_port_frame_pase()
{
	u16 data_cnt = 0;
	u16 parse_packet_step = 0;
	u16 packet_index = 0;
	u8 FrameLen = 0;
	u8 Frame_data_len = 0;
	int ret = 0;
	data_cnt = serial_port_buff_count(&recv);
	s16 tail = recv.tail;
	for (u16 i = 0; i < data_cnt; i++)
	{
		buff[packet_index++] = recv.buffer[tail];
		switch (parse_packet_step)
		{
			case PWM_HEAD:
				if (packet_index >= 2)
				{
					if ((buff[PWM_POS_START1] == PWM_BYTE_HEAD_1) && (buff[PWM_POS_START2] == PWM_BYTE_HEAD_2))
					{
						parse_packet_step = PWM_LEN;
					}
					else if (buff[PWM_POS_START2] == PWM_BYTE_HEAD_1)
					{
						buff[PWM_POS_START1] = PWM_BYTE_HEAD_1;
						packet_index = 1;
					}
					else
					{
						packet_index = 0;
					}
				}
				break;

			case PWM_LEN:
				if (packet_index >= 6)
				{
					Frame_data_len = buff[PWM_POS_LEN];
					FrameLen = Frame_data_len + 6;
					if (FrameLen <= BUFF_SIZE)
					{
						parse_packet_step = PWM_END;
					}
					else
					{
						parse_packet_step = PWM_HEAD;
						packet_index = 0;
						memset(buff, 0x00, sizeof(buff));
					}
				}
				break;

			case PWM_END:
				if (packet_index == FrameLen)
				{
					u16 crc16 = buff[PWM_POS_CRC1] << 8 | buff[PWM_POS_CRC2];
					if (crc16_check(&buff[PWM_POS_START1], Frame_data_len + 3, crc16))
					{
						ret = 1;
					}
					parse_packet_step = PWM_HEAD;
					packet_index = 0;
				}
				break;

			default:
				break;
		}
		tail = (tail + 1) % recv.size;
	}
	if (ret == 1)
	{
		recv.tail = tail;
	}
	return ret;
}

void serial_port_limit(u16 *value, int len)
{
	for (int i = 0; i < len; i++)
	{
		value[i] = value[i] > LIMIT_MAX ? LIMIT_MAX : value[i];
		value[i] = value[i] < LIMIT_MIN ? LIMIT_MIN : value[i];
	}
}

int serial_port_frame_recv_pwm(u16 *pwm)
{
	if (serial_port_frame_pase())
	{
		memcpy(pwm, &buff[PWM_POS_DATA], sizeof(u16) * 4);
		pwm_out_error_count = 0;
		serial_port_limit(pwm, 4);
		return 1;
	}
	return 0;
}

int serial_port_frame_send_rc(u16 *rc)
{
	serial_port_limit(rc, 8);
	//only support 6 channels.
	//2 + 1 + 12 + 1 + 2
	int len_data = 16;
	int len = 23;
	u8 frame[len];
	frame[RC_POS_START1] = RC_BYTE_HEAD_1;
	frame[RC_POS_START2] = RC_BYTE_HEAD_2;
	frame[RC_POS_LEN] = len_data;
	memcpy(&frame[RC_POS_DATA], rc, len_data);
	u16 crc16 = crc16_value(frame, len_data + 3);
	frame[RC_POS_CRC1] = crc16 >> 8;
	frame[RC_POS_CRC2] = crc16 & 0xff;
	frame[RC_POS_END1] = RC_BYTE_END_1;
	frame[RC_POS_END2] = RC_BYTE_END_2;

	serial_port_puts(frame, len);
	return len;
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		while ((USART1->SR & USART_SR_RXNE) == 0)
		{
		}
		recv.buffer[recv.head] = USART_ReceiveData(USART1);
		recv.head = (recv.head + 1) % recv.size;
		if (recv.head == recv.tail)
		{
			recv.over++;
		}
	}
}
