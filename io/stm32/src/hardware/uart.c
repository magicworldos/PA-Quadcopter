/*
 * uart.c
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#include <uart.h>
#include <crc.h>

struct uart_buffer_s _recv;
u8 _packet_buffer[BUFFER_SIZE];
u16 motor[6] = { 0, 0, 0, 0, 0, 0 };

void UART_Init()
{
	UART1_GPIO_Configuration();
	UART1_Configuration();

	_recv.head = 0;
	_recv.tail = 0;
	_recv.size = BUFFER_SIZE;
	memset(_recv.buffer, 0x00, BUFFER_SIZE);
}

void UART1_GPIO_Configuration(void)
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

void UART1_Configuration(void)
{
	USART_InitTypeDef usart1_init_struct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	USART_Cmd(USART1, ENABLE);
	usart1_init_struct.USART_BaudRate = 115200;
	usart1_init_struct.USART_WordLength = USART_WordLength_8b;
	usart1_init_struct.USART_StopBits = USART_StopBits_1;
	usart1_init_struct.USART_Parity = USART_Parity_No;
	usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &usart1_init_struct);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
}

u8 Uart1_PutChar(u8 ch)
{
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	//USART_GetFlagStatus(USART1, USART_FLAG_TC);
	USART_SendData(USART1, (u8) ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
	{
	}
	return ch;
}

void Uart1_PutString(u8* buf, u8 len)
{
	for (u8 i = 0; i < len; i++)
	{
		Uart1_PutChar(*buf++);
	}
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		while ((USART1->SR & USART_SR_RXNE) == 0)
		{
		}
		_recv.buffer[_recv.head] = USART_ReceiveData(USART1);
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
			case PWM_HEAD:
				if (packet_index >= 2)
				{
					if ((_packet_buffer[PWM_POS_START1] == PWM_BYTE_HEAD_1) && (_packet_buffer[PWM_POS_START2] == PWM_BYTE_HEAD_2))
					{
						parse_packet_step = PWM_LEN;
					}
					else if (_packet_buffer[PWM_POS_START2] == PWM_BYTE_HEAD_1)
					{
						_packet_buffer[PWM_POS_START1] = PWM_BYTE_HEAD_1;
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
					Frame_data_len = _packet_buffer[PWM_POS_LEN];
					FrameLen = Frame_data_len + 6;
					if (FrameLen <= BUFFER_SIZE)
					{
						parse_packet_step = PWM_END;
					}
					else
					{
						parse_packet_step = PWM_HEAD;
						packet_index = 0;
						memset(_packet_buffer, 0x00, sizeof(_packet_buffer));
					}
				}
				break;

			case PWM_END:
				if (packet_index == FrameLen)
				{
					u16 crc16 = _packet_buffer[PWM_POS_CRC1] << 8 | _packet_buffer[PWM_POS_CRC2];
					if (crc16_check(&_packet_buffer[PWM_POS_START1], Frame_data_len + 3, crc16))
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
		tail = (tail + 1) % _recv.size;
	}
	if (ret == 1)
	{
		_recv.tail = tail;
	}
	return ret;
}

int get_motor_pwm()
{
	if (parse_mag_feedback())
	{
		memcpy(motor, &_packet_buffer[PWM_POS_DATA], sizeof(u16) * 4);
		return 1;
	}
	return 0;
}
