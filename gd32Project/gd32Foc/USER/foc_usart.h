/******
foc_usart.h
author:tofly Choo
time:2026
******/

#ifndef __FOC_USART_H__
#define __FOC_USART_H__

#include "gd32e50x.h"
#include <stdio.h>
#include "focAlgorithm.h"

#define RX_BUFFER_SIZE 32
extern uint8_t u1_rx_buffer[RX_BUFFER_SIZE];

typedef struct{
	uint8_t header;		//帧头
	uint8_t cmd_code;	//功能码
	
	/*负载数据的联合体*/
	union{
		float 		f_value;
		uint32_t 	value;
		uint8_t		byte_value;
	} payload;
	
	uint8_t footer;		//帧尾

} RX_Frame;

typedef union{
	float f;
	uint8_t b[4];
}FloatToByte;

void usart1_init(void);
void usart1_send_byte(uint8_t data_byte);
void usart1_rx_dma_init(void);
void vofa_send_data(float ch1, float ch2,float ch3,float ch4,float ch5,float ch6, float ch7,float ch8,float ch9);
void vofa_send_array(float *data,uint8_t num);
void cmd_rx_decode(uint8_t cmd,float value,volatile FocStatus *foc);

#endif
