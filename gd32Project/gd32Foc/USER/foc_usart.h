/******
foc_usart.h
author:tofly Choo
time:2026
******/

#ifndef __FOC_USART_H__
#define __FOC_USART_H__

#include "gd32e50x.h"
#include <stdio.h>

void usart1_init(void);
void usart1_send_byte(uint8_t data_byte);
void vofa_send_data(float ch1, float ch2,float ch3,float ch4,float ch5,float ch6, float ch7,float ch8,float ch9);

#endif