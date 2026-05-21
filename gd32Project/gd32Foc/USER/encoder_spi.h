/******
@name:encoder_spi.h
@author:tofly Choo
@time:2026
******/

#ifndef __ENCODER_SPI_H__
#define __ENCODER_SPI_H__

#include "gd32e50x.h"
#include <stdio.h>
#include "focAlgorithm.h"
#include "systick.h"


void foc_spi_init(void);				//spi初始化
uint8_t read_spi_byte(void);		//读取spi接收值,原始数据
float read_encoder_value(void);		//读取编码器值,返回角度值
float read_kongxin_spi(void);       //读取空心杯电机编码器值，返回角度值
float read_soft_spi_kth7112(void);      //GPIO软件模拟方式读取spi


#endif
