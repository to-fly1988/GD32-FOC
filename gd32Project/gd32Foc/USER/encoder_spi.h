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
#include "gd32e50x_spi.h"

void foc_spi_init(void);		//spi初始化
uint8_t read_spi_byte(void);		//读取spi接收值
float read_encoder_ssi(void);	//读取编码器值


#endif
