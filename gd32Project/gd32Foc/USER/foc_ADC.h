/******
foc_ADC.h
author:tofly Choo
time:2026
******/

#ifndef __FOC_ADC_H__
#define __FOC_ADC_H__

#include "gd32e50x.h"
#include <stdio.h>
#include "gd32e50x_timer.h"
#include "gd32e50x_adc.h"
#include "focAlgorithm.h"
#include "systick.h"

void foc_adc_init(void);    //ADC初始化，包含GPIO,ADC
void foc_current_offset(volatile FocStatus *foc);	//采样电流校准



#endif
