/******
pwm_out.h
author:tofly Choo
time:2026
******/

#ifndef __FOC_TIM_PWM_H__
#define __FOC_TIM_PWM_H__

#include "gd32e50x.h"
#include <stdio.h>
#include "gd32e50x_timer.h"
#include "focAlgorithm.h"

void pwm_init(void);		//定时器及PWM初始化配置
void pwm_crr_setA(uint16_t pwm_crr);    //设置A相的PWM比较值
void pwm_crr_setB(uint16_t pwm_crr);    //设置B相的PWM比较值
void pwm_crr_setC(uint16_t pwm_crr);    //设置C相的PWM比较值
void timer1_it_init(void);


#endif
