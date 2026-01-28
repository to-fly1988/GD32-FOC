/******
@filename:focAlgorithm.h
@author:tofly Choo
@time:2026
******/

#ifndef __FOC_ALGORITHM_H__
#define __FOC_ALGORITHM_H__

#include <stdint.h>
#include <math.h>
#include "foc_tim_pwm.h"

#define SQRT3 1.73205f 		//根号三的数值
#define FOC_UDC 7.0f  		//母线电压
#define FOC_PWM_ARR 1000 	//svpwm的自动重载值（周期）
#define MAX_SPEED 15000		//电机最大转速
#define MAX_CURRENT 1.5f	//电机最大工作电流
#define OFFSET_ANGLE 293.0f 	//偏移角

typedef struct{

	float kp;		    //比例系数
	float ki;		    //积分系数
	float integral; //存储积分值
	float out_limit;	//输出限幅
	
}PidStatus;


//========================foc中所有状态值的结构体变量===========
typedef struct{
	uint8_t focEnable; //电机运行状态
	
	float targetSpeed; //目标转速
	float target_id;   //d轴参考电流
	float target_iq;   //q轴参考电流,速度环由PID计算得出iq
	
	//----反馈值------
	float ia;
	float ib;
	float ic;
	float speed;   //当前转速
	float theta_e; //电角度，电流环用
	float theta_m; //机械角度，速度环用
	
	//----中间状态值
	float id;
	float iq;
	float i_alpha;
	float i_beta;
	float ud;		//	反帕克变换输入
	float uq;
	float u_alpha;
	float u_beta;
	
	//----输出pwm值
	uint16_t pwm_a; //CCR值，用于比较输出
	uint16_t pwm_b; 
	uint16_t pwm_c; 
	
	//----PID状态
	PidStatus pid_speed;
	PidStatus pid_id;
	PidStatus pid_iq;

}FocStatus;


//====================对外函数===========================
void FOC_Init(volatile FocStatus *foc); //foc结构体中的参数初始化
void FOC_CURRENT_LOOP(volatile FocStatus *foc);//电流环运行
void FOC_SPEED_LOOP(volatile FocStatus *foc);	//速度环
void FOC_OPEN_LOOP(volatile FocStatus *foc);		//开环运行


#endif
