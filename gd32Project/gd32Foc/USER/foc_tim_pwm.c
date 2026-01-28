#include "foc_tim_pwm.h"


void pwm_init(void){
	
	timer_parameter_struct timer0_init; //定时器和通道配置的结构体变量
	timer_oc_parameter_struct ch_config;
	
	
	//------GPIO配置----
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_AF);
	
	///PA8,PA9,PA10对应定时器TIMER0的CH0,CH1,CH2
	gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
	gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
	gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);
	
	//-----定时器配置-----
	rcu_periph_clock_enable(RCU_TIMER0);
	timer_deinit(TIMER0);
	
	
	timer0_init.prescaler=18-1; //预分频系数,系统时钟180MHZ
	timer0_init.alignedmode=TIMER_COUNTER_CENTER_BOTH;  //计数模式
	//timer0_init.alignedmode=TIMER_COUNTER_EDGE;
	
	timer0_init.counterdirection=(TIMER_COUNTER_UP);  //计数方向
	timer0_init.period=FOC_PWM_ARR/2-1;		//周期
	timer0_init.clockdivision=TIMER_CKDIV_DIV1;  //
	timer0_init.repetitioncounter=1;									//RCR设为1，配合ADC触发，
	timer_init(TIMER0,&timer0_init);
	
	ch_config.outputstate=TIMER_CCX_ENABLE;    //输出使能
	ch_config.outputnstate=TIMER_CCXN_DISABLE;	//互补通道输出状态
	ch_config.ocpolarity=TIMER_OC_POLARITY_HIGH;	//输出极性
	ch_config.ocnpolarity=TIMER_OCN_POLARITY_HIGH;
	ch_config.ocidlestate=TIMER_OC_IDLE_STATE_LOW;
	ch_config.ocnidlestate=TIMER_OCN_IDLE_STATE_LOW;
	timer_channel_output_config(TIMER0,TIMER_CH_0,&ch_config);
	timer_channel_output_config(TIMER0,TIMER_CH_1,&ch_config);
	timer_channel_output_config(TIMER0,TIMER_CH_2,&ch_config);
	
	//CH0通道配置输出模式
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,499);
	timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM1);
	timer_channel_output_shadow_config (TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);
	
	//CH1
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,499);
	timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM1);
	timer_channel_output_shadow_config (TIMER0,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);
	
	//CH2
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,499);
	timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_PWM1);
	timer_channel_output_shadow_config (TIMER0,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
	
	
	timer_auto_reload_shadow_enable(TIMER0);
	timer_master_output_trigger_source_select(TIMER0,TIMER_TRI_OUT_SRC_UPDATE);		//使用更新事件作为TRGO，此项用于ADC触发
	timer_primary_output_config(TIMER0, ENABLE); //使用高级定时器所有通道都要使能打开
	timer_enable(TIMER0);
}
	
void pwm_crr_setA(uint16_t pwm_crr){

	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,pwm_crr);
	
}

void pwm_crr_setB(uint16_t pwm_crr){

	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,pwm_crr);
	
}

void pwm_crr_setC(uint16_t pwm_crr){

	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,pwm_crr);
	
}


/*速度环定时器设置*/
void timer1_it_init(void){
	timer_parameter_struct timer1_init;
	rcu_periph_clock_enable(RCU_TIMER1);
	timer_deinit(TIMER1);
	timer1_init.prescaler=180-1;
	timer1_init.alignedmode=TIMER_COUNTER_EDGE;
	timer1_init.counterdirection=TIMER_COUNTER_UP;
	timer1_init.period=1000-1;
	timer1_init.clockdivision=TIMER_CKDIV_DIV1;
	timer1_init.repetitioncounter=0;
	timer_init(TIMER1,&timer1_init);
	
	timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);
	timer_interrupt_enable(TIMER1,TIMER_INT_UP);
	
	/*NVIC配置*/
	nvic_irq_enable(TIMER1_IRQn,0,1);
	timer_enable(TIMER1);

}
	
	
	
	
	
	

