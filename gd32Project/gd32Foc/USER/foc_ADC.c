#include "foc_ADC.h"


void foc_adc_init(void){

	/*GPIO配置*/
	rcu_periph_clock_enable(RCU_GPIOA);
	//gpio_init(GPIOA,GPIO_MODE_AIN,GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1);  //配置PA0,PA1作为ADC模拟输入口,对应ADCIN0,ADCIN1
	gpio_init(GPIOA,GPIO_MODE_AIN,GPIO_OSPEED_50MHZ,GPIO_PIN_1);
	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_init(GPIOA,GPIO_MODE_AIN,GPIO_OSPEED_50MHZ,GPIO_PIN_0); 
	/*ADC配套定时器配置TIMER0*/
	/*定时器配置在pwm_out.c中*/
	
	
	/*ADC配置*/
	rcu_periph_clock_enable(RCU_ADC0);
	rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);		//ADC时钟六分频，30MHZ
	adc_mode_config(ADC_MODE_FREE);									//自由模式,即所有ADC独立运行
	adc_special_function_config(ADC0,ADC_SCAN_MODE,ENABLE);						//使能扫描模式
	adc_special_function_config(ADC0,ADC_CONTINUOUS_MODE,DISABLE);    //关闭连续模式
	adc_data_alignment_config(ADC0,ADC_DATAALIGN_RIGHT);							//数据右对齐
	adc_channel_length_config(ADC0,ADC_INSERTED_CHANNEL,2);						//2个注入组通道
	adc_inserted_channel_config(ADC0,0,ADC_CHANNEL_8,ADC_SAMPLETIME_13POINT5);		//通道0采样周期设置
	adc_inserted_channel_config(ADC0,1,ADC_CHANNEL_1,ADC_SAMPLETIME_13POINT5);		//通道1采样周期设置
	adc_external_trigger_source_config(ADC0,ADC_INSERTED_CHANNEL,ADC0_1_EXTTRIG_INSERTED_T0_TRGO);	//外部触发源设置
	adc_external_trigger_config(ADC0,ADC_INSERTED_CHANNEL,ENABLE);		//外部触发使能
	adc_interrupt_flag_clear(ADC0,ADC_INT_FLAG_EOC);		//清除EOC标志位,该标志置1表示一次扫描完成
	adc_interrupt_flag_clear(ADC0,ADC_INT_FLAG_EOIC);		//清除EOIC标志位，该标志位用于触发中断
	adc_interrupt_enable(ADC0,ADC_INT_EOIC);						//使能EOIC的中断
	adc_enable(ADC0);		//使能ADC0开启
	delay_1ms(1);				//设置1ms延迟，使ADC处于稳定状态
	adc_calibration_enable(ADC0);  //使能校准复位
	
	/*NVIC配置*/
	nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);		//配置优先组长度
	nvic_irq_enable(ADC0_1_IRQn,0,0);										//配置ADC0和1的全局中断优先级
	

}

void foc_current_offset(volatile FocStatus *foc){
	
	uint32_t sum_a=0;
  uint32_t sum_b=0;
	for(int i=0;i<1000;i++){
	
		sum_a=sum_a+adc_inserted_data_read(ADC0,ADC_INSERTED_CHANNEL_0);
		sum_b=sum_b+adc_inserted_data_read(ADC0,ADC_INSERTED_CHANNEL_1);
		
	}
	
	foc->ia_offset=(float)sum_a/1000;
	foc->ib_offset=(float)sum_b/1000;	

}
