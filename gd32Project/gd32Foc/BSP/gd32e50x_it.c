/*!
    \file    gd32e50x_it.c
    \brief   interrupt service routines

    \version 2025-07-09, V1.6.1, firmware for GD32E50x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e50x_it.h"
#include "main.h"
#include "string.h"
#include "systick.h"
#include "foc_ADC.h"
#include "encoder_spi.h"
#include "focAlgorithm.h"
#include "foc_usart.h"



/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    /* if NMI exception occurs, go to infinite loop */
    while(1) {
    }
}
/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
    /* if SVC exception occurs, go to infinite loop */
    while(1) {
    }
}
/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
    /* if PendSV exception occurs, go to infinite loop */
    while(1) {
    }
}
/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
   // led_spark();
    delay_decrement();
}


extern volatile FocStatus myfoc;
extern volatile float temp_angle;

void ADC0_1_IRQHandler(void){

		adc_interrupt_flag_clear(ADC0,ADC_INT_FLAG_EOIC);		//清除中断标志位
	
		/*ADC采样电流*/
		uint16_t adc_value1=adc_inserted_data_read(ADC0,ADC_INSERTED_CHANNEL_0);		//读取注入组通道0的值
		uint16_t adc_value2=adc_inserted_data_read(ADC0,ADC_INSERTED_CHANNEL_1);
//		myfoc.ia=adc_value1*3.2f/4096;		//采样值转化为实际电压
//		myfoc.ib=adc_value2*3.2f/4096;
//		myfoc.ia=(myfoc.ia-1.645f)*2.0f;	//换算成电流值
//		myfoc.ib=(myfoc.ib-1.645f)*2.0f;
		myfoc.ia=(float)-(adc_value1-myfoc.ia_offset)*3.3f/4096*2.0f;
		myfoc.ib=(float)-(adc_value2-myfoc.ib_offset)*3.3f/4096*2.0f;
		myfoc.ic=-myfoc.ia-myfoc.ib;
	
		
		/*读编码器，电角度更新*/
		//myfoc.theta_e=read_encoder_ssi()*0.017453f*1;	//第一个数字，转化为弧度制；第二个乘的数字为极对数
		//myfoc.theta_e=293.0f-OFFSET_ANGLE;
		myfoc.theta_e=read_encoder_ssi()-OFFSET_ANGLE;
		if (myfoc.theta_e < 0) {
			myfoc.theta_e += 360.0f;
		} else if (myfoc.theta_e >= 360.0f) {
			myfoc.theta_e -= 360.0f;
		}
		myfoc.theta_e=myfoc.theta_e*0.017453f*1;	//第一个数字，转化为弧度制；第二个乘的数字为极对数
		//myfoc.theta_e=temp_angle*0.017453f*1;
		
		/*运行电流环*/
		FOC_CURRENT_LOOP(&myfoc);
		/*运行开环程序*/
		//FOC_OPEN_LOOP(&myfoc);
	
}



void TIMER1_IRQHandler(void){
	  static uint8_t time_count=0;	//计数次数，每10次触发一次位置环计算
		time_count++;
		timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);	//清除中断标志位
		
		/*读编码器并计算速度值RPM*/
		float angle=read_encoder_ssi();
		float delta_angle=angle-myfoc.theta_m;
		if (delta_angle > 180.0f) {
				delta_angle -= 360.0f;
		} else if (delta_angle < -180.0f) {
				delta_angle += 360.0f;
		}
		myfoc.speed=delta_angle*166.6667f;	//166.6667=1/0.001/360*60
		myfoc.theta_m=angle;
		
		/*运行速度环*/
		FOC_SPEED_LOOP(&myfoc);
		
		/*运行位置环*/
		if(time_count==10){
			time_count=0;
			FOC_POSITION_LOOP(&myfoc);
		}
		
	//printf("end int\n");
}
		


void USART1_IRQHandler(void){

	if(RESET!=usart_interrupt_flag_get(USART1,USART_INT_FLAG_IDLE)){
		
		/* 顺序读取 STAT0 和 DATA，清除 IDLE 标志位,读STAT0已在if语句中操作 */
	  (void)usart_data_receive(USART1);		//假装读DATA，目的是彻底清除IDLE中断标志
		
		dma_channel_disable(DMA0,DMA_CH5);
		
		
		rx_len=RX_BUFFER_SIZE-dma_transfer_number_get(DMA0,DMA_CH5);
		
		if(rx_len==7){
			//RX_Frame *ptr=(RX_Frame *)u1_rx_buffer;		//buffer中的原始数据转变为帧协议格式
			
			if(u1_rx_buffer[0]==0xAA && u1_rx_buffer[6]==0xFF){
				float fvalue;
				memcpy(&fvalue,&u1_rx_buffer[2],4);		//数据位负值给fvalue变量
				cmd_rx_decode(u1_rx_buffer[1],fvalue ,&myfoc);	//指令解析
			}	
		}
		
		dma_transfer_number_config(DMA0,DMA_CH5,RX_BUFFER_SIZE);
		dma_channel_enable(DMA0,DMA_CH5);
		
	}
}

