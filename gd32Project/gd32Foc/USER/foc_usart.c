/******
@filename:foc_usart.c
@author:tofly Choo
@version:2026-02-03
******/

#include "foc_usart.h"

__attribute__((aligned(4))) uint8_t u1_rx_buffer[RX_BUFFER_SIZE];					//数组起始地址对齐四字节
volatile uint32_t rx_len=0;

void usart1_init(void){

	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_USART1);
	
	gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_2);					//PA2为USART1的TX	
	gpio_init(GPIOA,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_3);		//PA3为USART1的RX
	
	/*配置USRT1的参数*/
	usart_deinit(USART1);																	//复位
	usart_baudrate_set(USART1,115200);										//设置波特率
	usart_word_length_set(USART1,USART_WL_8BIT);					//设置数据位8位
	usart_stop_bit_set(USART1,USART_STB_1BIT);						//设置停止位1位
	usart_receive_config(USART1,USART_RECEIVE_ENABLE);		//使能接收
	usart_transmit_config(USART1,USART_TRANSMIT_ENABLE);	//使能发送
	
	/*使能DMA和中断*/
	usart_dma_receive_config(USART1,USART_RECEIVE_DMA_ENABLE);	//使能DMA接收功能
	usart_interrupt_enable(USART1,USART_INT_IDLE);							//串口空闲中断
	nvic_irq_enable(USART1_IRQn,0,2);														//配置中断优先级
	
	usart_enable(USART1);																				//使能串口

}


void usart1_rx_dma_init(void){
	/*usart1_rx<---->dma0_ch5*/
	rcu_periph_clock_enable(RCU_DMA0);
	dma_parameter_struct dma0_init_struct;
	
	dma_deinit(DMA0,DMA_CH5);
	dma0_init_struct.direction=DMA_PERIPHERAL_TO_MEMORY;				//数据传输方向
	dma0_init_struct.periph_addr=(uint32_t)&USART_DATA(USART1);	//外设基地址
	dma0_init_struct.memory_addr=(uint32_t)u1_rx_buffer;				//存储器基地址
	dma0_init_struct.periph_inc=DMA_MEMORY_INCREASE_DISABLE;		//外设地址生成增量模式
	dma0_init_struct.memory_inc=DMA_MEMORY_INCREASE_ENABLE;			//存储器地址生成增量模式
	dma0_init_struct.periph_width=DMA_MEMORY_WIDTH_8BIT;				//外设数据传输宽度
	dma0_init_struct.memory_width=DMA_MEMORY_WIDTH_8BIT;				//存储器数据传输宽度
	
	dma_init(DMA0,DMA_CH5,&dma0_init_struct);										//初始化DMA配置
	
	/*DMA模式配置*/
	dma_circulation_disable(DMA0,DMA_CH5);										//关闭DMA循环模式
	dma_memory_to_memory_disable(DMA0,DMA_CH5);								//禁用内存到内存模式（即软件触发DMA）
	dma_transfer_number_config(DMA0,DMA_CH5,RX_BUFFER_SIZE);	//传输数据位数	
	dma_channel_enable(DMA0,DMA_CH5);													//使能DMA通道
	
}


/*调用串口发送数据*/
void usart1_send_byte(uint8_t data_byte){

	while(RESET==usart_flag_get(USART1,USART_FLAG_TBE));
	usart_data_transmit(USART1,data_byte);

}


/*旧版发送justfloat格式数据*/
void vofa_send_data(float ch1, float ch2,float ch3,float ch4,float ch5,float ch6, float ch7,float ch8,float ch9){

	float send_data[9];
	send_data[0]=ch1;
	send_data[1]=ch2;
	send_data[2]=ch3;
	send_data[3]=ch4;
	send_data[4]=ch5;
	send_data[5]=ch6;
	send_data[6]=ch7;
	send_data[7]=ch8;
	send_data[8]=ch9;

	uint8_t *p = (uint8_t *)send_data;
  for (uint16_t i = 0; i < sizeof(send_data); i++) {
    usart1_send_byte(p[i]); // 调用串口重定向底层函数
    }
	usart1_send_byte(0x00);
	usart1_send_byte(0x00);
	usart1_send_byte(0x80);
	usart1_send_byte(0x7F);
}


/*新版发送justfloat格式数据，使用数组形式，优化了增删改数据功能*/
void vofa_send_array(float *data,uint8_t num){
	
	/*定义联合体，方便浮点数转成字节形式发送*/
	FloatToByte conv_data;

	/*按字节发送数据*/
	for(uint8_t i=0;i<num;i++){
		conv_data.f=data[i];
		usart1_send_byte(conv_data.b[0]);
		usart1_send_byte(conv_data.b[1]);
		usart1_send_byte(conv_data.b[2]);
		usart1_send_byte(conv_data.b[3]);
	}
	
	/*发送帧尾*/
	usart1_send_byte(0x00);
	usart1_send_byte(0x00);
	usart1_send_byte(0x80);
	usart1_send_byte(0x7F);
}


/*串口接收的指令解析*/
void cmd_rx_decode(uint8_t cmd,float value,volatile FocStatus *foc){

	switch(cmd){
		
		case 0x01:
			foc->targetSpeed=value;
			break;
		
		case 0x02:
			foc->target_id=value;
			break;
		
		case 0x03:
			foc->target_iq=value;
			break;
		
		case 0xA1:
			foc->pid_speed.kp=value;
			break;
		case 0xA2:
			foc->pid_speed.ki=value;
			break;
		
		case 0xB1:
			foc->pid_id.kp=value;
			break;
		case 0xB2:
			foc->pid_id.ki=value;
			break;
		
		case 0xC1:
			foc->pid_iq.kp=value;
			break;
		case 0xC2:
			foc->pid_iq.ki=value;
			break;
		
		default:
			break;
	}

}
