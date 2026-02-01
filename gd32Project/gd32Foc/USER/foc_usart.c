#include "foc_usart.h"

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
	usart_enable(USART1);																	//使能串口
	

}

void usart1_rx_dma_init(void){

	rcu_periph_clock_enable(RCU_DMA0);
	dma_parameter_struct dma0_init_struct;
	
	dma_deinit(DMA0,DMA_CH5);
	dma0_init_struct.direction=DMA_PERIPHERAL_TO_MEMORY;	//数据传输方向
	dma0_init_struct.periph_addr=(uint32_t)&USART_DATA(USART1);	//外设基地址
	dma0_init_struct.memory_addr=;
	dma0_init_struct.periph_inc=DMA_MEMORY_INCREASE_ENABLE;	//外设地址生成增量模式
	dma0_init_struct.memory_inc=DMA_MEMORY_INCREASE_ENABLE;	//存储器地址生成增量模式
	dma0_init_struct.periph_width=DMA_MEMORY_WIDTH_8BIT;	//外设数据传输宽度
	dma0_init_struct.memory_width=DMA_MEMORY_WIDTH_8BIT;	//存储器数据传输宽度
	
	
	

}

void usart1_send_byte(uint8_t data_byte){

	while(RESET==usart_flag_get(USART1,USART_FLAG_TBE));
	usart_data_transmit(USART1,data_byte);

}

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
    usart1_send_byte(p[i]); // 调用你的串口重定向底层函数
    }
	usart1_send_byte(0x00);
	usart1_send_byte(0x00);
	usart1_send_byte(0x80);
	usart1_send_byte(0x7F);
}