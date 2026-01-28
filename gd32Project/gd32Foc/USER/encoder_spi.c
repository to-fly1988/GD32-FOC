#include "encoder_spi.h"

void foc_spi_init(void){

  /*gpio引脚配置*/
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_SPI0);
	gpio_init(GPIOA,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_4);				//PA4--->SPI0-NSS
	gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5);				//PA5--->SPI0-SCK
	gpio_init(GPIOA,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_6);	//PA6--->SPI0-MISO
	/*MCU不向编码器发送数据，无需配置MOSI*/
	
	/*SPI配置*/
	spi_parameter_struct spi0_init_struct;
	spi_i2s_deinit(SPI0);
	spi0_init_struct.trans_mode  					= SPI_TRANSMODE_FULLDUPLEX;	//全双工模式
	spi0_init_struct.device_mode 					= SPI_MASTER;	//主机模式
	spi0_init_struct.frame_size  					= SPI_FRAMESIZE_8BIT;	//8bit数据传输
	spi0_init_struct.nss         					= SPI_NSS_SOFT;	//软件控制NSS
	spi0_init_struct.prescale		          = SPI_PSC_128;	//预分频
	spi0_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	spi0_init_struct.endian			 					= SPI_ENDIAN_MSB;	//
	spi_init(SPI0,&spi0_init_struct);
	spi_enable(SPI0);		//使能SPI0
	
}

uint8_t read_spi_byte(void){
	// printf("enter spi\n");
uint32_t timeout = 0;	
	/*等待发送缓冲区清零*/
	while(RESET==spi_i2s_flag_get(SPI0,SPI_FLAG_TBE)){
	if(++timeout > 1000) {
            return 0x00; // 超时退出！避免死锁
	}
}
	spi_i2s_data_transmit(SPI0,0xFF);
	// printf("TBE ok\n");  
	timeout=0;
	while(RESET==spi_i2s_flag_get(SPI0,SPI_FLAG_RBNE)){
	if(++timeout > 1000) {
            return 0x00; // 超时退出！避免死锁
	}
	}
	//  printf("RBNE ok\n");   
	return spi_i2s_data_receive(SPI0);

}

float read_encoder_ssi(void){
	uint8_t rx1,rx2,rx3;
	uint32_t raw_rx;
	uint16_t raw_angle;
	gpio_bit_reset(GPIOA,GPIO_PIN_4);
	for(volatile int i=0;i<50;i++);
	rx1=read_spi_byte();
	rx2=read_spi_byte();
	rx3=read_spi_byte();
	gpio_bit_set(GPIOA,GPIO_PIN_4);
	raw_rx=((uint32_t)rx1 << 16) | ((uint32_t)rx2 << 8) | rx3;
	raw_angle=(raw_rx>>9)&0x3FFF;
	//printf("rawangle=%d\n",raw_angle);
	float angle=raw_angle*360.0f/16384.0f;
//	printf("rx_encoder=%.3f\n",rx_encoder);
	return angle;
}
