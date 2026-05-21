#include "encoder_spi.h"
#include "gd32e50x_spi.h"

// CS 引脚
#define CS_LOW()     gpio_bit_reset(GPIOA, GPIO_PIN_4)
#define CS_HIGH()    gpio_bit_set(GPIOA, GPIO_PIN_4)

// SCK 引脚
#define SCK_LOW()    gpio_bit_reset(GPIOA, GPIO_PIN_5)
#define SCK_HIGH()   gpio_bit_set(GPIOA, GPIO_PIN_5)

// SDA 数据线引脚 (MOSI和MISO的结合体)
#define SDA_LOW()    gpio_bit_reset(GPIOA, GPIO_PIN_7)
#define SDA_HIGH()   gpio_bit_set(GPIOA, GPIO_PIN_7)
#define SDA_READ()   gpio_input_bit_get(GPIOA, GPIO_PIN_7)

// 切换 SDA 为推挽输出 (单片机发送)
#define SDA_TRANSMIT()  gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7)

// 切换 SDA 为上拉输入 (单片机接收) - 注意这里推荐用带上拉的输入，增强抗干扰
#define SDA_RECEIVE()   gpio_init(GPIOA,GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_7)

static void MT6701_spi_init(void){

  /*gpio引脚配置*/
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_SPI0);
	gpio_init(GPIOA,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_4);			//PA4--->SPI0-NSS
	gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5);			//PA5--->SPI0-SCK
	gpio_init(GPIOA,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_6);		//PA6--->SPI0-MISO
	/*MCU不向编码器发送数据，无需配置MOSI*/
	
	/*SPI配置*/
	spi_parameter_struct spi0_init_struct;
	spi_i2s_deinit(SPI0);
	spi0_init_struct.trans_mode  					= SPI_TRANSMODE_FULLDUPLEX;	//全双工模式
	spi0_init_struct.device_mode 					= SPI_MASTER;								//主机模式
	spi0_init_struct.frame_size  					= SPI_FRAMESIZE_8BIT;				//8bit数据传输
	spi0_init_struct.nss         					= SPI_NSS_SOFT;							//软件控制NSS
	spi0_init_struct.prescale		          = SPI_PSC_128;							//预分频
	spi0_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;		//MODEL1
	spi0_init_struct.endian			 					= SPI_ENDIAN_MSB;						//高位在前
	spi_init(SPI0,&spi0_init_struct);																	//SPI0初始化
	spi_enable(SPI0);																									//使能SPI0
	
}

static void KTH7112_spi_init(void){

  /*gpio引脚配置*/
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_SPI0);
	gpio_init(GPIOA,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_4);			//PA4--->SPI0-NSS
	gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5);			//PA5--->SPI0-SCK
	//gpio_init(GPIOA,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_6);
	gpio_init(GPIOA,GPIO_MODE_AF_OD,GPIO_OSPEED_50MHZ,GPIO_PIN_7);			//PA7--->SPI0-MOSI
	gpio_bit_set(GPIOA,GPIO_PIN_4);
	/*KKTH7112使用三线制SPI,MCU的MOSI需要设置为双向传输*/
	
	/*SPI配置*/
	spi_parameter_struct spi0_init_struct;
	spi_i2s_deinit(SPI0);
	spi0_init_struct.trans_mode  					= SPI_TRANSMODE_FULLDUPLEX;	//双工模式
	spi0_init_struct.device_mode 					= SPI_MASTER;								//主机模式
	spi0_init_struct.frame_size  					= SPI_FRAMESIZE_8BIT;				//8bit数据传输
	spi0_init_struct.nss         					= SPI_NSS_SOFT;							//软件控制NSS
	spi0_init_struct.prescale		          = SPI_PSC_128;							//预分频
	spi0_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;	//MODE3空心杯电机KTH7112
	spi0_init_struct.endian			 					= SPI_ENDIAN_MSB;						//高位在前
	spi_init(SPI0,&spi0_init_struct);																	//SPI0初始化
	spi_enable(SPI0);																									//使能SPI0
	
}

static void soft_spi_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_SPI0);
	gpio_init(GPIOA,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_4);			//PA4--->SPI0-NSS
	gpio_init(GPIOA,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5);			//PA5--->SPI0-SCK
	SDA_TRANSMIT();

	CS_HIGH();
	SCK_HIGH();
	SDA_HIGH();
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

static float read_mt6701_ssi(void){
	uint8_t rx1,rx2,rx3;
	uint32_t raw_rx;
	uint16_t raw_angle;
	gpio_bit_reset(GPIOA,GPIO_PIN_4);		//拉低CS片选信号
	for(volatile int i=0;i<50;i++);			//简单延时
	rx1=read_spi_byte();
	rx2=read_spi_byte();
	rx3=read_spi_byte();
	gpio_bit_set(GPIOA,GPIO_PIN_4);
	raw_rx=((uint32_t)rx1 << 16) | ((uint32_t)rx2 << 8) | rx3;
	raw_angle=(raw_rx>>9)&0x3FFF;				//移位只读取角度值
	float angle=raw_angle*360.0f/16384.0f;
	return angle;
}

float read_kongxin_spi(void)
{
	uint8_t rx1,rx2,rx_crc;
	uint16_t raw_rx;
	float raw_angle;

	spi_bidirectional_transfer_config(SPI0,SPI_BIDIRECTIONAL_TRANSMIT);
	gpio_bit_reset(GPIOA,GPIO_PIN_4);

	for(volatile int i=0;i<50;i++);

	while (spi_i2s_flag_get(SPI0,SPI_FLAG_TBE)==RESET);
	spi_i2s_data_transmit(SPI0,0x00);
	while(spi_i2s_flag_get(SPI0,SPI_FLAG_RBNE)==RESET);
	spi_i2s_data_receive(SPI0);

	while(spi_i2s_flag_get(SPI0,SPI_FLAG_TRANS)!=RESET);
	spi_bidirectional_transfer_config(SPI0,SPI_BIDIRECTIONAL_RECEIVE);

	rx1=read_spi_byte();
	rx2=read_spi_byte();
	rx_crc=read_spi_byte();

	//spi_bidirectional_transfer_config(SPI0,SPI_BIDIRECTIONAL_TRANSMIT);
	gpio_bit_set(GPIOA,GPIO_PIN_4);
	raw_rx=(uint16_t)( ((uint16_t)rx1 << 8) | rx2 );
	raw_angle=raw_rx*360.0f/65536.0f;
	return raw_rx;
}

static void spi_delay (void)
{
	for (volatile int i=0;i<10;i++);
}

static void softSpi_transByte(uint8_t data)
{
	SDA_TRANSMIT();
	for (int i=0;i<8 ;i++)
	{ 
		SCK_LOW();
		if(data&0x80)
		{
			SDA_HIGH();
		}
		else
		{
			SDA_LOW();
		}
		spi_delay();
		SCK_HIGH();
		spi_delay();
		data=data<<1;
	}

}

static uint8_t softSpi_readByte(void)
{
	uint8_t data=0;
	SDA_RECEIVE();
	for (int i=0;i<8;i++)
	{
		SCK_LOW();
		spi_delay();
		data=data<<1;
		SCK_HIGH();
		if (SDA_READ()!=RESET)
		{
			data|=0x01;
		}
		spi_delay();
	}
	return data;
}

float read_soft_spi_kth7112(void)
{
	uint8_t rx1,rx2,rxcrc;
	uint16_t raw_angle;

	CS_LOW();
	spi_delay();	//片选拉低后要等待一段时间延时再传输数据
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();

	softSpi_transByte(0x11);
	softSpi_transByte(0x0d);
	SDA_RECEIVE();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	spi_delay();
	

	rx1=softSpi_readByte();
	rx2=softSpi_readByte();
	//rxcrc=softSpi_readByte();

	CS_HIGH();
	SDA_TRANSMIT();
	SDA_HIGH();

	raw_angle=(uint16_t)( ((uint16_t)rx1 << 8) | rx2 );
	return raw_angle;
}

void foc_spi_init(void){

	// KTH7112_spi_init();	//KTH7112编码器
	soft_spi_init();	//软件模拟spi初始化
}
 
float read_encoder_value(void)
{
	return read_mt6701_ssi();

}