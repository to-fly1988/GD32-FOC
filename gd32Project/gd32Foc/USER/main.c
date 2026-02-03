#include  "gd32e50x.h"
#include <stdio.h>
#include "systick.h"
#include "focAlgorithm.h"
#include "foc_tim_pwm.h"
#include "foc_usart.h"
#include "main.h"
#include "foc_ADC.h"
#include "encoder_spi.h"
volatile FocStatus myfoc;
//volatile float temp_angle=0;



int main(){
myfoc.focEnable=0;	
/*所有外设硬件初始化*/
systick_config();
foc_spi_init();
myfoc.theta_m=read_encoder_ssi();
pwm_init();
usart1_rx_dma_init();		//开启DMA
usart1_init();

timer1_it_init();

foc_adc_init();


//printf("start ok");
/*foc初始参数*/
myfoc.ud=0;
myfoc.uq=1.5f;
FOC_Init(&myfoc);		//参数初始化
foc_current_offset(&myfoc);		//电流零位自校准，获取0电流时的基准量

//myfoc.u_alpha=3;
//myfoc.u_beta=0;
//myfoc.theta_e=0;
//myfoc.targetSpeed=2000;//设定参考转速
myfoc.focEnable=1; 




while(1)
	{
		/*串口待发送数据*/
		float tx_data[]={

			myfoc.speed,
	//		myfoc.theta_m,
			//myfoc.targetSpeed,
			myfoc.ia,
			myfoc.ib,
			myfoc.ic,
//			myfoc.id,
//			myfoc.iq,
			myfoc.i_alpha,
			myfoc.i_beta
//			myfoc.pid_speed.integral,
//			myfoc.pid_iq.integral
		};
		vofa_send_array(tx_data,sizeof(tx_data)/sizeof(float));
		//vofa_send_data(myfoc.speed,myfoc.targetSpeed,myfoc.ia,myfoc.ib,myfoc.ic,myfoc.id,myfoc.iq,myfoc.pid_speed.integral,myfoc.pid_iq.integral);
		//vofa_send_data(myfoc.speed,myfoc.theta_m,myfoc.ia,myfoc.ib,myfoc.ic,myfoc.id,myfoc.iq,myfoc.i_alpha,myfoc.i_beta);
//		printf("%.3f\n",myfoc.theta_m);
//		printf("speed=%.3f\n",myfoc.speed);
		//printf("hello\n");
		delay_1ms(10);	
}
	
}


#ifdef GD_ECLIPSE_GCC
/* retarget the C library printf function to the USART, in Eclipse GCC environment */
int __io_putchar(int ch)
{
    usart_data_transmit(USART0, (uint8_t) ch );
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
#else
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t)ch);
    while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));

    return ch;
}
#endif /* GD_ECLIPSE_GCC */
