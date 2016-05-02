#include "sys.h"
#include "usart.h"
#include "PWM_Output.h"
#include "main.h"
#include "rc.h"

/*****************************
	
	PA0 -- TIM2_CH1 -- front --  3310
	PA1 -- TIM2_CH2 -- back  --  3320
	PA2 -- TIM2_CH3 -- left  --  3330
	PA3 -- TIM2_CH4 -- right --  3340
*************************/


void usart_handler(void)
{
	u8 t;
	u8 len;
	u16 pwmval=0;
	uint16_t tmp;
	uint8_t i;
	
	printf("here?\r\n");
	LED0 = ~LED0;
	len=USART_RX_STA&0x3fff;
	if(len < 6){
		printf("len = %d\r\n",len);		
		for(t=0;t<len;t++)
		{
			tmp = (uint8_t)USART_RX_BUF[t] - 48;		
			for(i = 0; i < len - t -1;i++){
				tmp *= 10;
			}
			pwmval += tmp;
		}
		printf("pwmval_ori = %d\r\n",pwmval);
		
		if((pwmval / 100) == 22){
			if((pwmval % 10) == 1){
				Rc_Get.flag = 1;
				Rc_Get.STATUS_OK = 1;
			}else{
				Rc_Get.flag = 0;
				Rc_Get.STATUS_OK = 0;
				Rc_Get.THROTTLE = 1000;
			}
			Rc_Get.dir = 0;
			LED1 = ~LED1;
		}
		
		if((pwmval / 100) == 33){
			//select the diretion
			if((pwmval % 10) == 0){
				//selfcontrol flag
				Rc_Get.dir = 0;
			}else {
				Rc_Get.dir = pwmval % 100 / 10;
			}
		}
		
		printf("Change dir : %d\r\n",Rc_Get.dir);
		if((pwmval / 100) < 20){
			//control the speed of motor
			printf("flag = %d\r\n",Rc_Get.dir);
			if (pwmval > 0){
				Rc_Get.THROTTLE = pwmval;
			}
		}
	}else{
		printf("len more than 10 ,error \r\n");
	}
	USART_RX_STA=0;
}




