#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "main.h"
#include "PWM_Output.h"
#include "rc.h"
#include "control.h"


EK_XYZ	MPU6050_GYRO_LAST;		//最新一次读取值
RC_GETDATA Rc_Get;


int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  		//初始化延时函数
	uart_init(115200);		//初始化串口波特率为500000
	LED_Init();						//初始化LED 

	MPU_Init();					//初始化MPU6050
	PWM_Output_Initial();
	while(mpu_dmp_init());  //初始化mpu6050  同时矫正首次使用的值
	printf("init complete... \r\n");
	
	Rc_Get.flag = 0;
	Rc_Get.THROTTLE = 1000;

	Pid_init();
	EK_XYZ_init();

 	while(1)
	{
		if(USART_RX_STA&0x8000){
			usart_handler();
			LED1 = ~LED1;
		}
		if(Rc_Get.flag)
		{
			fly_control();
		}
	}
}



