#ifndef __MAIN_H
#define __MAIN_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

/******************自定义处理函数*******************/

void usart1_send_char(u8 c);
void usart1_niming_report(u8 fun,u8*data,u8 len);
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);

void usart_handler(void);
void fly_control(void);



#endif


