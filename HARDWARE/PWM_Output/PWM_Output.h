#ifndef __PWM_Output_H
#define __PWM_Output_H

#include "stm32f4xx.h"
#include "LED.h"


//---------------PWM��������--------------------------
#define  PWM_1_2_3_4_Period 	 		2500	  		//PWM���ͨ��1.2.3.4�����ڣ���λ us  TIM5
#define  PWM_5_6_Period          	3000	  		//PWM���ͨ��5.6�����ڣ���λ us	   TIM3
#define  PWM_Hz                 (1000000/PWM_1_2_3_4_Period)  //400HZ

//----------------------------------------------------
#define  Servo_Period            3000	  	//PWM��������ڣ���λ us
#define  Servo_Neutral_position   1500	  //����ֵ

//����ͨ����Ĭ��ֵ ��λ us
#define  PWMOuput_CH1_Default   1000
#define  PWMOuput_CH2_Default   1000
#define  PWMOuput_CH3_Default   1000
#define  PWMOuput_CH4_Default   1000
#define  PWMOuput_CH5_Default   1000
#define  PWMOuput_CH6_Default   1000




//-------------PWM ��� API ------------------------------
// x �ĵ�λ��us.
#define  Set_PWMOuput_CH1(x)  TIM2->CCR1 = x ;
#define  Set_PWMOuput_CH2(x)  TIM2->CCR2 = x ;
#define  Set_PWMOuput_CH3(x)  TIM2->CCR3 = x ;
#define  Set_PWMOuput_CH4(x)  TIM2->CCR4 = x ;

//------------��PWM ��� API ��-----------------------------
extern volatile uint8_t THROTTLE_LOCKed;
extern volatile uint8_t  Fly_Mode;

void PWM_Output_Initial(void);	//��ʼ�� 
void PWM_Output_Set_default(void);	// ���Ĭ��ֵ
void PWM_Write_Motors(void);



#endif

//------------------End of File----------------------------
