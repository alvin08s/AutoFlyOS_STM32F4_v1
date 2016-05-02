/* PWM_Output.c file
ռ��STM32 ��Դ��
1. ʹ��Tim2 Tim4 ��ʱ�� ����PWM�ź�
2. ��������
   PA0 -- TIM2_CH1
	 PA1 -- TIM2_CH2
	 PA2 -- TIM2_CH3
	 PA3 -- TIM2_CH4
	 /TIM2 Control motor,used to control speed/
	 
   PB8 -- TIM4_CH3
	 PB9 -- TIM4_CH4
	
	�������� ���ڲ���PWM�ź�
	���ܣ�
	��ʼ����ʱ�� ����PWM�ź�
------------------------------------
 */

#include "PWM_Output.h"
#include "usart.h"


/**************************ʵ�ֺ���********************************************
*����ԭ��:		static void PWMOutput_GPIO_Config(void) 
*��������:	    ����PWM����Ϊ�������������Ӧ��ʱ���ź� �����Ÿ���ʱ���ź�  	 
*******************************************************************************/
static void PWMOutput_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE); 

  /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;		   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // �����������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM4);
		
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		static void Tim3_NVIC_Config(void)
*��������:	    ���ö�ʱ��3 ���ж����ȼ��� 	 
*******************************************************************************/
static void Tim2_NVIC_Config(void) {
 NVIC_InitTypeDef NVIC_InitStructure; 
 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   //ѡ��TIM2�ж�
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;   //
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ��
 NVIC_Init(&NVIC_InitStructure); 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		static void PWMOutput_TIMERs_Config(void)
*��������:	    ����PWM�Ķ�Ӧ��ʱ�� 
*******************************************************************************/
static void PWMOutput_TIMERs_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE); 
	
  Tim2_NVIC_Config();
	/* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = Servo_Period;      //PWM����
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1;	    			//����Ԥ��Ƶ����Ϊ1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;				//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ

  TIM_TimeBaseStructure.TIM_Period = PWM_5_6_Period;      //PWM����
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = PWM_1_2_3_4_Period;  //PWM����
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

//  TIM2->DIER |= 0x0001;  //ʹ���ж�	 ʹ�ܶ�ʱ��2�� ����ж�

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = Servo_Neutral_position;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��1
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //ʹ��ͨ��2
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��3
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��3
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);	//ʹ��ͨ��4
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//ʹ��ͨ��4
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);			 // ʹ��TIM4���ؼĴ���ARR
  TIM_ARRPreloadConfig(TIM2, ENABLE);			 // ʹ��TIM5���ؼĴ���ARR

  TIM_Cmd(TIM4, ENABLE);                   //ʹ�ܶ�ʱ��4
  TIM_Cmd(TIM2, ENABLE);                   //ʹ�ܶ�ʱ��5
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void PWM_Output_Initial(void)
*��������:	    PWM��� ��ʼ����
*******************************************************************************/
void PWM_Output_Initial(void)
{
	PWMOutput_GPIO_Config();
	PWMOutput_TIMERs_Config();
	Set_PWMOuput_CH1(PWMOuput_CH1_Default);	 // ��Ĭ��ֵ���
	Set_PWMOuput_CH2(PWMOuput_CH2_Default);
	Set_PWMOuput_CH3(PWMOuput_CH3_Default);
	Set_PWMOuput_CH4(PWMOuput_CH4_Default);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void PWM_Output_Set_default(void)
*��������:	    PWM��� Ĭ��ֵ
*******************************************************************************/
void PWM_Output_Set_default(void) {

	Set_PWMOuput_CH1(PWMOuput_CH1_Default);
	Set_PWMOuput_CH2(PWMOuput_CH2_Default);
	Set_PWMOuput_CH3(PWMOuput_CH3_Default);
	Set_PWMOuput_CH4(PWMOuput_CH4_Default);

}

//------------------End of File----------------------------
