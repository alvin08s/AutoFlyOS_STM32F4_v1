/* PWM_Output.c file
占用STM32 资源：
1. 使用Tim2 Tim4 定时器 产生PWM信号
2. 以下引脚
   PA0 -- TIM2_CH1
	 PA1 -- TIM2_CH2
	 PA2 -- TIM2_CH3
	 PA3 -- TIM2_CH4
	 /TIM2 Control motor,used to control speed/
	 
   PB8 -- TIM4_CH3
	 PB9 -- TIM4_CH4
	
	以上引脚 用于产生PWM信号
	功能：
	初始化定时器 产生PWM信号
------------------------------------
 */

#include "PWM_Output.h"
#include "usart.h"


/**************************实现函数********************************************
*函数原型:		static void PWMOutput_GPIO_Config(void) 
*功　　能:	    配置PWM引脚为输出，并开启相应的时钟信号 和引脚复用时钟信号  	 
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
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // 复用推挽输出
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

/**************************实现函数********************************************
*函数原型:		static void Tim3_NVIC_Config(void)
*功　　能:	    配置定时器3 的中断优先级别 	 
*******************************************************************************/
static void Tim2_NVIC_Config(void) {
 NVIC_InitTypeDef NVIC_InitStructure; 
 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   //选择TIM2中断
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;   //
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能
 NVIC_Init(&NVIC_InitStructure); 
}

/**************************实现函数********************************************
*函数原型:		static void PWMOutput_TIMERs_Config(void)
*功　　能:	    配置PWM的对应定时器 
*******************************************************************************/
static void PWMOutput_TIMERs_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE); 
	
  Tim2_NVIC_Config();
	/* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = Servo_Period;      //PWM周期
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1;	    			//设置预分频：即为1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;				//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//向上计数模式

  TIM_TimeBaseStructure.TIM_Period = PWM_5_6_Period;      //PWM周期
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = PWM_1_2_3_4_Period;  //PWM周期
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

//  TIM2->DIER |= 0x0001;  //使能中断	 使能定时器2的 溢出中断

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = Servo_Neutral_position;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //使能通道1
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //使能通道2
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //使能通道3
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //使能通道3
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);	//使能通道4
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//使能通道4
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);			 // 使能TIM4重载寄存器ARR
  TIM_ARRPreloadConfig(TIM2, ENABLE);			 // 使能TIM5重载寄存器ARR

  TIM_Cmd(TIM4, ENABLE);                   //使能定时器4
  TIM_Cmd(TIM2, ENABLE);                   //使能定时器5
}

/**************************实现函数********************************************
*函数原型:		void PWM_Output_Initial(void)
*功　　能:	    PWM输出 初始化。
*******************************************************************************/
void PWM_Output_Initial(void)
{
	PWMOutput_GPIO_Config();
	PWMOutput_TIMERs_Config();
	Set_PWMOuput_CH1(PWMOuput_CH1_Default);	 // 放默认值输出
	Set_PWMOuput_CH2(PWMOuput_CH2_Default);
	Set_PWMOuput_CH3(PWMOuput_CH3_Default);
	Set_PWMOuput_CH4(PWMOuput_CH4_Default);
}

/**************************实现函数********************************************
*函数原型:		void PWM_Output_Set_default(void)
*功　　能:	    PWM输出 默认值
*******************************************************************************/
void PWM_Output_Set_default(void) {

	Set_PWMOuput_CH1(PWMOuput_CH1_Default);
	Set_PWMOuput_CH2(PWMOuput_CH2_Default);
	Set_PWMOuput_CH3(PWMOuput_CH3_Default);
	Set_PWMOuput_CH4(PWMOuput_CH4_Default);

}

//------------------End of File----------------------------
