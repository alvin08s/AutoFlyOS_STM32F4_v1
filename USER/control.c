
/******************
Uo(n) = P *e(n) + I *[e(n)+e(n-1)+...+e(0)]+ D *[e(n)-e(n-1)]
***********************/

#include <math.h>
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "main.h"
#include "control.h"
#include "PWM_Output.h"
#include "rc.h"

#define Moto_PwmMax 1999
#define Moto_PwmMin	1000
#define BALANCE_STATUS						0.0f
#define LR_DIR_DEGRESS_MIN				6.0f
#define LR_DIR_DEGRESS_MAX				4.0f
#define COUNT_NUM 								10
#define GET_DATA_TIME							168  //read the mpu6050 data per millisecond 

PID PID_ROL,PID_PIT,PID_YAW;
volatile uint16_t moto1,moto2,moto3,moto4;
float pitch, roll, yaw; 							//Real Time value欧拉角
static float	tempgx = 0,tempgy = 0,tempgz = 0;
static uint8_t cnt_g = 0;
static uint8_t count_time = 0;
static uint8_t graves_val; //char grave_aaa[20];


void fly_control(void)
{
	LED0 = ~LED0;
	count_time ++;
	if(count_time == GET_DATA_TIME){
//		printf("read the data ...\r\n");
		MPU6050_Dataanl();			//update data 
		
		PID_ROL.pout = PID_ROL.P * (roll - BALANCE_STATUS);
		PID_ROL.iout = PID_ROL.I * (MPU6050_GYRO_LAST.ROLL - BALANCE_STATUS*COUNT_NUM);
		PID_ROL.dout = PID_ROL.D * (roll - MPU6050_GYRO_LAST.ROLL_SECOND);
		
		PID_PIT.pout = PID_PIT.P * (pitch - BALANCE_STATUS);
		PID_PIT.iout = PID_PIT.I * (MPU6050_GYRO_LAST.PITCH - BALANCE_STATUS*COUNT_NUM);
		PID_PIT.dout = PID_PIT.D * (pitch - MPU6050_GYRO_LAST.PITCH_SECOND);
		
		PID_YAW.pout = PID_YAW.P * (roll - BALANCE_STATUS);
		PID_YAW.iout = PID_YAW.I * (MPU6050_GYRO_LAST.YAW - BALANCE_STATUS*COUNT_NUM);
		PID_YAW.dout = PID_YAW.D * (yaw - MPU6050_GYRO_LAST.YAW_SECOND);
		
		PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;
		PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
		PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;
		
		if(Rc_Get.THROTTLE>1200)
		{
			/***
			moto1 = Rc_Get.THROTTLE - PID_ROL.OUT;
			moto2 = Rc_Get.THROTTLE + PID_PIT.OUT;
			moto3 = Rc_Get.THROTTLE + PID_ROL.OUT;
			moto4 = Rc_Get.THROTTLE - PID_PIT.OUT;
			***/
			
			moto1 = Rc_Get.THROTTLE - PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT;
			moto2 = Rc_Get.THROTTLE - PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT;
			moto3 = Rc_Get.THROTTLE + PID_ROL.OUT +	PID_PIT.OUT + PID_YAW.OUT;
			moto4 = Rc_Get.THROTTLE + PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT;
			
			printf("moto1 = %d ,moto2 = %d,moto3 = %d,moto4 = %d \r\n",moto1,moto2,moto3,moto4);
			printf("roll = %f ,pitch = %f ,yaw = %f \r\n",roll,pitch,yaw);

		}
		else
		{
			moto1 = Rc_Get.THROTTLE;
			moto2 = Rc_Get.THROTTLE;
			moto3 = Rc_Get.THROTTLE;
			moto4 = Rc_Get.THROTTLE;
		}
		if(1 == Rc_Get.STATUS_OK){
			Moto_PwmRflash(moto1,moto2,moto3,moto4);
		}else {
			Moto_PwmRflash(1000,1000,1000,1000);
		}
		
		MPU6050_GYRO_LAST.ROLL = 0;
		MPU6050_GYRO_LAST.PITCH = 0;
		MPU6050_GYRO_LAST.YAW = 0;
		
		count_time = 0;
	}
}


void MPU6050_Dataanl(void)				//记得要更新mpu6050_buffer的信息
{
	
	MPU6050_GYRO_LAST.ROLL_SECOND = roll;
	MPU6050_GYRO_LAST.PITCH_SECOND = pitch;
	MPU6050_GYRO_LAST.YAW_SECOND = yaw;

	graves_val = mpu_dmp_get_data(&pitch, &roll, &yaw); 		
	//获得的为转换之后的结果
	//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
	//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
	//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
	if(graves_val == 0){
		tempgx+= roll;
		tempgy+= pitch;
		tempgz+= yaw;
		
		if(cnt_g == COUNT_NUM){
			/***
			MPU6050_GYRO_LAST.ROLL = tempgx/cnt_g ;
			MPU6050_GYRO_LAST.PITCH = tempgy/cnt_g;
			MPU6050_GYRO_LAST.YAW = tempgz/cnt_g;
			***/
			MPU6050_GYRO_LAST.ROLL = tempgx;
			MPU6050_GYRO_LAST.PITCH = tempgy;
			MPU6050_GYRO_LAST.YAW = tempgz;
			
			printf("tempgx = %f ,tempgy = %f ,tempgz = %f \r\n",tempgx,tempgy,tempgz);
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_g = 0;
			return;
		}
		cnt_g++;
	}
}

void Pid_init(void)
{
	PID_ROL.P = 6;
	PID_ROL.I = 0.3;
	PID_ROL.D = 1.0;
	
	PID_PIT.P = 6;
	PID_PIT.I = 0.3;
	PID_PIT.D = 1.0;
	
	PID_YAW.P = 0;
	PID_YAW.I = 0;
	PID_YAW.D = 0;
	
	PID_ROL.pout = 0;
	PID_ROL.iout = 0;
	PID_ROL.dout = 0;
	
	PID_PIT.pout = 0;
	PID_PIT.iout = 0;
	PID_PIT.dout = 0;
	
	PID_YAW.pout = 0;
	PID_YAW.iout = 0;
	PID_YAW.dout = 0;
	
	PID_ROL.OUT = 0;
	PID_PIT.OUT = 0;
	PID_YAW.OUT = 0;
}

void EK_XYZ_init(void)
{
	MPU6050_GYRO_LAST.ROLL = 0;
	MPU6050_GYRO_LAST.PITCH=0;
	MPU6050_GYRO_LAST.YAW = 0;
	MPU6050_GYRO_LAST.PITCH_SECOND = 0;
	MPU6050_GYRO_LAST.ROLL_SECOND = 0;
	MPU6050_GYRO_LAST.YAW_SECOND = 0;
	MPU6050_GYRO_LAST.EX_I = 0;
}

void Moto_PwmRflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
{
	
	if(MOTO1_PWM > Moto_PwmMax)
		MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM > Moto_PwmMax)
		MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM > Moto_PwmMax)	
		MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM > Moto_PwmMax)	
		MOTO4_PWM = Moto_PwmMax;
	
	if(MOTO1_PWM< Moto_PwmMin)
		MOTO1_PWM = Moto_PwmMin;
	if(MOTO2_PWM<Moto_PwmMin)
		MOTO2_PWM = Moto_PwmMin;
	if(MOTO3_PWM<Moto_PwmMin)
		MOTO3_PWM = Moto_PwmMin;
	if(MOTO4_PWM<Moto_PwmMin)	
		MOTO4_PWM = Moto_PwmMin;

	TIM2->CCR1 = MOTO1_PWM;
	TIM2->CCR2 = MOTO2_PWM;
	TIM2->CCR3 = MOTO3_PWM;
	TIM2->CCR4 = MOTO4_PWM;

}



