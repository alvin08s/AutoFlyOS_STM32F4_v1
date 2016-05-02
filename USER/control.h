#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f4xx.h"
#include "rc.h"


typedef struct PID{
	float P;
	float I;
	float D;
	float pout;
	float iout;
	float dout;
	float IMAX;
	float OUT;
}PID;

extern PID PID_ROL,PID_PIT,PID_YAW;

void CONTROL(float rol, float pit, float yaw);

void Pid_init(void);
void Moto_PwmRflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);

void upanddown(void);
void turnleft(void);
void turnright(void);



typedef struct{
	float ROLL;
	float PITCH;
	float YAW;
	float ROLL_SECOND;
	float PITCH_SECOND;
	float YAW_SECOND;
	float EX_I;
}EK_XYZ;

extern EK_XYZ MPU6050_GYRO_LAST;		//最新一次读取值
extern EK_XYZ	GYRO_OFFSET,ACC_OFFSET;								//零漂

void EK_XYZ_init(void);
void MPU6050_Dataanl(void);					//记得要更新mpu6050_buffer的信息
void Pid_init(void);
void EK_XYZ_init(void);

#endif
