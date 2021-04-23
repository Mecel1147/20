#ifndef __MOTOR_H
#define __MOTOR_H 
#include "sys.h"   

//电机方向控制IO宏定义
#define Motor_A_2   PEout(0)
#define Motor_A_1   PEout(1)
#define Motor_B_2   PEout(2)
#define Motor_B_1   PEout(3)
#define Motor_C_1   PEout(4)
#define Motor_C_2   PEout(5)
#define Motor_D_1   PEout(6)
#define Motor_D_2   PCout(13)

extern int Target_Speed[];
#define Target_A Target_Speed[0]
#define Target_B Target_Speed[1]
#define Target_C Target_Speed[2]
#define Target_D Target_Speed[3]


//电机速度PI的时间基准
void TIM7_Init(u16 arr,u16 psc);

//车身运动控制算法 
void Kinematic_Analysis(float Vx,float Vy,float Vz);

//电机速度控制算法
int Motor_A_PI(int Encoder, int Target);
int Motor_B_PI(int Encoder, int Target);
int Motor_C_PI(int Encoder, int Target);
int Motor_D_PI(int Encoder, int Target);
void Motor_Target_Limit(short int *A,short int *B,short int *C,short int *D);
int Motor_Pwm_Limit(int pwm);
void Caculate_Encoder(int temp[]);

//电机硬件编码器初始化函数
void TIM2_ENC_Init(void);
void TIM3_ENC_Init(void);
void TIM4_ENC_Init(void);
void TIM5_ENC_Init(void);

//电机的PWM信号初始化函数
void TIM1_Pwm_Init(u16 arr,u16 psc); 

//电机方向控制IO初始化
void Motor_IO_Init(void);       

//电机硬件底层初始化
void Motor_Init(void);
//电机转向设置
void Motor_start(void);
//电机停转
void Motor_stop(void);
//小车转向180
void Motor_whirl(int angle);

#endif
