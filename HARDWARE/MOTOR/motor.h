#ifndef __MOTOR_H
#define __MOTOR_H 
#include "sys.h"   

//����������IO�궨��
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


//����ٶ�PI��ʱ���׼
void TIM7_Init(u16 arr,u16 psc);

//�����˶������㷨 
void Kinematic_Analysis(float Vx,float Vy,float Vz);

//����ٶȿ����㷨
int Motor_A_PI(int Encoder, int Target);
int Motor_B_PI(int Encoder, int Target);
int Motor_C_PI(int Encoder, int Target);
int Motor_D_PI(int Encoder, int Target);
void Motor_Target_Limit(short int *A,short int *B,short int *C,short int *D);
int Motor_Pwm_Limit(int pwm);
void Caculate_Encoder(int temp[]);

//���Ӳ����������ʼ������
void TIM2_ENC_Init(void);
void TIM3_ENC_Init(void);
void TIM4_ENC_Init(void);
void TIM5_ENC_Init(void);

//�����PWM�źų�ʼ������
void TIM1_Pwm_Init(u16 arr,u16 psc); 

//����������IO��ʼ��
void Motor_IO_Init(void);       

//���Ӳ���ײ��ʼ��
void Motor_Init(void);
//���ת������
void Motor_start(void);
//���ͣת
void Motor_stop(void);
//С��ת��180
void Motor_whirl(int angle);

#endif
