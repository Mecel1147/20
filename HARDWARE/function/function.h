#ifndef __FUNCTION_H
#define __FUNCTION_H 
#include "sys.h"   
#define  PD0  PDin(0)

void TIM12_Init(u16 arr,u16 psc);
void straight_y(int speed);
void straight_x(int speed);
void GPIOA_Init(void);
#define PA11 PAout(11)
#define PA12 PAout(12)
void go_line(int choose,int num,int speed);
//X����ѭ�������ж�
void derictionx1(void);
//X����ѭ�������ж�
void derictionx2(void);
//Y����ѭ�������ж�
void derictiony1(void);
//Y����ѭ�������ж�
void derictiony2(void);
//OLED��ʾ����
void  oled_show(void);
//ֹͣ����
void secure(void);
void countx1(void);
void county1(void);
//������ʱ����
void one_delay(int time);
//��������ѭ��
void move_blue(void);
void maintain_distance(int distance,int times,int speed);
//ѭ��ץȡ
void move_grab_blue(int speed);
void move_grab_red(void);
void store(int speed, int low_speed);
void switch_storage(int target,int floor, int speed);
void switch_storage1(int target, int speed);
void count_storage(void);
void TIM14_Init(u16 arr,u16 psc);//��ʱ��14��ʼ��
int IC_OLED(void);
void disable(void);
void enable(void);
void countx2(void);
void county2(void);
#endif
