#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

void ECHO_Init(void);
void TIM8_CC_IRQHandler(void);
void TRIG_Init(void);          //���������Ϳڳ�ʼ��
short forward_distance(void); //ǰ����������
short back_distance(void);    //����������
short left_distance(void);    //����������
short right_distance(void);   //�ҳ���������

#endif























