#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

void ECHO_Init(void);
void TIM8_CC_IRQHandler(void);
void TRIG_Init(void);          //³¬Éù²¨·¢ËÍ¿Ú³õÊ¼»¯
short forward_distance(void); //Ç°³¬Éù²¨¾àÀë
short back_distance(void);    //ºó³¬Éù²¨¾àÀë
short left_distance(void);    //×ó³¬Éù²¨¾àÀë
short right_distance(void);   //ÓÒ³¬Éù²¨¾àÀë

#endif























