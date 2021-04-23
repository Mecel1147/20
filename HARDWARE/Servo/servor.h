#ifndef __SERVOR_H
#define	__SERVOR_H

//#include "stm32f10x.h"
#include "sys.h"
		void Servor_angle(u8 t);	
		void servor_init(void);
		void whirl_grab(void);
		void whirl_tank(void);
		void claw_whirl(void);
		void reteat_arm(void);
		void extend_arm(void);
		void extend_claw(void);
		void grab(void);

#endif /* __GPIO_H */
