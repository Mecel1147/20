#include "servor.h"
#include "usart.h"
#include "delay.h"
#define MAXPWM 2505		 //舵机最大PWM控制脉宽2.5ms宏定义
void Servor_angle(u8 t)	
{
	switch(t)
	{
		case 0:		printf("#1P500T100\r\n");break;
		case 1:		printf("#1P508T100\r\n");break;
		case 2:		printf("#1P517T100\r\n");break;
		case 3:		printf("#1P525T100\r\n");break;
		case 4:		printf("#1P533T100\r\n");break;
		case 5:		printf("#1P542T100\r\n");break;
		case 6:		printf("#1P550T100\r\n");break;
		case 7:		printf("#1P558T100\r\n");break;
		case 8:		printf("#1P567T100\r\n");break;
		case 9:		printf("#1P575T100\r\n");break;
		case 10:		printf("#1P583T100\r\n");break;
		case 11:		printf("#1P592T100\r\n");break;
		case 12:		printf("#1P600T100\r\n");break;
		case 13:		printf("#1P608T100\r\n");break;
		case 14:		printf("#1P617T100\r\n");break;
		case 15:		printf("#1P625T100\r\n");break;
		case 16:		printf("#1P633T100\r\n");break;
		case 17:		printf("#1P642T100\r\n");break;
		case 18:		printf("#1P650T100\r\n");break;
		case 19:		printf("#1P658T100\r\n");break;
		case 20:		printf("#1P667T100\r\n");break;
		case 21:		printf("#1P675T100\r\n");break;
		case 22:		printf("#1P683T100\r\n");break;
		case 23:		printf("#1P692T100\r\n");break;
		case 24:		printf("#1P700T100\r\n");break;
		case 25:		printf("#1P708T100\r\n");break;
		case 26:		printf("#1P717T100\r\n");break;
		case 27:		printf("#1P725T100\r\n");break;
		case 28:		printf("#1P733T100\r\n");break;
		case 29:		printf("#1P742T100\r\n");break;
		case 30:		printf("#1P750T100\r\n");break;
		case 31:		printf("#1P758T100\r\n");break;
//		case 32:		printf("#1P767T100\r\n");break;
//		case 33:		printf("#1P775T100\r\n");break;
//		case 34:		printf("#1P783T100\r\n");break;
//		case 35:		printf("#1P792T100\r\n");break;
//		case 36:		printf("#1P800T100\r\n");break;
//		case 37:		printf("#1P808T100\r\n");break;
//		case 38:		printf("#1P817T100\r\n");break;
//		case 39:		printf("#1P825T100\r\n");break;
//		case 40:		printf("#1P833T100\r\n");break;
//		case 41:		printf("#1P842T100\r\n");break;
//		case 42:		printf("#1P850T100\r\n");break;
//		case 43:		printf("#1P858T100\r\n");break;
//		case 44:		printf("#1P867T100\r\n");break;
//		case 45:		printf("#1P875T100\r\n");break;
//		case 46:		printf("#1P883T100\r\n");break;
//		case 47:		printf("#1P892T100\r\n");break;
//		case 48:		printf("#1P900T100\r\n");break;
//		case 49:		printf("#1P908T100\r\n");break;
//		case 50:		printf("#1P917T100\r\n");break;
//		case 51:		printf("#1P925T100\r\n");break;
//		case 52:		printf("#1P933T100\r\n");break;
//		case 53:		printf("#1P942T100\r\n");break;
//		case 54:		printf("#1P950T100\r\n");break;
//		case 55:		printf("#1P958T100\r\n");break;
//		case 56:		printf("#1P967T100\r\n");break;
//		case 57:		printf("#1P975T100\r\n");break;
//		case 58:		printf("#1P983T100\r\n");break;
//		case 59:		printf("#1P992T100\r\n");break;
//		case 60:		printf("#1P1000T100\r\n");break;
		default :break;
	}
}

void grab(void)
	{
//		printf("#4P1334T200\r\n");
//		delay_ms(200);
//		printf("#4P1340T200\r\n");
//		delay_ms(200);
//		printf("#4P1343T200\r\n");
//		delay_ms(200);
//		printf("#4P1346T200\r\n");
//		delay_ms(200);
//		printf("#4P1350T200\r\n");
//		delay_ms(200);
//		printf("#4P1353T200\r\n");
//		delay_ms(200);
//		printf("#4P1356T200\r\n");
//		delay_ms(200);
//		printf("#4P1360T200\r\n");
//		delay_ms(200);
//		printf("#4P1363T200\r\n");
//		delay_ms(200);
		printf("#4P1370T200\r\n");
		delay_ms(200);
	}
	//张开夹爪
	void extend_claw(void)
	{
		printf("#4P1000T300\r\n");
		delay_ms(500);
	}
	//伸出机械臂
	void extend_arm(void)
	{
		printf("#1P600T300\r\n");
		delay_ms(500);
	}
	//收回机械爪 
	void reteat_arm(void)
	{
		printf("#1P1800T400\r\n");
		delay_ms(500);
	}
	void claw_whirl(void)
	{
		printf("#5P1536T400\r\n");
		delay_ms(500);
	}
	void whirl_tank()
	{
		printf("#3P2000T1000\r\n");
		delay_ms(1000);;
	}
	void whirl_grab(void)
	{
		printf("#3P500T1000\r\n");
		delay_ms(1000);;
	}
	void servor_init(void)
	{
		printf("#1P1800#3P500#4P1000#5P1536T1000\r\n");
		delay_ms(1100);
	}
