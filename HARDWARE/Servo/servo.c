#include "servo.h"
#include "usart.h"
#include "delay.h"
#include "oled.h"
#include "motor.h"
#include "attitude.h"
#include "color.h"
#include "function.h"
extern int ic;
//复位
void DJ_Init()
{
  printf("#1P500#2P1147#3P2000#4P1412#5P1471#6P912T2000\r\n");
	delay_ms(2000);
	printf("#1P1803#2P1450#3P1427#4P950#5P1424#6P971T1000\r\n");
	delay_ms(1000);
}
//抓取伸出等待
void grab_Init()
{
//	printf("#1P500#2P1147#3P2000#4P1412#5P1471#6P912T2000\r\n");
//	delay_ms(2000);
	printf("#1P1803#2P1450#3P1427#4P950#5P1424#6P971T1000\r\n");
	delay_ms(1000);
	//printf("#1P1794#2P1176#3P1676#4P1235#5P2029#6P912T500\r\n");
	//delay_ms(500);
}

//抓取捕捉(返回初始位置) 
void grab()
{
	
//	printf("#1P1794#2P1176#3P1676#4P824#5P1382#6P1000T2000\r\n");
//	delay_ms(1500);
	printf("#1P1803#2P1450#3P1427#4P950#5P1424#6P971T950\r\n");
	delay_ms(1000);
	printf("#1P1803#2P1450#3P1545#4P950#5P1424#6P971T850\r\n");
	delay_ms(900);
	printf("#1P1803#2P1450#3P1500#4P950#5P1424#6P1403T800\r\n");
	delay_ms(850);
	printf("#1P1803#2P1450#3P1324#4P1441#5P1424#6P1403T800\r\n");
	delay_ms(850);
	printf("#1P1803#2P647#3P1971#4P618#5P1424#6P1403T800\r\n");
	delay_ms(850);
	printf("#1P1324#2P765#3P2059#4P765#5P1424#6P1403T650\r\n");
	delay_ms(700);
	printf("#1P1324#2P765#3P2059#4P765#6P1029T650\r\n");
	delay_ms(700);
}

// 二维码——放机械臂
void QR_code_play()
{
  printf("#1P500#2P912#3P1971#4P950#5P1424#6P971T800\r\n");
	delay_ms(900);
	printf("#1P1265#2P912#3P1971#4P950#5P1424#6P971T800\r\n");
	delay_ms(900);
	printf("#1P1147#2P618#3P588#4P912#5P1424#6P971T800\r\n");
	delay_ms(900);
	printf("#1P500#2P618#3P588#4P1088#5P1424#6P971T800\r\n");
	delay_ms(900);
}


//二维码回收机械臂
void QR_code_recycle()
{
  printf("#1P1147#2P618#3P588#4P912#5P1424#6P971T800\r\n");
	delay_ms(900);
	printf("#1P1265#2P912#3P1971#4P950#5P1424#6P971T800\r\n");
	delay_ms(900);
	printf("#1P500#2P912#3P1971#4P950#5P1424#6P971T800\r\n");
	delay_ms(900);
	
}

////拨球(抬起拨片) 
//void BQT()
//{
//	printf("#1P1763#2P1605#3P1132#4P632#5P1395#6P1684T500");
//	delay_ms(500);
//}



//放入仓库(返回初始位置) 
void store_house()
{
	 printf("#1P824#2P1294#3P1588#4P941#5P1735#6P1382T800\r\n");
		delay_ms(900);	
}

void first_floor(void)
{
			printf("#1P794#2P1647#3P1882#4P1412#5P1471#6P912T800\r\n");
			delay_ms(900);
			printf("#1P824#2P1676#3P1529#4P500#5P1765#6P912T800\r\n");
			delay_ms(900);
			printf("#1P824#2P1853#3P1412#4P500#5P1765#6P912T800\r\n");
			delay_ms(900);
			printf("#1P824#2P1853#3P1412#4P500#5P1735#6P1382T800\r\n");
			delay_ms(900);
			printf("#1P824#2P1559#3P1588#4P500#5P1735#6P1382T800\r\n");
			delay_ms(900);	
		  printf("#1P824#2P1294#3P1588#4P941#5P1735#6P1382T800\r\n");
			delay_ms(900);
				
//			printf("#1P500#2P1441#3P1882#4P1412#5P1471#6P2500T800\r\n");
//			delay_ms(900);
		
			printf("#1P1824#2P941#3P2029#4P882#5P1471#6P1353T550\r\n");
			delay_ms(600);
			printf("#2P941#3P2029#4P882#6P1176T900\r\n");
			delay_ms(1000);	
}
void second_floor()
{
	//			printf("#1P500#2P1588#3P1882#4P1412#5P1471#6P912T1000\r\n");
//			delay_ms(1000);
			printf("#1P794#2P1647#3P1882#4P1412#5P1471#6P912T1000\r\n");
			delay_ms(1050);
			printf("#1P824#2P1676#3P1529#4P500#5P1765#6P912T1000\r\n");
			delay_ms(1050);
			printf("#1P824#2P1853#3P1412#4P500#5P1765#6P912T1000\r\n");
			delay_ms(1050);
			printf("#1P824#2P1853#3P1412#4P500#5P1735#6P1382T1000\r\n");
			delay_ms(1050);
			printf("#1P824#2P1559#3P1588#4P500#5P1735#6P1382T1000\r\n");
			delay_ms(1050);
			printf("#1P824#2P1294#3P1588#4P941#5P1735#6P1382T1000\r\n");
			delay_ms(1050);	
		
			printf("#1P500#2P1441#3P1882#4P1412#5P1471#6P2500T500\r\n");
			delay_ms(550);
			printf("#1P1824#2P1382#3P1588#4P1000#5P1412#6P1382T5000\r\n");
			delay_ms(5050);
			printf("#2P1382#3P1588#4P1000#5P1412#6P1000T500\r\n");
			delay_ms(550);

}
void third_floor(void)
{
			
//			printf("#1P794#2P1647#3P1882#4P1412#5P1471#6P912T1000\r\n");
//			delay_ms(1050);
//			printf("#1P824#2P1676#3P1529#4P500#5P1765#6P912T1500\r\n");
//			delay_ms(1550);
//			printf("#1P824#2P1853#3P1412#4P500#5P1765#6P912T1000\r\n");
//			delay_ms(1050);
//			printf("#1P824#2P1853#3P1412#4P500#5P1735#6P1382T1000\r\n");
//			delay_ms(1050);
//			printf("#1P824#2P1559#3P1588#4P500#5P1735#6P1382T1000\r\n");
//			delay_ms(1050);
//			printf("#1P824#2P1294#3P1588#4P941#5P1735#6P1382T1000\r\n");
//			delay_ms(1050);	
	
	printf("#1P794#2P1647#3P1882#4P1412#5P1471#6P912T800\r\n");
			delay_ms(900);
			printf("#1P824#2P1676#3P1529#4P500#5P1765#6P912T800\r\n");
			delay_ms(900);
			printf("#1P824#2P1853#3P1412#4P500#5P1765#6P912T800\r\n");
			delay_ms(900);
			printf("#1P824#2P1853#3P1412#4P500#5P1735#6P1382T800\r\n");
			delay_ms(900);
			printf("#1P824#2P1559#3P1588#4P500#5P1735#6P1382T800\r\n");
			delay_ms(900);	
		  printf("#1P824#2P1294#3P1588#4P941#5P1735#6P1382T800\r\n");
			delay_ms(900);
		
			printf("#1P1824#2P1471#3P1235#4P882#5P1353#6P91382T800\r\n");
			delay_ms(2050);
			printf("#1P1824#2P1882#3P912#4P824#5P1412#6P1000T800\r\n");
			delay_ms(850);
			printf("#1P1824#2P1882#3P500#4P824#5P1412#6P1000T800\r\n");
			delay_ms(850);
			printf("#1P794#2P1882#3P500#4P824#5P1412#6P1000T800\r\n");
			delay_ms(850);
		
}
void floors_house(int a)
{
	//disable();
	switch(a){
		case 1:
						first_floor();
						break;

		case 2:
						second_floor();
						break;
			
		case 3:
						third_floor();
						break;
		}
		store_house();
	}

void End_recycle(){

 printf("#1P500#2P1147#3P2000#4P1412#5P1471#6P912T2000\r\n");
 delay_ms(2000);
}

