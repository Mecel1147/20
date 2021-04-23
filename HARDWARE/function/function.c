#include "sys.h"
#include "function.h"
#include "oled.h"
#include "iic.h"
#include "motor.h"
#include "attitude.h"
#include "grayscale.h"
#include "delay.h"
#include "distance.h"
#include "timer.h"
#include "servo.h"
#include "usart.h"
	u8 data[15] = {0x0F,0xA2,0x52,0x09,0x01,0x01,0x60,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x69,0x03};	
	//int c[9]=0;
extern int last_ic;
extern int ic;
extern int distance;
extern int speed;
int storage_direction=0;
extern u16 color_number;
extern int storage_line;
extern int i;
extern int destination;
extern u8 TIM8CH1_CAPTURE_STA;
extern u8 TIM8CH2_CAPTURE_STA;
extern u8 TIM8CH3_CAPTURE_STA;
extern u8 TIM8CH4_CAPTURE_STA;
extern int head;
extern int ball_num;
int d=0;//超声波数据
extern int count_x;
extern int count_y;
int num1;
extern int Encoder_Date[];
extern int PWM[];
extern int Angle;
extern int First_Angle;
extern int speedx;
extern int speedy;
extern int flag;
extern int bias;
extern int Reference_Angle;
extern int delay_flag;
extern int timesy;
int speedy=0;
int speedx=0;
//Y方向运动
void straight_y(int speed)
{
	if(speed>=0)
	{
		derictiony1();//Y
		Kinematic_Analysis(speedx,speed,-Dir_PID(First_Angle,Angle));
	}
		else if(speed<0)
		{
			derictiony2();
			Kinematic_Analysis(speedx,speed,-Dir_PID(First_Angle,Angle));
		}
		//one_delay(500);
		county1();
		//countx2();
	
}
//Y方向运动
void straight_y2(int speed)
{
	if(speed>=0)
	{
		derictiony1();//Y
		Kinematic_Analysis(speedx,speed,-Dir_PID(First_Angle,Angle));
	}
		else if(speed<0)
		{
			derictiony2();
			Kinematic_Analysis(speedx,speed,-Dir_PID(First_Angle,Angle));
		}
		//one_delay(500);
		//county1();
		countx2();
	
}
//X方向运动
void straight_x(int speed)
{
	
	if(speed>=0)
	{
		derictionx1();
		Kinematic_Analysis(speed,speedy,-Dir_PID(First_Angle,Angle));
		
	}
		else if(speed<0)
	{
		derictionx1();
		Kinematic_Analysis(speed,speedy,-Dir_PID(First_Angle,Angle));
		
	}
	countx1();
	}

void go_line(int choose,int num,int speed)
{
	int j,i;
	switch(choose)
	{
		case 1: 
		{
				for(j=0;j<num;j++)//走num个格
			{
				i=0;
			do
			{
				
				straight_x(speed);
				if(i==0)
				{
					delay_ms(300);
					i++;
				}
				}while(grayscale1()&&grayscale3());
		}
		};break;
		case 2: 
		{
				for(j=0;j<num;j++)//走num个格
			{
				i=0;
			do
			{
				
				straight_y(speed);
				if(i==0)
				{
					delay_ms(300);
					i++;
				}
				}while(grayscale4()&&grayscale2());
		}
		};break;
	

}
		}



//X正向循迹方向判断
void derictionx1(void)
{
		if(grayscale7()==1&&grayscale8()==0)
		{
			speedy=-3;
		}
		else if(grayscale7()==0&&grayscale8()==1)
		{
			speedy=3;
		}
		else 
		{
			speedy=0;
		}
}

//X负向循迹方向判断
void derictionx2(void)
{
	if(Reference_Angle>360)Reference_Angle-=360;
	if(Reference_Angle<0)Reference_Angle+=360;
	if(grayscale7()==1)
	{
		First_Angle=Reference_Angle-5;//车向左偏
		if(First_Angle>360)First_Angle-=360;
		if(First_Angle<0)First_Angle+=360;
	//while(grayscale8());
	}
	if(grayscale8()==1)
	{
		First_Angle=Reference_Angle+3;//车向右偏
		if(First_Angle>360)First_Angle-=360;
		if(First_Angle<0)First_Angle+=360;
		//while(grayscale7());
	}
		else
	{
		First_Angle=Reference_Angle;
	}
}

//Y正向循迹方向判断
void derictiony1(void)
{
	if(Reference_Angle>360)Reference_Angle-=360;
	if(Reference_Angle<0)Reference_Angle+=360;
	if(grayscale5()==1)									
	{
		speedx=3;
//		First_Angle=Reference_Angle-5;//车向左偏 应往右调整
//		//while(grayscale5());
//		if(First_Angle>360)First_Angle-=360;
//		if(First_Angle<0)First_Angle+=360;
	}
	if(grayscale6()==1&&grayscale5()==0)
	{
		speedx=-3;
//		First_Angle=Reference_Angle+3;
//		//First_Angle+=2;//车向右偏	应往左调整
//		//while(grayscale6());
//		if(First_Angle>360)First_Angle-=360;
//		if(First_Angle<0)First_Angle+=360;
	}
		else
	{
		First_Angle=Reference_Angle;
	}
}

//Y负向循迹方向判断
void derictiony2(void)
{
	if(Reference_Angle>360)Reference_Angle-=360;
	if(Reference_Angle<0)Reference_Angle+=360;
	if(grayscale5()==1)									
	{
		First_Angle=Reference_Angle-5;//车向左偏
		//while(grayscale5());
		if(First_Angle>360)Reference_Angle-=360;
		if(First_Angle<0)Reference_Angle+=360;
	}
	if(grayscale6()==1&&grayscale5()==0)
	{
		First_Angle=Reference_Angle+3;
		//First_Angle+=2;//车向右偏
		//while(grayscale6());
		if(First_Angle>360)Reference_Angle-=360;
		if(First_Angle<0)Reference_Angle+=360;
	}
		else
	{
		First_Angle=Reference_Angle;
	}
}

void GPIOA_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOE2,3,4
	
 
} 

void TIM12_Init(u16 arr,u16 psc)//定时器9初始化
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);///使能TIM7时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE);//允许定时器7更新中断
	
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_BRK_TIM12_IRQn ; //定时器9中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM14_Init(u16 arr,u16 psc)//定时器9初始化
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);///使能TIM14时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);//允许定时器7更新中断

	
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_TRG_COM_TIM14_IRQn ; //定时器14中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
u8 times=0;
void TIM8_TRG_COM_TIM14_IRQHandler ()//定时器14中断服务程序
{
	if(TIM_GetITStatus(TIM14,TIM_IT_Update)==SET) //溢出中断
	{
//		if(USART_RX_STA2&0x8000)
//		{
		USART_RX_STA2=0;
		USART2_Send_bytes(data,15);
			//i=300000;
			//while(i--);
		USART2_Send_bytes(data,15);
			//i=300000;
			//while(i--);
//		delay_ms(40);
		ic= Hex2Dec(USART_RX_BUF1[9]);
		if(ic==last_ic)ic=0;
		if(ic%10>3)ic=0;
		if(ic/10>3)ic=0;
		//OLED_ShowNum(0,6,ic,2,10);
		//}
		  //maintain_distance(distance,2,speed);
		TIM_ClearITPendingBit(TIM14,TIM_IT_Update);  //清除中断标志位
		

	}
}
u8 len;
int IC_OLED(void)
{	
		int temp_ic=0;

		//OLED_ShowNum(0,6,temp_ic,2,10);
		len=USART_RX_STA2&0x3fff;
		if(USART_RX_STA2&0x8000)
{		
		USART_RX_STA2=0;
		USART2_Send_bytes(data,15);
		delay_ms(70);
		temp_ic= Hex2Dec(USART_RX_BUF1[8]);
		OLED_ShowNum(0,6,temp_ic,2,10);
		
	}
	return temp_ic;
}

void TIM8_BRK_TIM12_IRQHandler ()//定时器12中断服务程序
{
	if(TIM_GetITStatus(TIM12,TIM_IT_Update)==SET) //溢出中断
	{	
		//oled_show();
		  maintain_distance(distance,2,speed);
			TIM_ClearITPendingBit(TIM12,TIM_IT_Update);  //清除中断标志位
		

	}
}
	
void  oled_show(void)//OLED显示函数
{
		static num;
		int falg;
		flag=PD0;
	
		num++;
		OLED_ShowNum(0,0,grayscale1(),3,10);
		OLED_ShowNum(0,1,grayscale2(),3,10);
		OLED_ShowNum(0,2,grayscale3(),3,10);
		OLED_ShowNum(0,3,grayscale4(),3,10);
		OLED_ShowNum(20,0,grayscale5(),3,10);
		OLED_ShowNum(20,1,grayscale6(),3,10);
		OLED_ShowNum(20,2,grayscale7(),3,10);
		OLED_ShowNum(20,3,grayscale8(),3,10);
//		OLED_ShowNum(20,0,grayscale9(),3,10);
//		OLED_ShowNum(20,1,grayscale10(),3,10);
//		OLED_ShowNum(20,2,grayscale11(),3,10);
//		OLED_ShowNum(20,3,grayscale12(),3,10);
		OLED_ShowNum(0,4,Angle,3,10);
		OLED_ShowNum(0,5,num,3,10);
		OLED_ShowNum(20,5,count_x,3,10);
		OLED_ShowNum(40,5,count_y,3,10);
		OLED_ShowNum(60,5,d,3,10);
		OLED_ShowNum(90,5,PD0,3,10);
		OLED_ShowNum(60,6,storage_line,3,10);
		OLED_ShowNum(0,6,ic,2,10);
}
//正向	X循迹计数
void countx1(void)
{
	switch(Reference_Angle)
	{
		case 180:
						if(grayscale3()==0)//||grayscale3()==0)
						{
							count_x++;
							if(count_x>4)count_x=4;
							while(!grayscale3());//&&grayscale3()));
						}break;
		case 360:	if(grayscale3()==0)//||grayscale3()==0)
						{
							count_x--;
							if(count_x<1)count_x=1;
							while(!grayscale3());//&&grayscale3()));
						}break;
					}
	}
//Y循迹时X线计数
void countx2(void)
{
	switch(Reference_Angle)
	{
		case 180:
						if(grayscale4()==0)//||grayscale3()==0)
						{
							count_x++;
							if(count_x>4)count_x=4;
							while(!grayscale4());//&&grayscale3()));
						}break;
		case 360:	if(grayscale4()==0)//||grayscale3()==0)
						{
							count_x--;
							if(count_x<1)count_x=1;
							while(!grayscale4());//&&grayscale3()));
						}break;
					}
	}
void county1(void)//Y向计数
{
	switch(Reference_Angle)
	{
		case 180:
						if(grayscale2()==0||grayscale4()==0)
							{
								count_y++;
								if(count_y>2)count_y=2;
								while(!(grayscale2()&&grayscale4()));
							}	break;
		case 360:	if(grayscale2()==0||grayscale4()==0)
							{
								count_y--;
								if(count_y<1)count_y=1;
								while(!(grayscale2()&&grayscale4()));
							}	break;	
//			default: if(grayscale2()==0||grayscale2()==0)
//										{
//											count_y--;
//											if(count_y<1)count_y=1;
//											while(!(grayscale1()&&!grayscale3()));
//										}	break;							
	}
}
void county2(void)//Y向计数
{
	switch(Reference_Angle)
	{
		case 180:
						if(grayscale1()==0||grayscale3()==0)
							{
								count_y++;
								if(count_y>2)count_y=2;
								while(!(grayscale1()&&grayscale3()));
							}	break;
		case 360:	if(grayscale1()==0||grayscale3()==0)
							{
								count_y--;
								if(count_y<1)count_y=1;
								while(!(grayscale1()&&!grayscale3()));
							}	break;
		default: if(grayscale1()==0||grayscale3()==0)
							{
								count_y--;
								if(count_y<1)count_y=1;
								while(!(grayscale1()&&!grayscale3()));
							}	break;
	}
}

void secure(void)//确保安全距离
{
	while(!PD0)Motor_stop();
	//Motor_start();

}
void one_delay(int time)//一次延时
{
	if(delay_flag==0)
		{
			delay_ms(time);
			delay_flag++;
		}
}
//蓝色方卡线循迹
void move_blue(void)
{
	while (1)
	{ 
		straight_y(20);
		oled_show();
		secure();
		if(!grayscale2())
			{
				count_y=2;
			while(count_x!=3)
			{
				straight_x(20);
				oled_show();
				secure();
			}
			delay_ms(300);
			Motor_whirl(360);
			Kinematic_Analysis(0,20,0);
			delay_ms(500);
			while(1)
			{
				straight_y(20);
				//one_delay(400);
				oled_show();
				secure();
				if(count_y==1)break;
				
				
			}
			//delay_flag=0;
			while(1)
			{
				straight_x(20);
				oled_show();
				secure();
				if(count_x==1)break;
			}
			while(1)
				{
				oled_show();
				straight_x(0);
				}
		}
	}
}


void move_grab_blue(int speed)//蓝方抓取循迹
{
	//countx1();
	
	Kinematic_Analysis(speed,head,-Dir_PID(First_Angle,Angle));
	
	//if(!PD0)												
	if(color_number==2)
	{
		//USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);//开启相关中断
		USART_Cmd(UART4, DISABLE);
		TIM_Cmd(TIM12,DISABLE);
		USART_Cmd(USART3, DISABLE);
		Kinematic_Analysis(0,0,0);
		grab();
		ball_num++;
		grab_Init();
		TIM_Cmd(TIM12,ENABLE);
		USART_Cmd(USART3, ENABLE);
		USART_Cmd(UART4, ENABLE);//使能串口4
		delay_ms(75);
		color_number=0;
 }
	
}
void move_grab_red()
{
	//countx1();
	
	Kinematic_Analysis(-5,head,-Dir_PID(First_Angle,Angle));
	if(!PD0)
	{
		Kinematic_Analysis(0,0,0);
		grab();
		ball_num++;
	}
}

void maintain_distance(int distance,int times,int speed)
{
	int temp=0;
	int j;
	u16 i=0;
	for(i=0;i<3;i++)
	{
	d+=(forward_distance());
	delay_ms(2);
	}
	d=d/3;
	/*for(j=0;j<3;j++)
	{
		temp+=(forward_distance());
		delay_ms(1);
		//while(!(TIM8CH1_CAPTURE_STA&0X80));
		//while(!TIM8CH1_CAPTURE_STA);
	}
	d=temp/3;*/
	//d=forward_distance();
		if(d>distance) //距离过远
		{
			head=speed;
			//speedy=speed;
		}
		else if(d<distance) 
		{
			head=-speed;
			//speedy=-speed;
		}
		else head=0;

}

void store(int speed, int low_speed)
{

	while(storage_line!=1)
	{
		  straight_x(speed);
			count_storage();
	}
	i=200000;
	while(i)
	{
		i--;
		straight_x(low_speed);
	}
	//Motor_stop();
	//放球
	//bal_num--;
	while(PD0)
	{
		straight_x(0);
	}
	while(storage_line!=2)//走到第二条线
	{
		count_storage();
		straight_x(speed);
	}
	i=200000;
	while(i)//多走一段距离
	{
		straight_x(low_speed);
		i--;
	}
	//Motor_stop();
	//放球
	//ball_num--;
	while(PD0)
	{
		straight_x(0);
	}
	while(storage_line!=3)//走到第三条线
	{
		count_storage();
		straight_x(speed);
	}
	i=200000;
	while(i)//多走一段距离
	{
		straight_x(low_speed);
		i--;
	}
	//Motor_stop();
	//放球
	//ball_num--;
	while(PD0)
	{
		straight_x(0);
	}
}

void switch_storage(int target,int floor, int speed)
{
	
	if(storage_line<target)
	{
		destination=1;//未到指定地点
		storage_direction=1;//正向行驶
		Kinematic_Analysis(speed,head,-Dir_PID(First_Angle,Angle));
	}
	if(storage_line>target)
	{
		destination=1;			//未到指定地点
		storage_direction=0;//反向行驶
		Kinematic_Analysis(-speed,head,-Dir_PID(First_Angle,Angle));
	}
	if(storage_line==target)
	{
		
		Kinematic_Analysis(0,0,0);
		//USART_Cmd(USART3, DISABLE);
		//TIM_Cmd(TIM12,DISABLE);
		floors_house(floor);//放球
		//TIM_Cmd(TIM12,ENABLE);
		//USART_Cmd(USART3, ENABLE);
	
		destination=0;//到达指定地点
	}
	timesy++;
	if(timesy>15)//一段时间后计数
	{
		count_storage();
	}
	}
void switch_storage1(int target, int speed)
{
	
	if(storage_line<target)
	{
		destination=1;//未到指定地点
		storage_direction=1;//正向行驶
		Kinematic_Analysis(speed,head,-Dir_PID(First_Angle,Angle));
	}
	if(storage_line>target)
	{
		destination=1;			//未到指定地点
		storage_direction=0;//反向行驶
		Kinematic_Analysis(-speed,head,-Dir_PID(First_Angle,Angle));
	}
	if(storage_line==target)
	{
		
		Kinematic_Analysis(0,0,0);
		//USART_Cmd(USART3, DISABLE);
		//TIM_Cmd(TIM12,DISABLE);
		//TIM_Cmd(TIM12,ENABLE);
		//USART_Cmd(USART3, ENABLE);
	
		destination=0;//到达指定地点
	}
	timesy++;
	if(timesy>15)//一段时间后计数
	{
		count_storage();
	}
	}

void count_storage(void)
{
	switch(storage_direction)
	{
		case 1:
						if(grayscale9()==0)
						{
							storage_line++;
							//delay_ms(100);
							if(storage_line>4)storage_line=4;
							while(!grayscale9())Kinematic_Analysis(15,head,-Dir_PID(First_Angle,Angle));
						}
						break;
		case 0:
						if(grayscale9()==0)
						{
							storage_line--;
							if(storage_line<0)storage_line=0;
							//delay_ms(100);
							while(!grayscale9())Kinematic_Analysis(-15,head,-Dir_PID(First_Angle,Angle));
						}
						break;
	
	}
		
}
void disable(void)
{
		TIM_Cmd(TIM7, DISABLE);
		TIM_Cmd(TIM12, DISABLE);
		TIM_Cmd(TIM5, DISABLE);
		USART_Cmd(USART3, DISABLE); 
		TIM_Cmd(TIM8,DISABLE);
}

void enable(void)
{
		TIM_Cmd(TIM7, ENABLE);
		TIM_Cmd(TIM12, ENABLE);
		TIM_Cmd(TIM5, ENABLE);
		USART_Cmd(USART3, ENABLE); 
		TIM_Cmd(TIM8,ENABLE);

}

/*
void test_light(void)
{
	if(PD0==0)//||grayscale3()==0)
						{
							color_flag=1++;
							if(count_x>4)count_x=4;
							while(!grayscale3());//&&grayscale3()));
						}

}*/



