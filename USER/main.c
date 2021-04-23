#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "servo.h"
#include "oled.h"
#include "iic.h"
#include "motor.h"
#include "attitude.h"
#include "function.h"
#include "grayscale.h"
#include "distance.h"
#include "timer.h"
#include "color.h"
int first_ball_num;
int last_ic;
int ic;
int distance=0;
int speed=0;
int timesy=0;
int destination=1;
int Encoder_Date[4],Target_Speed[4] ={0} ,PWM[4];
//仓库线
int storage_line=0; 
//串口3 姿态传感器
extern u8 ReceiveBuff[];
int flag=0;
int count_x=0;
int count_y=0;
//延时标志
int delay_flag=0;
//角度数据
extern int Angle;
int First_Angle;
//物件数量
int ball_num=0;
//前超声波距离
int head;
//颜色数据
extern char* color_date;
extern u16 color_number;
//OPENMV数据
extern u32 OPENMV_Data[4];
	
int Reference_Angle;
//偏转角度
//int	bias=0;
u8 DataReadBuf[16];


	 int i=0;

	int Hex2Dec(int Hex)//十六进制转换为十进制
{
    int dec=0,temp=0;
    temp= Hex & 0x0f;  //
    dec = (Hex >> 4) *10 + temp;//高4位×10+低四位
    return dec;
}


int main(void)
{ 
	int i = 0;
	int num=0;
	int j=0;
	
	BEEP_Init();  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(115200);	//串口初始化波特率为115200
	uart2_init(9600);
	LED_Init();		  		//初始化与LED连接的硬件接口  	
	IIC_Init();
	Light_Init();
	OLED_Init();//oled初始化
	Motor_Init();
	ADC_DMA_Init();
	ECHO_Init();
	TRIG_Init();
  USART3_DMA_Init();
	Motor_start();
	delay_ms(100);
	TIM14_Init(10000-1,8400-1);//84m/8400=10k  10k/5k
	TIM12_Init(10000-1,840-1); //84m/8400=10k  10k/5000=2hz
	Reference_Angle=180;
	First_Angle=Reference_Angle;
	UART4_Init(115200);//接收OPENMV数据
	Color_Init();//接收OPENMV数据标志
  DJ_Init();
	//secure();
	delay_flag=0;
	count_x=1;
	count_y=1;
	i=280000;
	ball_num=3;
	
//	while(1)
//{
//	TIM_Cmd(TIM12,ENABLE);//失能定时器12
//	TIM_Cmd(TIM14,ENABLE);//失能定时器12
//	oled_show();
//}

	//红方
	while(0)
	{
		while(grayscale7())Kinematic_Analysis(-30,0,-Dir_PID(First_Angle,Angle));//沿X正向从场外走到场地处 红方
		while(i)//快进
		{
			straight_y(50);//快进
			i--;
		}
		straight_y(30);//慢进，从场外进入场地
		oled_show();
		if(!grayscale4())//走到边线改变运动方向
			{
				count_y=2; //走到第二条线
				i=100000;
				while(i)
				{
						Kinematic_Analysis(-20,0,-Dir_PID(First_Angle,Angle));
					i--;
				}
				i=100000;
				while(i)
				{
						Kinematic_Analysis(-30,0,-Dir_PID(First_Angle,Angle));
					i--;
				}
				i=100000;
				while(i)
				{
						Kinematic_Analysis(-40,0,-Dir_PID(First_Angle,Angle));
					i--;
				}
				
				i=200000;
				while(i)
				{
						Kinematic_Analysis(-50,0,-Dir_PID(First_Angle,Angle));
					i--;
				}
				
				while(count_x!=2)//走到第二条线之后准备抓取
				{
					straight_x(-60);
					oled_show();
				}
				i=20;
				while(i)//向前继续行走一小段距离
				{
					oled_show();
					
					Kinematic_Analysis(-16,6,-Dir_PID(First_Angle,Angle));
					i--;
				}
				USART_Cmd(UART4, ENABLE);//OPENMV串口失能
				TIM_Cmd(TIM12,ENABLE);//使能定时器12
				i=430000;	//平台2
				for(ball_num=0;ball_num<4;)//抓四次球之后跳出循环
				{
					distance=22;
					speed=2;
					move_grab_blue(-7);	
					first_ball_num=ball_num;
					if(i==0)break;//超时跳出循环
					i--;
				}
				USART_Cmd(UART4, DISABLE);//OPENMV串口失能
				TIM_Cmd(TIM12,DISABLE);//失能定时器12
				
				while(grayscale7()!=0)//退回到线上
				{
					oled_show();
					Kinematic_Analysis(0,-20,-Dir_PID(First_Angle,Angle));
				}
				//平台1抓取结束
				i=40;
				while(i)
				{
					straight_x(-50);
					oled_show();
					i--;
				}
				while(count_x!=4)//向前走，走到X第四条线转身
				{
					straight_x(-40);
					//oled_show();
				}
					//	delay_ms(100);  //平台1 到平台2 
						Motor_whirl(90);
						i=20;
						while(i)//向前继续行走一小段距离
						{
							oled_show();
							Kinematic_Analysis(-25,15,-Dir_PID(First_Angle,Angle));
							i--;
						}
						USART_Cmd(UART4, ENABLE);//OPENMV串口失能
						TIM_Cmd(TIM12,ENABLE);//使能定时器12
							i=420000;	//平台2
						for(;ball_num<first_ball_num+4;)//抓四次球之后跳出循环
						{
							distance=21;
							speed=3;
							move_grab_blue(-7);					
							if(i==0)break;//超时跳出循环
							i--;
						}
						USART_Cmd(UART4, DISABLE);//OPENMV串口失能
						TIM_Cmd(TIM12,DISABLE);//失能定时器12
						Motor_whirl(90);
						while(grayscale7()!=0)//退回到线上
						{
							oled_show();
							Kinematic_Analysis(-20,0,-Dir_PID(First_Angle,Angle));
						}
						
						while(count_y!=1)//走到Y第一条线变向
					{
						straight_y(25);
						oled_show();
					}
					i=20;
					while(i)
					{
						Kinematic_Analysis(-30,0,-Dir_PID(First_Angle,Angle));
						i--;
						oled_show();
					}
					while(count_x!=3)
					{
						straight_x(-50);
					
					}
						i=8;
					while(i)//向前继续行走一小段距离
					{
						oled_show();
						Kinematic_Analysis(-36,3,-Dir_PID(First_Angle,Angle));
						i--;
					}
					Kinematic_Analysis(0,0,0);
					USART_Cmd(UART4, DISABLE);//OPENMV
					USART_Cmd(USART3,ENABLE);
					while(1)
						{		
							i=2000000;
							ic=0;
								TIM_Cmd(TIM14,ENABLE);//定时器14使能
								while(!ic)//等待IC卡读取
							{
								i--;
								if(i==0)break;
							}
								TIM_Cmd(TIM14,DISABLE);
								//TIM_Cmd(TIM12,ENABLE);//定时器12使能
								speed=3;
								distance=6;
								while(destination)
							{
								if(i==0)switch_storage(last_ic/10+1,1,10);
								else
								{
									maintain_distance(distance,3,3);
									switch_storage(ic/10+1,ic%10,10);
									oled_show();
								}
							}
							
							//TIM_Cmd(TIM12,DISABLE);//失能定时器12
							timesy=0;
							destination=1;
							last_ic=ic;
							speed=0;
							ball_num--;
						}
						switch_storage1(4,10);//到第四根线
						USART_Cmd(USART3,ENABLE);
				while(grayscale7()!=0)//退回到线上
						{
							oled_show();
							Kinematic_Analysis(0,-20,-Dir_PID(First_Angle,Angle));
						}
				while(count_x!=1)
				{
					straight_x(40);
					oled_show();
				}
				i=36;
				while(i)
				{
					straight_y(-25);
					oled_show();
					i--;
				}
				i=22;
				while(i)
				{
					straight_x(19);
					oled_show();
					i--;
				}
				while(1)
				{
					Kinematic_Analysis(0,0,0);
			    End_recycle();         //回收机械臂
				}
			}
	}
//红方
	while(0)
	{
		while(i)//快进
		{
			straight_x(40);//快进
			county2();
			i--;
		}
		straight_x(20);//慢进，从场外进入场地
		county2();
		oled_show();
		if(!grayscale1())//走到边线改变运动方向
			{
				count_x=1;
				count_y=2; //走到第二条线
				i=200000;
				while(i)
				{
					Kinematic_Analysis(0,40,-Dir_PID(First_Angle,Angle));
					i--;
				}
				while(count_x!=2)//走到第二条线之后准备抓取
				{
					straight_y(60);
					countx2();
					oled_show();
				}
				Motor_whirl(270);
				i=15;
				while(i)//向前继续行走一小段距离
				{
					oled_show();
					
					Kinematic_Analysis(-7,15,-Dir_PID(First_Angle,Angle));
					i--;
				}
				USART_Cmd(UART4, ENABLE);//OPENMV串口失能
				TIM_Cmd(TIM12,ENABLE);//使能定时器12
				i=430000;	//平台2
				for(ball_num=0;ball_num<4;)//抓四次球之后跳出循环
				{
					distance=22;
					speed=2;
					move_grab_blue(-7);	
					first_ball_num=ball_num;
					if(i==0)break;//超时跳出循环
					i--;
				}
				USART_Cmd(UART4, DISABLE);//OPENMV串口失能
				TIM_Cmd(TIM12,DISABLE);//失能定时器12
				Motor_whirl(90);
				while(grayscale7()!=0)//退回到线上
				{
					oled_show();
					Kinematic_Analysis(-20,0,-Dir_PID(First_Angle,Angle));
				}
				//平台1抓取结束
				i=40;
				while(i)
				{
					straight_y(50);
					countx2();
					oled_show();
					i--;
				}
				while(count_x!=4)//向前走，走到X第四条线转身
				{
					straight_y(30);
					countx2();
					//oled_show();
				}
						//delay_ms(100);  //平台1 到平台2 
						//Motor_whirl(90);
						i=20;
						while(i)//向前继续行走一小段距离
						{
							oled_show();
							Kinematic_Analysis(-15,15,-Dir_PID(First_Angle,Angle));
							i--;
						}
						USART_Cmd(UART4, ENABLE);//OPENMV串口失能
						TIM_Cmd(TIM12,ENABLE);//使能定时器12
							i=420000;	//平台2
						for(;ball_num<first_ball_num+4;)//抓四次球之后跳出循环
						{
							distance=21;
							speed=3;
							move_grab_blue(-7);					
							if(i==0)break;//超时跳出循环
							i--;
						}
						USART_Cmd(UART4, DISABLE);//OPENMV串口失能
						TIM_Cmd(TIM12,DISABLE);//失能定时器12
						Motor_whirl(270);
						while(grayscale7()!=0)//退回到线上
						{
							oled_show();
							Kinematic_Analysis(-20,0,-Dir_PID(First_Angle,Angle));
						}
						//Motor_whirl(270);
						while(count_y!=1)//走到Y第一条线变向
					{
						straight_y(-25);
						county1();
						oled_show();
					}
					i=0;
					while(i)//转角处后退
					{
						straight_y(-15);
						i--;
						oled_show();
					}
					while(count_x!=3)
					{
						straight_y(-50);
						county2();
					}
					Motor_whirl(270);
						i=8;
					while(i)//向前继续行走一小段距离
					{
						oled_show();
						Kinematic_Analysis(-36,3,-Dir_PID(First_Angle,Angle));
						i--;
					}
					Kinematic_Analysis(0,0,0);
					USART_Cmd(UART4, DISABLE);//OPENMV
					USART_Cmd(USART3,ENABLE);
					while(ball_num)
						{		
							ic=0;
								TIM_Cmd(TIM14,ENABLE);//定时器14使能
								while(!ic)//等待IC卡读取
							{
								
							}
								TIM_Cmd(TIM14,DISABLE);
								//TIM_Cmd(TIM12,ENABLE);//定时器12使能
								speed=-3;
								distance=6;
								while(destination)
							{
								maintain_distance(distance,3,3);
								switch_storage(ic/10+1,ic%10,-10);
								oled_show();
							}
							
							//TIM_Cmd(TIM12,DISABLE);//失能定时器12
							timesy=0;
							destination=1;
							last_ic=ic;
							speed=0;
							ball_num--;
						}
						switch_storage1(4,-10);//到第四根线
						USART_Cmd(USART3,ENABLE);
				while(grayscale7()!=0)//退回到线上
						{
							oled_show();
							Kinematic_Analysis(0,-20,-Dir_PID(First_Angle,Angle));
						}
				while(count_x!=1)
				{
					straight_x(-40);
					oled_show();
				}
				i=36;
				while(i)
				{
					straight_y(-25);
					oled_show();
					i--;
				}
				i=22;
				while(i)
				{
					straight_x(-19);
					oled_show();
					i--;
				}
				while(1)
				{
					Kinematic_Analysis(0,0,0);
			    End_recycle();         //回收机械臂
				}
			}
	}

//蓝方
	while(1)
	{
		while(grayscale5())Kinematic_Analysis(40,0,0);//沿X正向从场外走到场地处 蓝方
		while(i)//快进
		{
			straight_y(50);//快进
			i--;
		}
		straight_y(20);//慢进，从场外进入场地
		oled_show();
		if(!grayscale2())//走到边线改变运动方向
			{
				count_y=2; //走到第二条线
				while(count_x!=2)//走到第二条线之后准备抓取
				{
					straight_x(60);
					oled_show();
				}
				i=20;
				while(i)//向前继续行走一小段距离
				{
					oled_show();
					
					Kinematic_Analysis(16,6,-Dir_PID(First_Angle,Angle));
					i--;
				}
				USART_Cmd(UART4, ENABLE);//OPENMV串口失能
				TIM_Cmd(TIM12,ENABLE);//使能定时器12
				
				
				
				i=4300000;	//平台1
				for(ball_num=0;ball_num<4;)//抓四次球之后跳出循环
				{
					distance=22;
					speed=2;
					move_grab_blue(7);	
					first_ball_num=ball_num;
					if(i==0)break;//超时跳出循环
					i--;
				}
				USART_Cmd(UART4, DISABLE);//OPENMV串口失能
				TIM_Cmd(TIM12,DISABLE);//失能定时器12
				
				while(grayscale7()!=0)//退回到线上
				{
					oled_show();
					Kinematic_Analysis(0,-20,-Dir_PID(First_Angle,Angle));
				}
				//平台1抓取结束
				i=40;
				while(i)
				{
					straight_x(50);
					oled_show();
					i--;
				}
				while(count_x!=4)//向前走，走到X第四条线转身
				{
					straight_x(30);
					//oled_show();
				}
						delay_ms(100);  //平台1 到平台2 
						Motor_whirl(270);
						i=20;
						while(i)//向前继续行走一小段距离
						{
							oled_show();
							Kinematic_Analysis(25,15,-Dir_PID(First_Angle,Angle));
							i--;
						}
						USART_Cmd(UART4, ENABLE);//OPENMV串口失能
						TIM_Cmd(TIM12,ENABLE);//使能定时器12
						

						i=4200000;	//平台2
						for(;ball_num<first_ball_num+4;)//抓四次球之后跳出循环
						{
							distance=21;
							speed=3;
							move_grab_blue(7);					
							if(i==0)break;//超时跳出循环
							i--;
						}
						USART_Cmd(UART4, DISABLE);//OPENMV串口失能
						TIM_Cmd(TIM12,DISABLE);//失能定时器12
						
						while(grayscale7()!=0)//退回到线上
						{
							oled_show();
							Kinematic_Analysis(0,-20,-Dir_PID(First_Angle,Angle));
						}
						Motor_whirl(270);
						while(count_y!=1)//走到Y第一条线变向
					{
						straight_y(25);
						oled_show();
					}
					i=0;
					while(i)//转角处后退
					{
						straight_y(-15);
						i--;
						oled_show();
					}
					while(count_x!=3)
					{
						straight_x(50);
					
					}
						i=8;
					while(i)//向前继续行走一小段距离
					{
						oled_show();
						Kinematic_Analysis(36,8,-Dir_PID(First_Angle,Angle));
						i--;
					}
					Kinematic_Analysis(0,0,0);
					
					
					USART_Cmd(UART4, DISABLE);//OPENMV
					USART_Cmd(USART3,ENABLE);
					
					ball_num=8;
					while(ball_num)
						{		
							Kinematic_Analysis(0,0,0);
							i=200000000;
							ic=0;
								TIM_Cmd(TIM14,ENABLE);//定时器14使能
								while(!ic)//等待IC卡读取
							{
								Kinematic_Analysis(0,0,0);
								i--;
								if(i==0)break;
							}
								Kinematic_Analysis(0,0,0);
								TIM_Cmd(TIM14,DISABLE);
								//TIM_Cmd(TIM12,ENABLE);//定时器12使能
								speed=3;
								distance=6;
								while(destination)
							{
								if(i==0)switch_storage(last_ic/10+1,1,10);
									else
									{
									maintain_distance(distance,3,3);
									switch_storage(ic/10+1,ic%10,10);
									oled_show();
									}
							}
							
							//TIM_Cmd(TIM12,DISABLE);//失能定时器12
							timesy=0;
							destination=1;
							last_ic=ic;
							speed=0;
							ball_num--;
							Kinematic_Analysis(0,0,0);
						}
						switch_storage1(4,10);//到第四根线
						USART_Cmd(USART3,ENABLE);
						i=20;
						while(i)
						{
							oled_show();
							Kinematic_Analysis(20,0,-Dir_PID(First_Angle,Angle));
							i--;
						}
				while(grayscale7()!=0)//退回到线上
						{
							oled_show();
							Kinematic_Analysis(0,-20,-Dir_PID(First_Angle,Angle));
						}
				while(count_x!=1)
				{
					straight_x(40);
					oled_show();
				}
				i=36;
				while(i)
				{
					straight_y(-25);
					oled_show();
					i--;
				}
				i=22;
				while(i)
				{
					straight_x(19);
					oled_show();
					i--;
				}
				while(1)
				{
					Kinematic_Analysis(0,0,0);
			    End_recycle();         //回收机械臂
				}
			}
	}
}
	
		

//int main(void)
//{	
//	
//	LED_Init();		        //初始化LED端口
//	
////	PUT();
////	start_workspace_red();
////	grab_wedges(1);
//    while(1)
//	{	
//		if(color_number==2)
//			{
//		GPIO_ResetBits(GPIOF,GPIO_Pin_9);  //LED0对应引脚GPIOF.9拉低，亮  等同LED0=0;
////	  GPIO_SetBits(GPIOF,GPIO_Pin_10);   //LED1对应引脚GPIOF.10拉高，灭 等同LED1=1;
////	  delay_ms(500);  		   //延时300ms
////	  GPIO_SetBits(GPIOF,GPIO_Pin_9);	   //LED0对应引脚GPIOF.0拉高，灭  等同LED0=1;
//	  GPIO_ResetBits(GPIOF,GPIO_Pin_10); //LED1对应引脚GPIOF.10拉低，亮 等同LED1=0;
//    delay_ms(1000);        
//		
//		}
//		else if(color_number==3)
//			{
//		GPIO_ResetBits(GPIOF,GPIO_Pin_9);  //LED0对应引脚GPIOF.9拉低，亮  等同LED0=0;
//	  GPIO_SetBits(GPIOF,GPIO_Pin_10);   //LED1对应引脚GPIOF.10拉高，灭 等同LED1=1;
//	  delay_ms(500);  		   //延时300ms
//	  GPIO_SetBits(GPIOF,GPIO_Pin_9);	   //LED0对应引脚GPIOF.0拉高，灭  等同LED0=1;
//	  GPIO_ResetBits(GPIOF,GPIO_Pin_10); //LED1对应引脚GPIOF.10拉低，亮 等同LED1=0;
//    delay_ms(500);                     //延时300ms
//		}
//		else {
//		GPIO_SetBits(GPIOF,GPIO_Pin_10);   //LED1对应引脚GPIOF.10拉高，灭 等同LED1=1;
//		GPIO_SetBits(GPIOF,GPIO_Pin_9);	   //LED0对应引脚GPIOF.0拉高，灭  等同LED0=1;
//		}
//		
//		
////		//颜色数据
////		COLOR_JUDGE = 1;    //openMV使能
////		OLED_ShowChar(0,1,color_date,12);
////		OLED_ShowNum(0,1,OPENMV_Data[0],5,12);		
////		OLED_ShowNum(0,2,OPENMV_Data[1],5,12);	
////		OLED_ShowNum(0,3,OPENMV_Data[2],5,12);
////		OLED_ShowNum(0,4,OPENMV_Data[3],5,12);
////		OLED_ShowNum(0,5,color_number,5,12);
////英文显示		
////	if(color_number==1)
////	{color_date = "red   ";}
////	else if(color_number==2)
////	{color_date = "yellow";}	
////	else if(color_number==4)
////	{color_date = "blue  ";}
////	else{color_date = "0     ";}
////	OLED_ShowString(0,1,color_date,12);
//		
////		//角度
////		OLED_ShowNum(0,1,First_Angle,5,12);	
////		OLED_ShowNum(0,2,Angle,5,12);			
////		
////		//超声波数据
////		OLED_ShowNum(0,1,forward_distance(),5,12);
////		OLED_ShowNum(0,2,back_distance(),5,12);
////		OLED_ShowNum(0,3,left_distance(),5,12);
////		OLED_ShowNum(0,4,right_distance(),5,12);
////		
////		//灰度数据
////		OLED_ShowNum(0,1,ADC_Data[0],5,12);		
////		OLED_ShowNum(0,2,ADC_Data[7],5,12);	
////		OLED_ShowNum(0,3,ADC_Data[8],5,12);
////		OLED_ShowNum(0,4,ADC_Data[9],5,12);
////		OLED_ShowNum(0,5,ADC_Data[10],5,12);
////		OLED_ShowNum(0,6,ADC_Data[11],5,12);

//	}
//}

//		
//	
//	
	
	
	
	
	
//}
//		OLED_ShowNum(90,t,t,1,10);
//		t++;
//		if(t>7)
//		{
//			OLED_Clear();
//			t=1;
//		}

	

	//串口发送测试
//		for(i=0; i<3;i++)
//		{
//			USART_SendData(USART1,a[i]);         //向串口1发送数据
//				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}
//			if(USART_RX_STA&0x8000)
//			{
//				USART_RX_STA=0;
//				//printf("0F A2 52 09 01 01 60 FF FF FF FF FF FF 69 03\r\n");
//				delay_ms(100);
//				OLED_ShowString(1,t,USART_RX_BUF,10);
//				t++;
//		}


