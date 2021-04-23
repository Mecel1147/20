#include "motor.h"
#include "iic.h"
#include "oled.h"
#include "attitude.h"
#include "grayscale.h"
#include "delay.h"
extern  int Angle;
extern  int Reference_Angle;
extern  int First_Angle;
extern 	int i;
int num;
//float Kp= 1,Ki=0,Kd = 0;
#define PWM_MAX 499
#define PWM_MIN -499
#define a_PARAMETER (0.64f)               
#define b_PARAMETER (0.52f)     
#define Target_Max 350

float Kp= 40,Ki=30,Kd = 0;


void Motor_Target_Limit(short int *A,short int *B,short int *C,short int *D)
{
	short int max,temp[4];
	u8 i;
	
	if(*A>Target_Max || *B>Target_Max || *C>Target_Max || *D>Target_Max)//当计算出来的目标值过大时需要进行限制  
	{
		temp[0] = *A;
		temp[1] = *B;
		temp[2] = *C;
		temp[3] = *D;
		max = temp[0];
		for(i=1;i<4;i++)//找出最大值
		{
			if(temp[i]>max)
			{
				max = temp[i];
			}
		}
		*A = temp[0] * (float)(Target_Max/max);
		*B = temp[1] * (float)(Target_Max/max);
		*C = temp[2] * (float)(Target_Max/max);
		*D = temp[3] * (float)(Target_Max/max);		
	}
}


int Motor_A_PI(int Encoder,int Target)
{ 
  static int bias,pwm,Last_Bias,Last_Last_Bias;
	bias = Encoder-Target;//计算偏差                
	pwm -= Kp*(bias-Last_Bias)+Ki*bias;//增量式PI控制器  
	Last_Bias = bias;//保存上一次偏差	                  
	pwm = Motor_Pwm_Limit(pwm);
	return pwm;  
}
int Motor_B_PI(int Encoder,int Target)
{   
  static int bias,pwm,Last_Bias,Last_Last_Bias;
	bias = Encoder-Target;                
	pwm -= Kp*(bias-Last_Bias)+Ki*bias;  
	Last_Bias = bias;	                  
	pwm = Motor_Pwm_Limit(pwm);
//	if(pwm>499)pwm=499;
//	if(pwm<0)pwm=0;
	return pwm; 
}
int Motor_C_PI(int Encoder,int Target)
{   
  static int bias,pwm,Last_Bias,Last_Last_Bias;
	bias = Encoder-Target;                
	pwm -= Kp*(bias-Last_Bias)+Ki*bias;  
	Last_Bias = bias;	                  
	pwm = Motor_Pwm_Limit(pwm);
	return pwm; 
}
int Motor_D_PI(int Encoder,int Target)
{   
  static int bias,pwm,Last_Bias,Last_Last_Bias;
	bias = Encoder-Target;
	pwm -= Kp*(bias-Last_Bias)+Ki*bias;  
	Last_Bias = bias;	                  
	pwm = Motor_Pwm_Limit(pwm);
	
	
	return pwm; 
}


void Caculate_Encoder(int temp[])
{
	extern int Encoder_Date[];	
	if(TIM2->CR1&0x10)//A15 B3 Motor_A
	{
	  Encoder_Date[0] = 0xffffffff-TIM2->CNT;
		TIM2->CNT = 0xffffffff;
	}
	else 
	{
		Encoder_Date[0] = TIM2->CNT;
		TIM2->CNT = 0;
	}
	if(TIM3->CR1&0x10)//A6 A7 Motor_C
	{
		Encoder_Date[2] = 0xffff-TIM3->CNT;
		TIM3->CNT = 0xffff;
	}
	else 
	{
		Encoder_Date[2] = TIM3->CNT;
		TIM3->CNT = 0;		
	}
	if(TIM4->CR1&0x10)//D12 D13 Motor_B
	{
		Encoder_Date[1] = 0xffff-TIM4->CNT;
		TIM4->CNT = 0xffff;
	}
	else 
	{
		Encoder_Date[1] = TIM4->CNT;
		TIM4->CNT = 0;	
	}
	if(TIM5->CR1&0x10)//A0 A1 Motor_D
	{
		Encoder_Date[3] = 0xffffffff-TIM5->CNT;
		TIM5->CNT = 0xffffffff;
	}
	else 
	{
		Encoder_Date[3] = TIM5->CNT;
		TIM5->CNT = 0;	
	}
}

//配置TIM7
void TIM7_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);///使能TIM7时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);//允许定时器7更新中断
	TIM_Cmd(TIM7,ENABLE);//使能定时器7
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //定时器7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


int Motor_Pwm_Limit(int pwm)
{
	if(pwm>PWM_MAX)pwm = PWM_MAX;
	if(pwm<PWM_MIN)pwm = PWM_MIN;
	return pwm;
}

//配置TIM1_PWM四路通道 
void TIM1_Pwm_Init(u16 arr,u16 psc) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);//使能PORTE时钟	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);//TIM1时钟使能 
	         
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9 ,GPIO_AF_TIM1);//GPIOE9复用为定时器1
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_11| GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOE,&GPIO_InitStructure);
  	
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;   
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInitStructure.TIM_Period = arr;    
  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	 
  TIM_OC1Init(TIM1,&TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM1,&TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM1,&TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM1,&TIM_OCInitStructure);
  
  TIM_Cmd(TIM1,ENABLE);
  TIM_CtrlPWMOutputs(TIM1,ENABLE);
}



//配置Motor_Dir
void Motor_IO_Init(void)   
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	
	Motor_A_1 = 0; 
	Motor_A_2 = 0;   
	Motor_B_2 = 0;    
	Motor_B_1 = 0;    
	Motor_C_1 = 0;    
	Motor_C_2 = 0;  
	Motor_D_1 = 0;    
	Motor_D_2 = 0;    
}


//配置编码器


	void TIM2_ENC_Init(void)//Motor_A
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能TIM2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//最高速度为100MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//引脚复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//开漏输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2);//将PA15引脚配置为TIM2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2);//将PB3引脚配置为TIM2
	
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI1,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//编码器接口配置

	TIM_Cmd(TIM2, ENABLE);//使能TIM2 
}
void TIM3_ENC_Init(void)//Motor_C
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM3, ENABLE); 
}
void TIM4_ENC_Init(void)//Motor_B
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_Cmd(TIM4, ENABLE); 
}
void TIM5_ENC_Init(void)//Motor_D
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	TIM_TimeBaseStructure.TIM_Prescaler = 1-1;					//????????1,????
//	TIM_TimeBaseStructure.TIM_Period = 499;					//????????????(?????)??????(???),65535???????,????
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //????????????,????????TI1?TI2??????
//	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
	
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_SetCounter(TIM5, 0);
	TIM_Cmd(TIM5, ENABLE); 
}



//函数功能：小车运动数学模型
//入口参数：X Y Z 三轴速度或者位置


void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
		short int temp[4];
		temp[0] = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
		temp[1] = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
		temp[2] = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
		temp[3] = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
	
		if(temp[0]>0)  //A +
		{
			Motor_A_1 = 1;
			Motor_A_2 = 0;			
		}
		else if(temp[0]<0) 	//A -
		{
			Motor_A_1 = 0;
			Motor_A_2 = 1;
			temp[0]*= -1;
		}
		else 								//A  ..
		{
			Motor_A_1 = 0;
			Motor_A_2 = 0;
		}
		
		if(temp[1]>0)
		{
			Motor_B_1 = 1;
			Motor_B_2 = 0;			
		}
		else if(temp[1]<0) 
		{
			Motor_B_1 = 0;
			Motor_B_2 = 1;			
			temp[1] *= -1;
		}
		else 
		{
			Motor_B_1 = 0;
			Motor_B_2 = 0;
		}
		
		if(temp[2]>0)
		{
			Motor_C_1 = 1;
			Motor_C_2 = 0;			
		}
		else if(temp[2]<0) 
		{
			Motor_C_1 = 0;
			Motor_C_2 = 1;			
			temp[2]*= -1;
		}
		else 
		{
			Motor_C_1 = 0;
			Motor_C_2 = 0;
		}
		
		if(temp[3]>0)
		{
			Motor_D_1 = 1;
			Motor_D_2 = 0;			
		}
		else if(temp[3]<0) 
		{
			Motor_D_1 = 0;
			Motor_D_2 = 1;			
  		temp[3]*= -1;
	  }
		else 
		{
			Motor_D_1 = 0;
			Motor_D_2 = 0;
		}
	
		//Motor_Target_Limit(temp,temp+1,temp+2,temp+3);	
    TIM_Cmd(TIM7,DISABLE);
		Target_A = temp[0];
		Target_B = temp[1];
		Target_C = temp[2];
		Target_D = temp[3];
    TIM_Cmd(TIM7,ENABLE);
}

//定时器7中断服务函数
void TIM7_IRQHandler(void)
{
	extern int Encoder_Date[],Target_Speed[],PWM[];	
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) //溢出中断
	{	
		Caculate_Encoder(Encoder_Date);
		PWM[0] = Motor_A_PI(Encoder_Date[0],Target_Speed[0]);
		PWM[1] = Motor_B_PI(Encoder_Date[1],Target_Speed[1]);
		PWM[2] = Motor_C_PI(Encoder_Date[2],Target_Speed[2]);
		PWM[3] = Motor_D_PI(Encoder_Date[3],Target_Speed[3]);
		TIM_SetCompare1(TIM1,500 + PWM[3]);//Motor_D
    TIM_SetCompare2(TIM1,500 + PWM[2]);//Motor_C
		TIM_SetCompare3(TIM1,500 + PWM[1]);//Motor_B
		TIM_SetCompare4(TIM1,500 + PWM[0]);//Motor_A		
	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //清除中断标志位
}


void Motor_Init(void)
{
  TIM1_Pwm_Init(1000-1,8-1);
	TIM2_ENC_Init();
	TIM3_ENC_Init();
	TIM4_ENC_Init();
	TIM5_ENC_Init();
  TIM7_Init(1000-1,84-1);	//84Mhz/840=100khz 100k /2k=50 
	Motor_IO_Init();
}



void Motor_start(void)
{
	Motor_A_1 = 1; 
	Motor_A_2 = 0;
	Motor_B_1 = 1;  
	Motor_B_2 = 0;
	Motor_C_1 = 1; 
	Motor_C_2 = 0; 
	Motor_D_1 = 1; 
	Motor_D_2 = 0; 
}

void Motor_stop(void)
{
	Motor_A_1 = 0; 
	Motor_A_2 = 0;
	Motor_B_1 = 0;  
	Motor_B_2 = 0;
	Motor_C_1 = 0; 
	Motor_C_2 = 0; 
	Motor_D_1 = 0; 
	Motor_D_2 = 0; 
}

void Motor_whirl(int angle)
{
	/*TIM_Cmd(TIM7,DISABLE);//失能定时器7
	Motor_A_1 = 1; //电机转向相反
	Motor_A_2 = 0;
	Motor_B_1 = 1;  
	Motor_B_2 = 0;
	Motor_C_1 = 0; 
	Motor_C_2 = 1; 
	Motor_D_1 = 0; 
	Motor_D_2 = 1;
	
	TIM_SetCompare1(TIM1,800);//Motor_D
  TIM_SetCompare2(TIM1,800);//Motor_C
	TIM_SetCompare3(TIM1,800);//Motor_B
	TIM_SetCompare4(TIM1,800);//Motor_A		
	
	delay_ms(900);
	
	Reference_Angle=0;//回归电机转向
	Motor_A_1 = 1; 
	Motor_A_2 = 0;
	Motor_B_1 = 1;  
	Motor_B_2 = 0;
	Motor_C_1 = 1; 
	Motor_C_2 = 0; 
	Motor_D_1 = 1; 
	Motor_D_2 = 0; 
	
	TIM_Cmd(TIM7,ENABLE);//使能定时器7**/
	Kinematic_Analysis(0,0,0);
	Reference_Angle+=angle;
	if(Reference_Angle>360)Reference_Angle-=360;
	
	delay_ms(200);
	if(angle==360) angle=0;
	i=600000;
	while(i)
	{
	//oled_show();
		i--;
	Kinematic_Analysis(0,0,-Dir_PID(Reference_Angle,Angle));
	}
	//delay_ms(200);
	First_Angle=Reference_Angle;
	
}
