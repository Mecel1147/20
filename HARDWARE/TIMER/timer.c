#include "timer.h"
#include "led.h"
#include "usart.h"
#include "delay.h"
void ECHO_Init(void)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM8_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);//TIM8时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//使能PORTC时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//GPIOC6/7/8/9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC6/7/8/9

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);//GPIO6复用位定时器8
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);//GPIO7复用位定时器8
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);//GPIO8复用位定时器8
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);//GPIO9复用位定时器8
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseStructure.TIM_Period=65535;//自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
	
	//初始化TIM8输入捕获参数
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//上升沿捕获
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//映射到TI1上
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//配置输入分频,不分频 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM8,&TIM8_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_2; 
  TIM_ICInit(TIM8,&TIM8_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_3; 
  TIM_ICInit(TIM8,&TIM8_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_4; 
  TIM_ICInit(TIM8,&TIM8_ICInitStructure);
		
	TIM_ClearFlag(TIM8,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC1/2/3/4IE捕获中断	
	
  TIM_Cmd(TIM8,ENABLE);//使能定时器8
  //中断配置
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

}  
u8  TIM8CH1_CAPTURE_STA=0;//输入捕获状态		    				
u16	TIM8CH1_CAPTURE_VAL;//输入捕获值
u8  TIM8CH2_CAPTURE_STA=0;//输入捕获状态		    				
u16	TIM8CH2_CAPTURE_VAL;//输入捕获值
u8  TIM8CH3_CAPTURE_STA=0;//输入捕获状态		    				
u16	TIM8CH3_CAPTURE_VAL;//输入捕获值
u8  TIM8CH4_CAPTURE_STA=0;//输入捕获状态		    				
u16	TIM8CH4_CAPTURE_VAL;//输入捕获值	
 

//定时器8中断服务程序	 
void TIM8_CC_IRQHandler(void)
{ 
	//前超声波
 	if((TIM8CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)	 
		{	    
			if(TIM8CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM8CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM8CH1_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM8CH1_CAPTURE_VAL=0XFFFF;
				}else TIM8CH1_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM8,TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM8CH1_CAPTURE_STA&0X40)//捕获到一个下降沿 		
			{	  			
				TIM8CH1_CAPTURE_STA|=0X80;//标记成功捕获到一次上升沿
				TIM8CH1_CAPTURE_VAL=TIM_GetCapture1(TIM8);
		   	TIM_OC1PolarityConfig(TIM8,TIM_ICPolarity_Rising);//CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM8CH1_CAPTURE_STA=0;//清空
				TIM8CH1_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM8,0);
				TIM8CH1_CAPTURE_STA|=0X40;//标记捕获到了上升沿
		   	TIM_OC1PolarityConfig(TIM8,TIM_ICPolarity_Falling);//CC1P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}
	
	//后超声波
 	if((TIM8CH2_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)	 
		{	    
			if(TIM8CH2_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM8CH2_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM8CH2_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM8CH2_CAPTURE_VAL=0XFFFF;
				}else TIM8CH2_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM8,TIM_IT_CC2) != RESET)//捕获2发生捕获事件
		{	
			if(TIM8CH2_CAPTURE_STA&0X40)//捕获到一个下降沿 		
			{	  			
				TIM8CH2_CAPTURE_STA|=0X80;//标记成功捕获到一次上升沿
				TIM8CH2_CAPTURE_VAL=TIM_GetCapture2(TIM8);
		   	TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Rising);//CC2P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM8CH2_CAPTURE_STA=0;//清空
				TIM8CH2_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM8,0);
				TIM8CH2_CAPTURE_STA|=0X40;//标记捕获到了上升沿
		   	TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Falling);//CC2P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}	

	//左超声波
 	if((TIM8CH3_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)	 
		{	    
			if(TIM8CH3_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM8CH3_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM8CH3_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM8CH3_CAPTURE_VAL=0XFFFF;
				}else TIM8CH3_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM8,TIM_IT_CC3) != RESET)//捕获3发生捕获事件
		{	
			if(TIM8CH3_CAPTURE_STA&0X40)//捕获到一个下降沿 		
			{	  			
				TIM8CH3_CAPTURE_STA|=0X80;//标记成功捕获到一次上升沿
				TIM8CH3_CAPTURE_VAL=TIM_GetCapture3(TIM8);
		   	TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Rising);//CC3P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM8CH3_CAPTURE_STA=0;//清空
				TIM8CH3_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM8,0);
				TIM8CH3_CAPTURE_STA|=0X40;//标记捕获到了上升沿
		   	TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Falling);//CC3P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}	
	
	//右超声波
 	if((TIM8CH4_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)	 
		{	    
			if(TIM8CH4_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM8CH4_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM8CH4_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM8CH4_CAPTURE_VAL=0XFFFF;
				}else TIM8CH4_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM8,TIM_IT_CC4) != RESET)//捕获4发生捕获事件
		{	
			if(TIM8CH4_CAPTURE_STA&0X40)//捕获到一个下降沿 		
			{	  			
				TIM8CH4_CAPTURE_STA|=0X80;//标记成功捕获到一次上升沿
				TIM8CH4_CAPTURE_VAL=TIM_GetCapture4(TIM8);
		   	TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Rising);//CC4P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM8CH4_CAPTURE_STA=0;//清空
				TIM8CH4_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM8,0);
				TIM8CH4_CAPTURE_STA|=0X40;//标记捕获到了上升沿
		   	TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Falling);//CC4P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}	
	
  TIM_ClearITPendingBit(TIM8,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update); //清除中断标志位
}


//超神波发送口初始化
void TRIG_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB12/13/14/15初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);//拉低
}

//前超声波距离
short forward_distance(void)
{
	u32 temp;
	short distance =0;
	
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	delay_us(20);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);

	if(TIM8CH1_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
	{
		temp=TIM8CH1_CAPTURE_STA&0X3F;
		temp*=0XFFFF;//溢出时间总和
		temp+=TIM8CH1_CAPTURE_VAL;//得到总的高电平时间
		distance = temp*170/20000;
		TIM8CH1_CAPTURE_STA=0;//开启下一次捕获
	}
	return distance;
}
//后超声波距离
short back_distance(void)
{
	u32 temp;
	short distance =0;
	
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	delay_us(20);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);

	if(TIM8CH2_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
	{
		temp=TIM8CH2_CAPTURE_STA&0X3F;
		temp*=0XFFFF;//溢出时间总和
		temp+=TIM8CH2_CAPTURE_VAL;//得到总的高电平时间
		distance = temp*170/20000;
		TIM8CH2_CAPTURE_STA=0;//开启下一次捕获
	}
	return distance;
}
//左超声波距离
short left_distance(void)
{
	u32 temp;
	short distance =0;
	
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	delay_us(20);
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);

	if(TIM8CH3_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
	{
		temp=TIM8CH3_CAPTURE_STA&0X3F;
		temp*=0XFFFF;//溢出时间总和
		temp+=TIM8CH3_CAPTURE_VAL;//得到总的高电平时间
		distance = temp*170/20000;
		TIM8CH3_CAPTURE_STA=0;//开启下一次捕获
	}
	return distance;
}
//右超声波距离
short right_distance(void)
{
	u32 temp;
	short distance =0;
	
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
	delay_us(20);
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);

	if(TIM8CH4_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
	{
		temp=TIM8CH4_CAPTURE_STA&0X3F;
		temp*=0XFFFF;//溢出时间总和
		temp+=TIM8CH4_CAPTURE_VAL;//得到总的高电平时间
		distance = temp*170/20000;
		TIM8CH4_CAPTURE_STA=0;//开启下一次捕获
	}
	return distance;
}
