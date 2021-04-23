#include "attitude.h"
#include "delay.h"
#include "math.h"

#define RECEIVE_BUF_SIZE  500
u8  ReceiveBuff[RECEIVE_BUF_SIZE];  
u8  Free_receive;
int Angle;
u16 USART_RX_STA3;
u8 USART_RX_BUF3[RECEIVE_BUF_SIZE];

//姿态传感器初始化
void USART3_DMA_Init(void)
{     
 	GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	DMA_InitTypeDef    DMA_InitStructure;
 
	//使能相应时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	
	//复用
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
 
	//PB11初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
 
  //串口3初始化
	USART_InitStructure.USART_BaudRate = 4800;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	
	USART_Init(USART3, &USART_InitStructure); 
 
  //使能串口3
  USART_Cmd(USART3, ENABLE);  
	
	//开启相关中断
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//空闲中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//接收中断
	
	//使能串口3DMA发送和接收
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);    
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE); 
	
  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x01;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
 
	//配置USART3—DMA接收
	DMA_DeInit(DMA1_Stream1);
	//等待DMA可配置
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
	//通道选择USART3-RX
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
	//DMA外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;
	//DMA存储器0地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ReceiveBuff;
	//外设到存储器模式
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	//数据传输量
	DMA_InitStructure.DMA_BufferSize = RECEIVE_BUF_SIZE;
	//外设非增量模式
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//存储器增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//外设数据长度8位
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//存储器数据长度8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//使用普通模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//中等优先级
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	//FIFO失能
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	//存储器突发单次传输
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	//外设突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//初始化DMA
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);
	
	//使能DMA
	DMA_Cmd(DMA1_Stream1, ENABLE);  
}


//校验和
u8 Check_Sum(void)
{
	u8 temp =0,i;
	for(i = 0;i<10;i++)
	{
		temp += ReceiveBuff[i];
	}
	return temp;
}

//串口3中断服务函数
void USART3_IRQHandler(void)
{
	short temp;

	//接收中断
	if(USART_GetITStatus(USART3,USART_IT_IDLE) != RESET) 
	{ 
		Free_receive=USART3->DR;
		USART_ClearITPendingBit(USART3,USART_IT_IDLE|USART_IT_RXNE);
		//DMA1_Stream1关闭，准备重新配置
		DMA_Cmd(DMA1_Stream1,DISABLE); 
		
		//清除DMA1_Stream1标志位
		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);
		
		//数据接收完成
		if(ReceiveBuff[10] == Check_Sum())
		{
			temp = ReceiveBuff[7]<<8|ReceiveBuff[6];
			Angle = temp;
			Angle *=180;
			Angle /= 32768.0;
			Angle += 180;
		}
		
		//设置接收数据量
		DMA_SetCurrDataCounter(DMA1_Stream1, RECEIVE_BUF_SIZE);  
		
		//开启下一次DMA传输
		DMA_Cmd(DMA1_Stream1, ENABLE);  
	}
	
	//清除中断标志位
	
}
float Position_KP = 1,Position_KI = 0,Position_KD = 0;
int Dir_PID(int Aim_Dir,int Rel_Dir)
{
		static float Bias,Integral_bias,Last_Bias;
		int pwm = 0;;
		if(Aim_Dir> Rel_Dir)
		{
			if(abs(Aim_Dir-Rel_Dir)>abs(Aim_Dir-Rel_Dir-360))
			{
					Bias = abs(Aim_Dir-Rel_Dir-360);
			}else
			{
					Bias = -1*abs(Aim_Dir-Rel_Dir);
			}
		}
		else 
		{
			if(abs(Aim_Dir-Rel_Dir)>abs(Rel_Dir-Aim_Dir-360))
			{
				
					Bias = -1*abs(Rel_Dir-Aim_Dir-360);
			}else 
			{
					Bias = abs(Aim_Dir-Rel_Dir);
			}
		}
		Integral_bias+=Bias;
		pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias); 
		Last_Bias=Bias;
		if(pwm > 60) 	pwm = 60;
		if(pwm < -60) 	pwm = -60;
		return pwm; 
}
