#include "grayscale.h"

u16 ADC_Data[10] = {0};	

u16 boundary = 3200;
//u16 grayscale1 = 0;
//u16 grayscale2 = 0;
//u16 grayscale3 = 0;
//u16 grayscale4 = 0;
//u16 grayscale5 = 0;
//u16 grayscale6 = 0;
//u16 grayscale7 = 0;
//u16 grayscale8 = 0;
//u16 grayscale9 = 0;
//u16 grayscale10 = 0;
//u16 grayscale11 = 0;
//u16 grayscale12 = 0;

void ADC_DMA_Init(void)
{    
	//结构体定义
	DMA_InitTypeDef DMA_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;	
	GPIO_InitTypeDef  GPIO_InitStructure;

	//DMA2时钟使能
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); 
	//ADC1时钟使能
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	//GPIOA/B/C时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//不带上下拉
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//PA4/5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//PB0/1 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//PC0/1/2/3/4/5 


	//DMA2_Stream4
	DMA_DeInit(DMA2_Stream4);
	//等待DMA可配置
	while (DMA_GetCmdStatus(DMA2_Stream4) != DISABLE);
	//通道选择
  DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
	//DMA外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&ADC1->DR);	
	//DMA存储器0地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ADC_Data;  
  // //dma传输方向---外设到存储器模式
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
	//设置DMA在传输时缓冲区的长度  
	DMA_InitStructure.DMA_BufferSize = 12;	
	//设置DMA的外设递增模式，一个外设
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  //设置DMA的内存递增模式  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
  //外设数据字长 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
  //内存数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	
	//设置DMA的传输模式 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  //设置DMA的优先级别 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  //指定如果FIFO模式或直接模式将用于指定的流:不使能FIFO模式
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  
  //指定了FIFO阈值水平 
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  //指定的Burst转移配置内存传输  
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  //指定的Burst转移配置外围转移 */ 
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
	//配置DMA的通道   
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);
	//使能通道
  DMA_Cmd(DMA2_Stream4, ENABLE);

	//ADC1复位和复位结束
  ADC_DeInit();

  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//精度为12位 
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;//扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
	ADC_InitStructure.ADC_NbrOfConversion = 12;//用于转换的通道数
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_Init(ADC1, &ADC_InitStructure);//初始化	

	//设置指定ADC的规则通组通道，一个序列，采样时间 
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_2,1,ADC_SampleTime_15Cycles);   //PA2     ADC_Data[0]
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_3,2,ADC_SampleTime_15Cycles);    //PA3
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4,1,ADC_SampleTime_15Cycles);    //PA4      ADC_Data[0]  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5,2,ADC_SampleTime_15Cycles);		//PA5			 ADC_Data[1]
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8,3,ADC_SampleTime_15Cycles);    //PB0			 ADC_Data[2]
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9,4,ADC_SampleTime_15Cycles);    //PB1			 ADC_Data[3]
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10,5,ADC_SampleTime_15Cycles); 	//PC0
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11,6,ADC_SampleTime_15Cycles); 	//PC1
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12,7,ADC_SampleTime_15Cycles); 	//PC2
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13,8,ADC_SampleTime_15Cycles);	//PC3
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14,9,ADC_SampleTime_15Cycles); 	//PC4
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15,10,ADC_SampleTime_15Cycles); 	//PC5

  //使能ADC的DMA  
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); 
  ADC_DMACmd(ADC1, ENABLE);
	//开启AD转换器  
  ADC_Cmd(ADC1, ENABLE);
	//使能指定的ADC软件
	ADC_SoftwareStartConv(ADC1);
}	

//PC0
int grayscale1(void)
{
	if(ADC_Data[4] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}
//PC1
int grayscale2(void)
{
	if(ADC_Data[5] > boundary+500)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}

//PC2
int grayscale3(void)
{
	if(ADC_Data[6] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}
//PC3
int grayscale4(void)
{
	if(ADC_Data[7] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}
//PC4
int grayscale5(void)
{
	if(ADC_Data[8] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}
//PC5
int grayscale6(void)
{
	if(ADC_Data[9] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}

//pA2
/*int grayscale7(void)
{
	if(ADC_Data[0] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}
//pA3
int grayscale8(void)
{
	if(ADC_Data[1] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}*/


//pA4
int grayscale7(void)
{
	if(ADC_Data[0] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}

//PA5
int grayscale8(void)
{
	if(ADC_Data[1] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}

//PB0
int grayscale9(void)
{
	if(ADC_Data[2] > boundary)//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}

//PB1
int grayscale10(void)
{
	if(ADC_Data[3] > (boundary))//黑色
	{
		return 1;
	}
	else//白色
	{
		return 0;
	}
}