#include "attitude.h"
#include "delay.h"
#include "math.h"

#define RECEIVE_BUF_SIZE  500
u8  ReceiveBuff[RECEIVE_BUF_SIZE];  
u8  Free_receive;
int Angle;
u16 USART_RX_STA3;
u8 USART_RX_BUF3[RECEIVE_BUF_SIZE];

//��̬��������ʼ��
void USART3_DMA_Init(void)
{     
 	GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	DMA_InitTypeDef    DMA_InitStructure;
 
	//ʹ����Ӧʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	
	//����
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
 
	//PB11��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
 
  //����3��ʼ��
	USART_InitStructure.USART_BaudRate = 4800;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	
	USART_Init(USART3, &USART_InitStructure); 
 
  //ʹ�ܴ���3
  USART_Cmd(USART3, ENABLE);  
	
	//��������ж�
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//�����ж�
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�
	
	//ʹ�ܴ���3DMA���ͺͽ���
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);    
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE); 
	
  //Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x01;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
 
	//����USART3��DMA����
	DMA_DeInit(DMA1_Stream1);
	//�ȴ�DMA������
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
	//ͨ��ѡ��USART3-RX
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
	//DMA�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;
	//DMA�洢��0��ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ReceiveBuff;
	//���赽�洢��ģʽ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	//���ݴ�����
	DMA_InitStructure.DMA_BufferSize = RECEIVE_BUF_SIZE;
	//���������ģʽ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//�洢������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//�������ݳ���8λ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//�洢�����ݳ���8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//ʹ����ͨģʽ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//�е����ȼ�
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	//FIFOʧ��
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	//����ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//��ʼ��DMA
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);
	
	//ʹ��DMA
	DMA_Cmd(DMA1_Stream1, ENABLE);  
}


//У���
u8 Check_Sum(void)
{
	u8 temp =0,i;
	for(i = 0;i<10;i++)
	{
		temp += ReceiveBuff[i];
	}
	return temp;
}

//����3�жϷ�����
void USART3_IRQHandler(void)
{
	short temp;

	//�����ж�
	if(USART_GetITStatus(USART3,USART_IT_IDLE) != RESET) 
	{ 
		Free_receive=USART3->DR;
		USART_ClearITPendingBit(USART3,USART_IT_IDLE|USART_IT_RXNE);
		//DMA1_Stream1�رգ�׼����������
		DMA_Cmd(DMA1_Stream1,DISABLE); 
		
		//���DMA1_Stream1��־λ
		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);
		
		//���ݽ������
		if(ReceiveBuff[10] == Check_Sum())
		{
			temp = ReceiveBuff[7]<<8|ReceiveBuff[6];
			Angle = temp;
			Angle *=180;
			Angle /= 32768.0;
			Angle += 180;
		}
		
		//���ý���������
		DMA_SetCurrDataCounter(DMA1_Stream1, RECEIVE_BUF_SIZE);  
		
		//������һ��DMA����
		DMA_Cmd(DMA1_Stream1, ENABLE);  
	}
	
	//����жϱ�־λ
	
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
