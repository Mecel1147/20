#include "color.h"

//unsigned char color_date = '0';//��ɫ����
u16 color_number = 0;
u32 OPENMV_Data[4];

//��ʼ������3
void UART4_Init(u32 bound)
{
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��UART4ʱ��
 
	//����4��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);//GPIOC11����ΪUSART4
	
	//UART4�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);//��ʼ��PC11

  //USART4 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ
  USART_Init(UART4,&USART_InitStructure);//��ʼ������4
	
  USART_Cmd(UART4, ENABLE);//ʹ�ܴ���4
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart4 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//����4�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

//��ɫʶ���־�ź�
void Color_Init(void)   
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);   
	
	COLOR_JUDGE = 0;//������ɫʶ���־�ź� 
}


////����3�жϷ������    ����һλ�ַ���
//void UART4_IRQHandler(void)                
//{
//	if(USART_GetITStatus(UART4,USART_IT_RXNE) != RESET)  
//	{
//		color_date = USART_ReceiveData(UART4);
//	} 
//} 


//����3�жϷ������    ������Ϣ����
void UART4_IRQHandler(void)                
{
static u8 rebuf[3] = {0},i = 0;
		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  
		{	
				rebuf[i++]=USART_ReceiveData(UART4);  //USART_ReceiveData(),���ں��������ڽ��յ���һ�ֽ�
				if(rebuf[0] != 0xff)
        { 
				 color_number = 0;
				 i = 0;
				}		//�ж�֡ͷ
				if(i > 1)
				{		
					OPENMV_Data[0] = rebuf[0];
					OPENMV_Data[1] = rebuf[1];   
					OPENMV_Data[2] = rebuf[2];   
					color_number = rebuf[1];     //��ɫ��Ӧ����  ��-2  ��-3
					
					if(rebuf[2] == 0xfe)
					{
						i = 0;
					}
//					else
//					{
//						OPENMV_Data[0] = rebuf[0];
//						OPENMV_Data[1] = rebuf[1];   
//						OPENMV_Data[2] = rebuf[2];   
//						color_number = rebuf[1];     //��ɫ��Ӧ����  ��-2  ��-1
//					}
				}
		
		}
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);

}