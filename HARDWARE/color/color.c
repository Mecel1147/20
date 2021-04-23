#include "color.h"

//unsigned char color_date = '0';//颜色数据
u16 color_number = 0;
u32 OPENMV_Data[4];

//初始化串口3
void UART4_Init(u32 bound)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART4时钟
 
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);//GPIOC11复用为USART4
	
	//UART4端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);//初始化PC11

  //USART4 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
  USART_Init(UART4,&USART_InitStructure);//初始化串口4
	
  USART_Cmd(UART4, ENABLE);//使能串口4
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart4 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口4中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}

//颜色识别标志信号
void Color_Init(void)   
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);   
	
	COLOR_JUDGE = 0;//拉低颜色识别标志信号 
}


////串口3中断服务程序    接收一位字符串
//void UART4_IRQHandler(void)                
//{
//	if(USART_GetITStatus(UART4,USART_IT_RXNE) != RESET)  
//	{
//		color_date = USART_ReceiveData(UART4);
//	} 
//} 


//串口3中断服务程序    结束信息数组
void UART4_IRQHandler(void)                
{
static u8 rebuf[3] = {0},i = 0;
		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  
		{	
				rebuf[i++]=USART_ReceiveData(UART4);  //USART_ReceiveData(),串口函数，串口接收的是一字节
				if(rebuf[0] != 0xff)
        { 
				 color_number = 0;
				 i = 0;
				}		//判断帧头
				if(i > 1)
				{		
					OPENMV_Data[0] = rebuf[0];
					OPENMV_Data[1] = rebuf[1];   
					OPENMV_Data[2] = rebuf[2];   
					color_number = rebuf[1];     //颜色对应数字  红-2  蓝-3
					
					if(rebuf[2] == 0xfe)
					{
						i = 0;
					}
//					else
//					{
//						OPENMV_Data[0] = rebuf[0];
//						OPENMV_Data[1] = rebuf[1];   
//						OPENMV_Data[2] = rebuf[2];   
//						color_number = rebuf[1];     //颜色对应数字  红-2  蓝-1
//					}
				}
		
		}
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);

}