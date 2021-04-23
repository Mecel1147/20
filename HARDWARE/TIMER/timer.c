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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);//TIM8ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//ʹ��PORTCʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//GPIOC6/7/8/9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC6/7/8/9

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);//GPIO6����λ��ʱ��8
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);//GPIO7����λ��ʱ��8
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);//GPIO8����λ��ʱ��8
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);//GPIO9����λ��ʱ��8
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;//��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=65535;//�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
	
	//��ʼ��TIM8���벶�����
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//�����ز���
  TIM8_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//ӳ�䵽TI1��
  TIM8_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;//���������Ƶ,����Ƶ 
  TIM8_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM8,&TIM8_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_2; 
  TIM_ICInit(TIM8,&TIM8_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_3; 
  TIM_ICInit(TIM8,&TIM8_ICInitStructure);
	TIM8_ICInitStructure.TIM_Channel = TIM_Channel_4; 
  TIM_ICInit(TIM8,&TIM8_ICInitStructure);
		
	TIM_ClearFlag(TIM8,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
	TIM_ITConfig(TIM8,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//��������ж� ,����CC1/2/3/4IE�����ж�	
	
  TIM_Cmd(TIM8,ENABLE);//ʹ�ܶ�ʱ��8
  //�ж�����
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x00;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

}  
u8  TIM8CH1_CAPTURE_STA=0;//���벶��״̬		    				
u16	TIM8CH1_CAPTURE_VAL;//���벶��ֵ
u8  TIM8CH2_CAPTURE_STA=0;//���벶��״̬		    				
u16	TIM8CH2_CAPTURE_VAL;//���벶��ֵ
u8  TIM8CH3_CAPTURE_STA=0;//���벶��״̬		    				
u16	TIM8CH3_CAPTURE_VAL;//���벶��ֵ
u8  TIM8CH4_CAPTURE_STA=0;//���벶��״̬		    				
u16	TIM8CH4_CAPTURE_VAL;//���벶��ֵ	
 

//��ʱ��8�жϷ������	 
void TIM8_CC_IRQHandler(void)
{ 
	//ǰ������
 	if((TIM8CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)	 
		{	    
			if(TIM8CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM8CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM8CH1_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM8CH1_CAPTURE_VAL=0XFFFF;
				}else TIM8CH1_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM8,TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(TIM8CH1_CAPTURE_STA&0X40)//����һ���½��� 		
			{	  			
				TIM8CH1_CAPTURE_STA|=0X80;//��ǳɹ�����һ��������
				TIM8CH1_CAPTURE_VAL=TIM_GetCapture1(TIM8);
		   	TIM_OC1PolarityConfig(TIM8,TIM_ICPolarity_Rising);//CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM8CH1_CAPTURE_STA=0;//���
				TIM8CH1_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM8,0);
				TIM8CH1_CAPTURE_STA|=0X40;//��ǲ�����������
		   	TIM_OC1PolarityConfig(TIM8,TIM_ICPolarity_Falling);//CC1P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
 	}
	
	//������
 	if((TIM8CH2_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)	 
		{	    
			if(TIM8CH2_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM8CH2_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM8CH2_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM8CH2_CAPTURE_VAL=0XFFFF;
				}else TIM8CH2_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM8,TIM_IT_CC2) != RESET)//����2���������¼�
		{	
			if(TIM8CH2_CAPTURE_STA&0X40)//����һ���½��� 		
			{	  			
				TIM8CH2_CAPTURE_STA|=0X80;//��ǳɹ�����һ��������
				TIM8CH2_CAPTURE_VAL=TIM_GetCapture2(TIM8);
		   	TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Rising);//CC2P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM8CH2_CAPTURE_STA=0;//���
				TIM8CH2_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM8,0);
				TIM8CH2_CAPTURE_STA|=0X40;//��ǲ�����������
		   	TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Falling);//CC2P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
 	}	

	//������
 	if((TIM8CH3_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)	 
		{	    
			if(TIM8CH3_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM8CH3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM8CH3_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM8CH3_CAPTURE_VAL=0XFFFF;
				}else TIM8CH3_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM8,TIM_IT_CC3) != RESET)//����3���������¼�
		{	
			if(TIM8CH3_CAPTURE_STA&0X40)//����һ���½��� 		
			{	  			
				TIM8CH3_CAPTURE_STA|=0X80;//��ǳɹ�����һ��������
				TIM8CH3_CAPTURE_VAL=TIM_GetCapture3(TIM8);
		   	TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Rising);//CC3P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM8CH3_CAPTURE_STA=0;//���
				TIM8CH3_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM8,0);
				TIM8CH3_CAPTURE_STA|=0X40;//��ǲ�����������
		   	TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Falling);//CC3P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
 	}	
	
	//�ҳ�����
 	if((TIM8CH4_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)	 
		{	    
			if(TIM8CH4_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM8CH4_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM8CH4_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM8CH4_CAPTURE_VAL=0XFFFF;
				}else TIM8CH4_CAPTURE_STA++;
			}	 
		}
		if (TIM_GetITStatus(TIM8,TIM_IT_CC4) != RESET)//����4���������¼�
		{	
			if(TIM8CH4_CAPTURE_STA&0X40)//����һ���½��� 		
			{	  			
				TIM8CH4_CAPTURE_STA|=0X80;//��ǳɹ�����һ��������
				TIM8CH4_CAPTURE_VAL=TIM_GetCapture4(TIM8);
		   	TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Rising);//CC4P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM8CH4_CAPTURE_STA=0;//���
				TIM8CH4_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM8,0);
				TIM8CH4_CAPTURE_STA|=0X40;//��ǲ�����������
		   	TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Falling);//CC4P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
 	}	
	
  TIM_ClearITPendingBit(TIM8,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update); //����жϱ�־λ
}


//���񲨷��Ϳڳ�ʼ��
void TRIG_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB12/13/14/15��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB,&GPIO_InitStructure);//��ʼ��
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);//����
}

//ǰ����������
short forward_distance(void)
{
	u32 temp;
	short distance =0;
	
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	delay_us(20);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);

	if(TIM8CH1_CAPTURE_STA&0X80)//�ɹ�������һ��������
	{
		temp=TIM8CH1_CAPTURE_STA&0X3F;
		temp*=0XFFFF;//���ʱ���ܺ�
		temp+=TIM8CH1_CAPTURE_VAL;//�õ��ܵĸߵ�ƽʱ��
		distance = temp*170/20000;
		TIM8CH1_CAPTURE_STA=0;//������һ�β���
	}
	return distance;
}
//����������
short back_distance(void)
{
	u32 temp;
	short distance =0;
	
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	delay_us(20);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);

	if(TIM8CH2_CAPTURE_STA&0X80)//�ɹ�������һ��������
	{
		temp=TIM8CH2_CAPTURE_STA&0X3F;
		temp*=0XFFFF;//���ʱ���ܺ�
		temp+=TIM8CH2_CAPTURE_VAL;//�õ��ܵĸߵ�ƽʱ��
		distance = temp*170/20000;
		TIM8CH2_CAPTURE_STA=0;//������һ�β���
	}
	return distance;
}
//����������
short left_distance(void)
{
	u32 temp;
	short distance =0;
	
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	delay_us(20);
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);

	if(TIM8CH3_CAPTURE_STA&0X80)//�ɹ�������һ��������
	{
		temp=TIM8CH3_CAPTURE_STA&0X3F;
		temp*=0XFFFF;//���ʱ���ܺ�
		temp+=TIM8CH3_CAPTURE_VAL;//�õ��ܵĸߵ�ƽʱ��
		distance = temp*170/20000;
		TIM8CH3_CAPTURE_STA=0;//������һ�β���
	}
	return distance;
}
//�ҳ���������
short right_distance(void)
{
	u32 temp;
	short distance =0;
	
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
	delay_us(20);
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);

	if(TIM8CH4_CAPTURE_STA&0X80)//�ɹ�������һ��������
	{
		temp=TIM8CH4_CAPTURE_STA&0X3F;
		temp*=0XFFFF;//���ʱ���ܺ�
		temp+=TIM8CH4_CAPTURE_VAL;//�õ��ܵĸߵ�ƽʱ��
		distance = temp*170/20000;
		TIM8CH4_CAPTURE_STA=0;//������һ�β���
	}
	return distance;
}
