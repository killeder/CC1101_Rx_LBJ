/*********************************************************************************************
项目名：USART-Utils
文件名：Usart.c - 串行口通信实现(基于串行口1) 　 
编写人：谢英男(E-mail:xieyingnan1994@163.com)　　 　
编写时间：2018年09月02日　　　　  
修改日志：　　
　　NO.1- 20180911 增加USART1_SendByte发送一字节函数
	NO.2- 20181023 USART_FLAG_TC改为USART_FLAG_TXE解决第一个字符丢掉的问题
**********************************************************************************************
说明：
**********************************************************************************************/
#include "Hardware.h"

/*-------------------------与其他模块共享的变量----------------------------*/	
uint8_t USART1_RxBuffer[USART1_RX_LENGTH];     //接收缓冲，一行USART1_RX_LENGTH字符
uint16_t USART1_RxState = 0;     //接收状态标记//bit15->接收完成标志
								   //bit14->接收到0x0d('\r')
								   //bit13~0->接收到的有效字节数目
/*---------------------------------------------------------------------------
加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
---------------------------------------------------------------------------*/
/*#pragma import(__use_no_semihosting) //标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}*/ 
/*---------------------------------------------------------------------------
使用MicroLIB的方法	  
---------------------------------------------------------------------------*/
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}	
   
    return ch;
}
int GetKey (void)  
{ 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
/*---------------------------------------------------------------------------
函数名：HW_USART1_Init
功能：USART1初始化函数
输入参数：uint32_t BaudRate - 通信波特率
返回值：无
---------------------------------------------------------------------------*/ 	   
void HW_USART1_Init(uint32_t BaudRate)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9  
	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10
	
    //USART 初始化设置
	USART_InitStructure.USART_BaudRate = BaudRate;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器
  
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	
	USART_Cmd(USART1, ENABLE);                    //使能串口1 
}
/*---------------------------------------------------------------------------
函数名：USART1_IRQHandler
功能：USART中断服务函数
输入参数：无
返回值：无
---------------------------------------------------------------------------*/ 	   
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	uint8_t Res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //如果是接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res = USART_ReceiveData(USART1);	//读取接收到的数据		
		if((USART1_RxState & 0x8000) == 0)//如果接收未完成
		{
			if(USART1_RxState & 0x4000)//接收到了0x0d(\r)
			{
				if(Res != '\n')
					USART1_RxState = 0;//接收错误,重新开始
				else 
				{
					USART1_RxBuffer[USART1_RxState&0x3fff] = '\0';	//行末尾填入字符串结束符
					USART1_RxState |= 0x8000;	//收到了0x0a(\n)接收完成了
				}
			}
			else //还没收到0x0d(\r)
			{	
				if(Res == '\r')
					USART1_RxState |= 0x4000;
				else if(isalnum(Res)||ispunct(Res))	//如果收到标点和数字和字母
				{
					USART1_RxBuffer[USART1_RxState & 0x3fff] = Res;
					USART1_RxState++;
					if(USART1_RxState >= (USART1_RX_LENGTH - 1))	//如果下个字符将占用字符串结束符0的位置
						USART1_RxState = 0;//那么接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
     }	
}
/*---------------------------------------------------------------------------
函数名：HW_USART1_SendByte
功能：USART1发送一个字节
输入参数：uint8_t dat - 要发送的数据
返回值：无
---------------------------------------------------------------------------*/ 	   
void HW_USART1_SendByte(uint8_t dat)
{
	USART_SendData(USART1,dat);
	while (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
}

