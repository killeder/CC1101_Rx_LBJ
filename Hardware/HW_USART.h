/*********************************************************************************************
项目名：USART-Utils
文件名：Usart.h - 串行口通信(基于串行口1)头文件 
编写人：谢英男(E-mail:xieyingnan1994@163.com)　　 　
编写时间：2018年09月02日　　　　  
修改日志：　　
　　NO.1-								
**********************************************************************************************
说明：
**********************************************************************************************/
#ifndef HW_USART_H
#define HW_USART_H

void HW_USART1_Init(uint32_t BaudRate);	//USART1初始化函数
void HW_USART1_SendByte(uint8_t dat);	//USART1发送一个字节数据

#define USART1_RX_LENGTH	81
#define USART1_RXCOMPLETE_FLAG	0x8000	//USART1接收完成标志，与变量USART1_RxState配合，二者相与后
										//不为0表示接收完成
extern uint16_t USART1_RxState;			//USART1接收状态变量
extern uint8_t USART1_RxBuffer[];		//接收缓冲

#endif
