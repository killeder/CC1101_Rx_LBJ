/*-----------------------------------------------------------------------
*@file     HW_GPIO.c
*@brief    IO端口的初始化和应用
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2019/06/20
-----------------------------------------------------------------------*/
#include "Hardware.h"
BEEPER_MODE BeeperMode = BEEP_OFF;	//蜂鸣器的蜂鸣方式
BLINK_MODE StatusBlinkMode = BLINK_OFF;	//状态指示灯闪烁模式 
BLINK_MODE InfoBlinkMode = BLINK_OFF;	//消息指示灯闪烁模式 
/*-----------------------------------------------------------------------
*@brief		GPIO初始化为推挽输出模式
*@param		GPIO时钟
*@param		GPIO端口
*@param		GPIO引脚
*@param 	GPIO输出模式(推挽/开漏)
*@retval	无
-----------------------------------------------------------------------*/
void HW_GPIO_Init_Out(u32 gpio_clk,GPIO_TypeDef * gpio,u16 gpio_pin,
													GPIOMode_TypeDef mode)			
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(gpio_clk,ENABLE);	 			//使能端口时钟
	GPIO_InitStructure.GPIO_Pin = gpio_pin;						//端口配置
	GPIO_InitStructure.GPIO_Mode = mode; 		 	//输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	//IO口速度为50MHz
	GPIO_Init(gpio, &GPIO_InitStructure);					//根据设定参数初始化
}
/*-----------------------------------------------------------------------
*@brief		GPIO初始化为上拉输入模式
*@param		GPIO时钟
*@param		GPIO端口
*@param		GPIO引脚
*@retval	无
-----------------------------------------------------------------------*/
void HW_GPIO_Init_In(u32 gpio_clk,GPIO_TypeDef * gpio,u16 gpio_pin)			
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(gpio_clk,ENABLE);	 			//使能端口时钟
	GPIO_InitStructure.GPIO_Pin = gpio_pin;						//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 		//上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	//IO口速度为50MHz
	GPIO_Init(gpio, &GPIO_InitStructure);					//根据设定参数初始化
}
