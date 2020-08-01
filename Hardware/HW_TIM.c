/*-----------------------------------------------------------------------
*@file     HW_TIM.c
*@brief    定时器中断配置
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2019/05/15
-----------------------------------------------------------------------*/
#include "Hardware.h"
/*-----------------------------------------------------------------------
*@brief		定时器中断配置
*@param		Period 周期
*@param		Prescaler 预分频数
*@retval	无
-----------------------------------------------------------------------*/
void HW_TIM_Interrupt_Init(u16 Period, u16 Prescaler)		  
{	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(INT_TIM_CLK,ENABLE); //时钟使能
	//定时器初始化
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;					 					
	TIM_TimeBaseStructure.TIM_Period = Period;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(INT_TIMER, &TIM_TimeBaseStructure); //根据指定的参数初始化时间基数单位
	TIM_ITConfig(INT_TIMER,TIM_IT_Update,ENABLE); //使能指定中断,允许更新中断
	TIM_ClearITPendingBit(INT_TIMER, TIM_IT_Update);  //清除更新中断标志
	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = INT_TIMER_IRQn;  //中断线
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
}
/*-----------------------------------------------------------------------
*@brief		定时器中断使能
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void HW_TIM_Interrupt_Enable(void)
{
	TIM_Cmd(INT_TIMER,ENABLE);	
}
/*-----------------------------------------------------------------------
*@brief		定时器中断除能
*@param		无
*@retval	无
-----------------------------------------------------------------------*/
void HW_TIM_Interrupt_Disable(void)
{
	TIM_Cmd(INT_TIMER,DISABLE);	
}
/*-----------------------------------------------------------------------
*@brief		重设定时器重装值
*@param		Prescaler 预分频数
*@param 	Autoload  周期
*@retval	无
-----------------------------------------------------------------------*/
void HW_TIM_Interrupt_ValueConfig(u32 Prescaler,u32 Autoreload)
{
	TIM_PrescalerConfig(INT_TIMER,(uint16_t)Prescaler,TIM_PSCReloadMode_Immediate);
	TIM_SetAutoreload(INT_TIMER,(uint16_t)Autoreload);			
}
