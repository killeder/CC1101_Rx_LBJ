/*-----------------------------------------------------------------------
*@file     HW_TIM.h
*@brief    定时器中断配置
*@author   谢英男(xieyingnan1994@163.com）
*@version  1.0
*@date     2019/05/15
-----------------------------------------------------------------------*/
#ifndef __HW_TIM_H
#define __HW_TIM_H
void HW_TIM_Interrupt_Init(u16 Period, u16 Prescaler);	//初始化定时器中断
void HW_TIM_Interrupt_Enable(void);	//定时器中断使能
void HW_TIM_Interrupt_Disable(void);	//定时器中断除能
void HW_TIM_Interrupt_ValueConfig(u32 Prescaler,u32 Autoreload);//重设定时器重装值

#define INT_TIM_CLK 			RCC_APB1Periph_TIM2
#define INT_TIMER 				TIM2
#define INT_TIM_PERIOD			99
#define INT_TIM_PRESCALER		7199	//(72E6/7199+1)*(99+1)=10mS
#define INT_TIMER_IRQn			TIM2_IRQn
#define INT_TIMER_IRQHandler 	TIM2_IRQHandler	

#endif
